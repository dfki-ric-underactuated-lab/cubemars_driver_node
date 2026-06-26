# CubeMars Driver Node - Software Architecture (`main` branch)

This document describes how the `main` branch of the `cubemars_hardware_interface` package is laid
out: how the communication layer and the functional layer are separated, and how the Robot Operating
System 2 (ROS 2) node works. It is the reference for the integration effort in which `main` gains
support for MAB electronics as a second, configuration-selectable communication backend; the
`MAB_FDCAN` branch's protocol work is absorbed into `main`, not the reverse. See
[MAB_FDCAN_PARITY.md](MAB_FDCAN_PARITY.md) for the branch comparison and the integration plan.

Status reference: written against `main` at commit `837e408` ("more things"), 2026-06-24.

---

## 1. Purpose and scope

The package is a hardware driver for CubeMars (T-Motor) quasi-direct-drive actuators that speak over a
Controller Area Network (CAN) bus. It exposes the motors to the rest of a robot stack so that a
controller can send joint commands (position, velocity, effort, and the impedance gains `kp`/`kd`) and
read back joint states (position, velocity, effort, temperature).

The driver supports two generations of CubeMars electronics:

- **V2** motors, controlled with the MIT-style impedance protocol over classic CAN (8-byte frames,
  fixed-point bit-packed fields).
- **V3** motors, controlled with the CubeMars extended-frame protocol (29-bit identifiers, a dedicated
  response packet, and physical-unit conversions using pole-pair count, gear ratio, and torque
  constant).

The bus is Linux SocketCAN throughout. There is no vendor software development kit (SDK); all framing
is done directly against `<linux/can.h>`.

---

## 2. The big picture: a layered architecture

This repository drives the motors through a single entry point: the standalone **lifecycle node**
`CubeMarsHardwareNode`. The node is built on a clean separation between a functional layer and a
communication layer, joined by a shared set of data types.

```
                    ROS 2 controllers / higher-level stack
                                   |
                                   v
       FUNCTIONAL LAYER  ->  the lifecycle node
       CubeMarsHardwareNode
       src/cubemars_hardware_node.cpp (1672 ln)
       - lifecycle, dedicated comm threads, filters,
         latency stats, dynamic reconfigure, services
                                   |
                                   v
       COMMUNICATION LAYER  ->  cubemars::CubemarsCan
       src/cubemars_can.cpp (648 ln)
       - SocketCAN, V2/V3 framing, send/receive cycle, timestamping
                                   |
                                   v
       SHARED DATA CONTRACT (header-only)
       include/.../cubemars_com.hpp
       - joint_config_t, joint_cmd_t, joint_state_t
       - JointMode, ErrorCode, ComStatus, CAN_PACKET_ID, SERIES_TYPE
       - per-motor-type config map, MIT magic-byte sequences
                                   |
                                   v
              Linux SocketCAN (can0, can1, ...)
                                   |
                          CubeMars motors
```

Key consequences of this layout:

- The **functional layer** (what the node does: lifecycle, threading, filtering, diagnostics,
  services) is cleanly separated from the **communication layer** (how bytes get on and off the bus).
- The node never touches a socket directly. It talks to motors only through the `cubemars::CubemarsCan`
  class and the shared structs in `cubemars_com.hpp`. **That boundary is the seam along which a
  different electronics protocol (such as the MAB FDCAN protocol) is swapped in.**

The remainder of this document drills into each layer.

---

## 3. The shared data contract (`cubemars_com.hpp`)

This header-only file is the contract that the node and the communication class agree on. It defines no
behavior beyond small `constexpr` string helpers; it is pure types and constants.

### 3.1 Enumerations

- `JointMode` - `UNDEFINED, POSITION, VELOCITY, EFFORT, ERROR`. A command-mode enumeration carried in
  the shared types.
- `ErrorCode` - device fault codes reported by the motor (`NO_FAULT, MOTOR_OVER_TEMP, OVER_CURRENT,
  OVER_VOLTAGE, UNDER_VOLTAGE, ENCODER_FAULT, MOSFET_OVER_TEMP, MOTOR_STALL`) with a `constexpr
  errorFlagToString`.
- `ComStatus` - per-joint communication outcome of one bus cycle: `SUCCESS, CAN_WRITE_FAILED,
  CAN_READ_FAILED, CAN_NO_RESPONSE, CAN_WRITE_FAILED_BUT_RESPONSE_RECEIVED`. This is how the comm layer
  reports partial failures back to the node without throwing.
- `CAN_PACKET_ID` - the CubeMars V3 command identifiers (`SET_DUTY, SET_CURRENT, ..., SET_POS,
  SET_ORIGIN_HERE, SET_POS_SPD, SET_MIT = 8, RESPONSE = 0x29`).
- `SERIES_TYPE` - `V2` or `V3`, the per-motor protocol selector.

### 3.2 Structs

- `joint_config_t` - the static description of one motor: `can_id`, name, the per-quantity min/max
  ranges (`P`, `V`, `I`, `KP`, `KD`), `invert`, `reply_on_own_id`, `series_type`, and the V3-only
  conversion constants `numer_of_pole_pairs`, `gear_ratio`, `torque_constant`,
  `current_mesurement_has_sign`.
- `joint_cmd_t` - one outgoing command: `float pos, vel, torque, kp, kd`.
- `joint_state_t` - one incoming state: `float pos, vel, torque, temp`, plus
  `ErrorCode device_status`, `ComStatus communication_status`, `int com_errno`, and five timing fields:
  `rx_timestamp_ns` (kernel software receive timestamp, `CLOCK_REALTIME`), `send_timestamp_ns` (kernel
  software transmit-completion timestamp), `dequeue_timestamp_ns` (user-space arrival time),
  `enqueue_timestamp_ns` (`CLOCK_REALTIME` taken right after the command frame was written into the
  transmit buffer), and `rx_hw_timestamp_ns` (the network card's raw hardware-clock receive timestamp,
  a free-running clock that is NOT `CLOCK_REALTIME`). These timing fields are what feed the node's
  latency diagnostics.

### 3.3 Constants

- The three V2 MIT magic-byte sequences: `START_MOTOR_CONTROL_MODE` (`...0xFC`),
  `EXIT_MOTOR_CONTROL_MODE` (`...0xFD`), `SET_ZERO_POSITION` (`...0xFE`).
- `joint_config_per_motor_type` - a `std::map` of named presets (`AK10-9`, `AK80-6`, `AK10-9v3`,
  `AK80-8v3`, and variants) giving each motor type its ranges, series, pole pairs, gear ratio, and
  torque constant. The node looks a motor up here by the `motor_type` string in the configuration.

---

## 4. The communication layer: `cubemars::CubemarsCan`

File: `src/cubemars_can.cpp` (648 lines), header `include/cubemars_hardware_interface/cubemars_can.hpp`.

One `CubemarsCan` object owns one SocketCAN socket bound to one CAN interface (for example `can0`) and
manages all the motors on that interface. The node creates one instance per CAN interface.

### 4.1 Public interface (the abstraction boundary)

This is the entire surface the node depends on. Keeping a future MAB implementation interface-compatible
with this list is what makes the node reusable.

```cpp
CubemarsCan(const std::string &can_interface,
            const int &enable_loopback,
            const std::vector<joint_config_t> &joint_configs,
            const long &socket_timeout_sec,
            const long &socket_timeout_usec,
            unsigned int max_init_connect_trials,
            bool enable_tx_timestamping = true,
            bool enable_can_error_frames = false);

void start_motor_control_mode(unsigned int joint_id, bool set_zero_position_on_enable);
void end_motor_control_mode(unsigned int joint_id);
void start_motor_control_mode(bool set_zero_position_on_enable);   // all joints
void end_motor_control_mode();                                     // all joints

void send_and_receive(const std::vector<joint_cmd_t> &cmds,
                      std::vector<joint_state_t> &states);

const std::string &GetName();
canid_t get_can_id(unsigned int joint_index);

int64_t get_tx_fill_duration_ns() const;   // time to write all command frames into the TX buffer
int64_t get_rx_duration_ns() const;        // time to receive all replies after the TX buffer filled
int64_t get_tx_fill_end_ns() const;        // absolute timestamp the TX fill finished (reply timing origin)
unsigned int get_last_tx_errq_count() const;  // error-queue entries drained in the last cycle
unsigned int get_last_error_count() const;    // CAN bus-error frames seen in the last send_and_receive()
canid_t get_last_error_canid() const;         // can_id (error-class bits) of the most recent error frame
```

Everything else (frame packing, timestamp extraction, the V2/V3 helpers, the fixed-point converters,
the socket members) is private.

### 4.2 Construction and socket setup

The constructor opens a raw SocketCAN socket (`socket(AF_CAN, SOCK_RAW, CAN_RAW)`) and configures it
step by step before any motor traffic. This layer uses **classic CAN** frames; it does not enable CAN
with Flexible Data-rate (CAN FD). The configuration steps, in order:

- **`CAN_RAW_LOOPBACK`** (a `setsockopt` on `SOL_CAN_RAW`, wired to the `enable_loopback` parameter).
  SocketCAN's *local loopback*: when enabled, every frame this socket transmits is also delivered by
  the kernel to the *other* local sockets bound to the same CAN interface, so several processes on the
  same host can observe each other's traffic. It does not echo a frame back to the sending socket
  itself unless `CAN_RAW_RECV_OWN_MSGS` is also set (this driver does not set it). Loopback is on by
  default in the kernel; exposing it as a parameter lets a deployment turn it off when nothing else on
  the host needs to see the bus.
- **`CAN_RAW_FILTER`** (a `setsockopt` on `SOL_CAN_RAW` taking an array of `can_filter` structs, one
  `{can_id, can_mask}` pair each). The kernel then delivers a received frame to this socket only if it
  matches at least one filter, that is `(frame.can_id & mask) == (filter.can_id & mask)`. The driver
  installs one filter per configured motor, so the socket wakes only for actual replies and never for
  unrelated bus traffic. The filter identifier depends on protocol generation: V2 motors filter on
  either their own identifier or on identifier 0 (depending on `reply_on_own_id`); V3 motors filter on
  the extended-frame response identifier `can_id | (RESPONSE << 8)`. CAN bus-error frames are matched
  by a separate error filter (below), not by this data filter.
- **`SIOCGIFINDEX`** (an `ioctl`, "socket I/O control: get interface index"). SocketCAN's `bind()`
  needs a numeric *interface index*, not the textual name. The driver fills an `ifreq` with the
  interface name (for example `"can0"`), calls `ioctl(fd, SIOCGIFINDEX, &ifr)` to resolve it to the
  kernel's integer index, and then binds the socket to that specific index. Binding to index 0 would
  mean "all CAN interfaces"; the driver always binds to one named interface.
- **`bind`**: attaches the socket to the resolved interface index through a `sockaddr_can`.
- **`SO_RCVTIMEO`** (receive timeout, from the constructor's `socket_timeout_sec`/`_usec` arguments).
  Bounds each blocking receive so a missing reply surfaces as a timeout (`CAN_READ_FAILED`) instead of
  blocking the cycle forever.
- **`SO_TIMESTAMPNS`** (always on): kernel *software* receive timestamps in `CLOCK_REALTIME`
  nanoseconds, the same epoch as ROS system time.
- **`SO_TIMESTAMPING`** (always set, with at least `SOF_TIMESTAMPING_RX_HARDWARE |
  SOF_TIMESTAMPING_RAW_HARDWARE`): additionally captures the network card's *hardware-clock* receive
  timestamp. Comparing the hardware receive timestamp against the software one separates a reply that
  was genuinely late on the wire from one the kernel merely delivered late. When `enable_tx_timestamping`
  is set, the software transmit flags (`SOF_TIMESTAMPING_TX_SOFTWARE | SOF_TIMESTAMPING_SOFTWARE`) are
  added too; those transmit-completion timestamps are reported on the socket error queue and drained
  each cycle. When transmit timestamping is off, no error-queue entries are generated and that drain is
  skipped.
- **`CAN_RAW_ERR_FILTER`** (only when `enable_can_error_frames` is set): asks the kernel to deliver CAN
  bus-error frames (bus-off, acknowledgement errors, controller and transceiver problems, arbitration
  loss, and so on) on the normal receive queue, using the full error mask `CAN_ERR_MASK`. These frames
  arrive interleaved with replies and are recognized and handled in `send_and_receive` (section 4.3).

### 4.3 The core cycle: `send_and_receive(cmds, states)`

This is the per-control-cycle batched input/output. It writes all command frames first, then reads all
replies. The "all writes, then all reads" ordering matters because a CubeMars reply uses the same
identifier as its command, so interleaving would risk collisions.

1. Stamp the transmit-fill start time.
2. **Write loop**, per joint: apply `invert`, clamp each quantity to its configured range, bit-pack
   into the frame (V2 or V3 layout, below), reset the per-joint timestamp fields, and `write()`. On a
   write failure record `CAN_WRITE_FAILED`; on success record `enqueue_timestamp_ns` (the user-space
   moment the frame finished being written into the transmit buffer) and mark the state provisionally
   `CAN_NO_RESPONSE`.
3. Stamp the transmit-fill end time and compute `tx_fill_duration_ns_`.
4. **Receive loop**, per joint that was written: `recv_frame_with_timestamp()` pulls a frame together
   with both its software and hardware receive timestamps. When error frames are enabled, any frame
   carrying `CAN_ERR_FLAG` is counted (`last_error_count_`, `last_error_canid_`) and skipped so the
   loop reads the next real reply, bounded by a 16-frame storm guard that otherwise gives up and treats
   the reply as missing. A receive failure (including the `SO_RCVTIMEO` timeout) records
   `CAN_READ_FAILED`. Otherwise the reply is matched to a joint **by the identifier in the received
   frame, not by loop position**, so out-of-order replies are demultiplexed correctly. The matched
   state gets its `rx_timestamp_ns`, `rx_hw_timestamp_ns`, `dequeue_timestamp_ns`, decoded fields,
   `invert` undone, and status `SUCCESS`.
5. Compute `rx_duration_ns_`.
6. If transmit timestamping is enabled, `collect_tx_timestamps()` drains the socket error queue
   (`MSG_ERRQUEUE`) and assigns each software transmit-completion timestamp to its joint by matching
   the echoed frame identifier, recording the number of entries drained (`last_tx_errq_count_`).

The `ComStatus` per joint is the node-visible result: the node decides how to react to
`CAN_WRITE_FAILED`, `CAN_READ_FAILED`, and so on. Hard failures (an exception escaping
`send_and_receive`) are reserved for "something is really wrong".

### 4.4 Protocol framing: V2 versus V3

- **V2 (MIT mode, classic frame):** `pos` (16-bit), `vel` (12-bit), `kp` (12-bit), `kd` (12-bit),
  `torque` (12-bit) are clamped to the configured ranges and bit-packed into 8 bytes via
  `float_to_uint`. Replies are decoded with `uint_to_float` against the same ranges. Temperature is
  reported with a -40 degree Celsius offset. Identifier carried in the first data byte of the reply.
- **V3 (extended frame):** the command uses an extended (29-bit) identifier
  `can_id | (SET_MIT << 8)` with a different byte order (gains first). Replies arrive as the response
  packet `0x29` and are decoded with **physical-unit conversions** rather than range maps: position in
  0.1 degree units to radians; electrical revolutions per minute (ERPM) to radians per second using
  `numer_of_pole_pairs`, `gear_ratio`, and a factor of 10; current in 0.01 ampere units to torque via
  `torque_constant`. Temperature has no offset.

### 4.5 Motor control mode and zeroing

`start_motor_control_mode` enables a motor and optionally zeros its position at the current pose:

- **V2:** sends the magic byte sequences (`START_MOTOR_CONTROL_MODE`, and `SET_ZERO_POSITION` when
  requested).
- **V3:** has no explicit enable; it primes the connection by retrying a zero-current command up to
  `max_init_connect_trials` times (V3 motors can stick in their response state after an emergency
  stop), and uses `SET_ORIGIN_HERE` for zeroing. During zeroing the receive timeout is temporarily
  raised to 10 seconds because the operation takes seconds.

`end_motor_control_mode` exits via the V2 magic byte sequence or, for V3, a zero-current command.

### 4.6 Timestamping and error-frame helpers

- `recv_frame_with_timestamp()` uses `recvmsg` and walks the control messages to extract two receive
  timestamps from the one reply: the `SO_TIMESTAMPNS` kernel software timestamp (in `CLOCK_REALTIME`
  nanoseconds, the same epoch as ROS system time, so cross-timestamp arithmetic with ROS message
  stamps is valid) and the `SCM_TIMESTAMPING` hardware timestamp (`ts[2]`, the card's raw free-running
  clock).
- `collect_tx_timestamps()` drains the error queue for `SCM_TIMESTAMPING` software transmit-completion
  timestamps, matches each to its joint, and counts how many entries it drained.
- CAN bus-error frames are handled inline in `send_and_receive` (counted and skipped), not here; the
  counts are exposed through `get_last_error_count()`, `get_last_error_canid()`, and
  `get_last_tx_errq_count()` for the node to log.

---

## 5. The functional layer: `CubeMarsHardwareNode`

File: `src/cubemars_hardware_node.cpp` (1672 lines), header
`include/cubemars_hardware_interface/cubemars_hardware_node.hpp`.

This is a managed (lifecycle) node, `rclcpp_lifecycle::LifecycleNode`, named `cubemars_hardware_node`.
It is the primary, actively developed driver. The README describes its operational contract; this
section describes its internals.

### 5.1 Lifecycle state machine

The node follows the standard managed-node lifecycle. The README summarizes the operator view; here is
what each transition does internally:

- **`on_configure`** (the heavy transition): declares all global and per-joint parameters and
  validates them (rejecting duplicate `msg_idx`, duplicate `can_id` on one interface, non-positive
  transmission ratios, and unknown filter types); sizes every per-interface vector (interfaces are
  indexed by the lexicographic order of the `std::set` of interface names); builds each joint's
  `joint_config_t`, initial command/state, `JointParameters`, and filters; constructs one
  `CubemarsCan` per interface; creates the publishers, the `~/joint_commands` subscriber, the publish
  timer, the two origin services, and the supervisor timer; enables every motor
  (`start_motor_control_mode`), accumulating all failures before reporting so it can name every bad
  motor; registers the runtime parameter callback; and finally starts one communication thread per
  interface. After configure the motors are enabled but only zeros are sent, so they do not move.
- **`on_activate`**: refuses to activate unless at least one valid command has been received
  (`msg_received_`), then creates the watchdog timer. The communication threads, already running, begin
  applying real commands once the state is active.
- **`on_deactivate`**: stores a damping command (`kp = 0`, `kd = default_damping_KD`) into the atomic
  command pointer so the still-running threads hold the joints in damping; resets the watchdog; clears
  `msg_received_`.
- **`on_cleanup`**: tears down in a deliberate order. It joins the communication threads **before**
  taking the bus and message mutexes (a thread mid-cycle holds the bus mutex shared, so locking it
  exclusively first would deadlock), then disables every motor, destroys the `CubemarsCan` objects,
  clears all state, and resets publishers and the subscriber. It always returns success so the node
  ends up unconfigured.
- **`on_shutdown`**: from active it runs deactivate (damping) then cleanup; from inactive it runs
  cleanup; called from `main()` after the executor stops.
- **`on_error`**: manually unwinds whatever `on_configure` created if configure failed; otherwise treats
  the error as unrecoverable and forces finalization.

### 5.2 Threading model

The defining design choice: the send/receive cycle is driven by **one dedicated `std::thread` per CAN
interface**, not by a ROS timer dispatched on the executor. This removes per-cycle executor dispatch
latency and lets each bus free-run at its natural rate.

- `comm_loop(idx)` is a tight loop calling `can_cycle_callback(idx)` back-to-back while the thread's
  `running` flag is set.
- `start_comm_thread` spawns the thread and raises it to real-time scheduling
  (`SCHED_FIFO`, first-in-first-out, priority 80) on a best-effort basis; `main()` does the same for
  itself. No central processing unit (CPU) affinity is pinned.
- `stop_all_comm_threads` clears all the running flags first, then joins, so the threads wind down in
  parallel.
- Because a communication thread cannot run `cleanup()` itself (cleanup joins the communication
  threads, and a thread joining itself deadlocks), a thread that hits a fatal error while inactive
  sets `cleanup_requested_`, and a **supervisor timer** running on the executor performs the actual
  cleanup transition.

The ROS timers (publish, watchdog, supervisor) run on a `MultiThreadedExecutor`; they handle
publishing, command-staleness detection, and deferred cleanup respectively. They do not drive bus
traffic.

### 5.3 Command data flow (controller to motor)

```
~/joint_commands (robot_control_msgs/JointCommand)
        |
joint_cmd_msg_callback  -- validates array lengths; records receipt time (cmd_rx_ns_);
        |                  stores the shared_ptr directly (no copy) only when ACTIVE
        v
joint_cmd_ptr_  : std::atomic<std::shared_ptr<const JointCommand>>   (lock-free handoff)
        |
        |  (read by each comm thread at the top of its cycle)
        v
can_cycle_callback(idx):
   - map each joint's slot by msg_idx; subtract zero_position from commanded position
   - (ACTIVE only) apply Stribeck friction compensation using the measured velocity
   - (ACTIVE only) apply transmission ratio (joint space -> motor space)
        v
CubemarsCan::send_and_receive(cmds, states)   under a shared lock on the bus mutex
```

The command handoff is deliberately **lock-free**: the latest command is an immutable message held
behind a single `std::atomic<std::shared_ptr<const ...>>`. The subscriber stores a fresh pointer; the
communication threads load it. There is no mutex, no blocking, and no priority inversion between the
real-time threads and the subscriber. The `msg_idx` field maps each joint's slot in the flat command
arrays to its per-interface local index.

### 5.4 State data flow (motor to controller)

```
CubemarsCan::send_and_receive fills states[]  (pos, vel, torque, temp, status, timestamps)
        |
can_cycle_callback(idx):
   - snapshot unfiltered pos/vel (motor space) for the raw diagnostic topics
   - position median pre-filter (outlier rejection), only on SUCCESS
   - velocity filter: NONE | MOVING_AVERAGE | ALPHA_BETA (the alpha-beta filter is a coupled
     position+velocity estimator timed by per-joint receive timestamps)
   - apply transmission ratio (motor space -> joint space)
   - write per-joint values into the aggregated joint_state_msg_ (indexed by msg_idx),
     refresh per-joint rx timestamp, compute latency metrics
        v
joint_state_publish_callback (publish timer, runs on the executor at `frequency`):
   - copy the continuously-written _msg_ buffers into _msg_to_pub_ buffers under the lock
     (double buffering: minimize lock hold time, decouple write rate from publish rate)
   - stamp the message with the OLDEST successful receive time across joints (a conservative
     freshness bound) and publish all topics
```

The filters live in `include/cubemars_hardware_interface/filters.hpp`: a `MovingAverage`, an
`AlphaBetaFilter` (constant-velocity model, corrects both position and velocity from the position
measurement), and a `MedianFilter` (streaming median for outlier rejection, no allocation after
construction).

### 5.5 Watchdog

The watchdog timer (created on activate, period from `watchdog_frequency`) deactivates the node into
damping if no valid command arrived since the previous watchdog cycle. The subscriber sets
`msg_received_ = true` on every valid message; the watchdog clears it each tick. This guarantees the
motors fall into damping if the command stream stops.

### 5.6 Dynamic reconfiguration

`on_set_parameters_callback` allows a defined subset of parameters to be changed at runtime: the
global `default_damping_KD`, `damping_on_motor_error`, `max_can_errors_before_motor_shutdown`, and the
per-joint filter selection and sizes, alpha-beta gains, the six friction parameters, and the position
limits. It is a three-pass design (parse and per-parameter validate, then validate the cross-field
constraints against the effective post-batch values, then apply) and applies updates under a
**writer-preferring shared mutex**.

That custom mutex (`WritePreferringSharedMutex`, wrapping a `pthread_rwlock_t` configured for writer
preference) exists because the standard library shared mutex on this platform is reader-preferring;
against the constant high-frequency readers in `can_cycle_callback`, a reader-preferring lock would let
the parameter writer starve indefinitely.

### 5.7 Services

- `set_all_motors_origin_here` (`std_srvs/Trigger`): zeros every motor at its current pose. It stops
  all communication threads, takes the bus exclusively, disables then re-enables each motor with
  zeroing requested, then restarts the threads.
- `set_motor_origin_here` (`robot_control_msgs/srv/SetMotorOriginHere`): the same for a single motor
  identified by `motor_id` (matched against `msg_idx`), touching only that motor's interface.

Both require the inactive (configured) state.

### 5.8 Diagnostics, statistics, and latency instrumentation

This is one of the largest feature areas added on `main`. Every metric uses a best-effort,
depth-1 Quality of Service (QoS) profile. The publishers and their computations:

| Topic | Type | Meaning |
| --- | --- | --- |
| `~/joint_states` | `robot_control_msgs/JointState` | aggregated position/velocity/effort, stamped with the oldest successful receive time |
| `joint_temperatures` | `Float32MultiArray` | per-joint temperature |
| `can_cycle_frequencies` | `Float32MultiArray` | per-interface achieved cycle rate (`1/dt`) |
| `can_tx_fill_durations_us` | `Float32MultiArray` | per-interface time to fill the transmit buffer |
| `can_rx_durations_us` | `Float32MultiArray` | per-interface receive-phase duration |
| `can_processing_durations_us` | `Float32MultiArray` | per-interface cycle time minus the send and receive phases (the prologue, filters, write-back, and diagnostics "plumbing") |
| `can_intercycle_gaps_us` | `Float32MultiArray` | per-interface gap from the end of one cycle to the start of the next (loop and scheduling overhead; spikes mean the thread was descheduled) |
| `joint_reply_after_tx_us` | `Float32MultiArray` | per-joint time from transmit-fill end to reply dequeue |
| `joint_state_ages_us` | `Float32MultiArray` | per-joint freshness of the published state |
| `joint_cmd_to_bus_latencies_us` | `Float32MultiArray` | per-joint time from command receipt to frame on the wire (requires transmit timestamping) |
| `joint_motor_reply_latencies_us` | `Float32MultiArray` | per-joint motor turnaround, reply time minus transmit time (requires transmit timestamping) |
| `joint_rx_delivery_offsets_us` | `Float32MultiArray` | per-joint kernel receive-delivery delay: software minus hardware receive timestamp, with the per-joint constant clock offset subtracted (near zero normally; spikes mean kernel/interrupt delivery delay, not wire delay) |
| `joint_rx_hw_timestamps_us` | `Float32MultiArray` | per-joint card hardware-clock receive timestamp, baseline-subtracted (diff consecutive values for the card's own reply inter-arrival timing) |
| `joint_enqueue_to_wire_us` | `Float32MultiArray` | per-joint transmit-path time: software transmit timestamp minus the post-`write()` enqueue timestamp (requires transmit timestamping) |
| `joint_velocities_unfiltered` | `Float32MultiArray` | pre-filter velocity, in joint space |
| `joint_positions_unfiltered` | `Float32MultiArray` | pre-filter position, in joint space |
| `controller_latency_us` | `Float32` | round-trip from the incoming command's stamp to now |
| `~/ros2_joint_state` | `sensor_msgs/JointState` | optional standard-message mirror |

The three transmit-dependent topics (`joint_cmd_to_bus_latencies_us`, `joint_motor_reply_latencies_us`,
`joint_enqueue_to_wire_us`) are created only when `enable_tx_timestamping` is true. All latency
arithmetic relies on the timing fields that `CubemarsCan::send_and_receive` writes into each
`joint_state_t`, which is why those fields are part of the shared contract. The receive-side decomposition
deliberately splits wire time, kernel delivery time, and transmit-path time: `joint_rx_delivery_offsets_us`
isolates kernel/interrupt delivery jitter (software minus hardware receive timestamp), while
`joint_rx_hw_timestamps_us` exposes the card's own inter-arrival timing. Two throttled health warnings
complement the topics: one when CAN error frames are seen (section 5.9) and one when transmit timestamping
is on but a joint that replied received no software transmit timestamp.

### 5.9 Error handling and safety

- **Device faults** (a non-`NO_FAULT` `device_status`): when active and `damping_on_motor_error` is
  set, the node deactivates into damping.
- **Communication errors** (`ComStatus` other than `SUCCESS`): counted per interface; the counter is
  cumulative across cycles and reset only by a fully clean cycle. When the count reaches
  `max_can_errors_before_motor_shutdown`, an active node deactivates into damping and an inactive node
  requests cleanup (motors off) via the supervisor.
- **Position-limit violations** (active only): deactivate into damping.
- **CAN bus-error frames** (only when `enable_can_error_frames` is set): the communication layer counts
  and skips them inside the receive loop; the node logs a throttled warning naming the interface, the
  number of error frames this cycle, and the decoded error classes (`canErrorFrameToString` over the
  error identifier: bus-off, acknowledgement, controller, transceiver, arbitration loss, and so on).
  This is observability only; it does not by itself trigger damping. A resulting missing reply is
  handled by the communication-error path above.
- **Timestamp health warning**: when transmit timestamping is enabled but a joint that received a reply
  has no software transmit timestamp, the node logs a throttled warning and reports how many error-queue
  entries it drained that cycle, flagging a timestamping or frame-matching problem.
- The two escalation outcomes are always either damping (active) or full motor shutdown via the
  supervisor (inactive). A communication thread never performs cleanup directly.

### 5.10 Concurrency and synchronization model

This section ties together how the parallel per-interface threads combine the data from each motor and
each interface into the single published joint state. The short version of the mental model is correct:
the node runs one free-running thread per CAN interface, and those threads merge their results into one
shared, `msg_idx`-indexed set of buffers that a single publish timer snapshots and emits. The detail is
in *how* that merge stays consistent without the threads blocking each other.

```
  comm thread A (can0)          comm thread B (can1)        ...   (one per CAN interface)
  free-running loop             free-running loop
  send_and_receive():           send_and_receive():
    write all cmds                 write all cmds
    read all replies,             read all replies,
    demux by can_id               demux by can_id
        |                             |
   per-interface states[]        per-interface states[]
   (filter, transmission)        (filter, transmission)
        |                             |
        +-------------+---------------+
                      |  each thread writes ONLY its own motors' slots,
                      |  indexed by globally-unique msg_idx
                      v
        SHARED AGGREGATED BUFFERS  (joint_state_msg_, joint_temp_msg_, the diagnostic arrays)
        written under joint_state_msg_mutex_ held SHARED (writers touch disjoint indices)
                      |
                      |  exclusive lock, once per publish period
                      v
        joint_state_publish_callback (single publish timer on the executor, at `frequency`)
        - snapshot all buffers into the _to_pub_ copies (double buffering)
        - stamp with the OLDEST successful per-motor receive time
        - publish ONE ~/joint_states spanning every motor on every interface
```

**Three levels of combination.**

1. *Within one bus cycle (one interface).* A comm thread's `send_and_receive` writes all its motors'
   command frames, then reads all replies and demultiplexes them to motors **by CAN identifier** (not
   by arrival order). Each reply lands in that interface's local `states` vector. The thread then maps
   each local motor to its global slot through the motor's `msg_idx` (see sections 5.3 and 5.4).
2. *Across interfaces (the shared buffers).* Every comm thread writes its motors' results into the
   **same** aggregated buffers (`joint_state_msg_`, `joint_temp_msg_`, and the per-joint diagnostic
   arrays), each entry addressed by the motor's `msg_idx`. Because `msg_idx` is validated to be unique
   across the entire robot at configure time, and each thread only ever writes the slots belonging to
   its own motors, **no two threads ever write the same slot**. That disjointness is the reason the
   threads can hold `joint_state_msg_mutex_` in *shared* mode while writing: they are not racing each
   other for the same memory, they are only excluding the one reader that needs the whole array at once.
3. *At publish time (the single combiner).* The publish timer is one timer on the executor, firing at
   the `frequency` parameter. It takes `joint_state_msg_mutex_` **exclusively**, copies every aggregated
   buffer into its `_to_pub_` twin (double buffering), releases the lock, and publishes one
   `~/joint_states` message that spans every motor on every interface, plus the diagnostic topics.

**What "synchronized" means here, and what it does not.** The published message is a *consistent
snapshot*: the exclusive lock during the copy guarantees no torn reads, so every motor value in one
message came from a coherent moment of the buffers. It is also *conservatively stamped* with the oldest
successful per-motor receive time, so the stamp is a valid lower bound on the freshness of every joint
in that message. It is **not** phase synchronization: the per-interface threads are not started in
lockstep and each free-runs at its own bus rate, so different motors in the same published message can
have slightly different ages. That spread is exactly what `joint_state_ages_us` and the conservative
stamp expose, rather than hide.

**The synchronization primitives, in one place.**

- `joint_cmd_ptr_` (a `std::atomic<std::shared_ptr<const JointCommand>>`): the command path is
  **lock-free**. The subscriber stores a fresh immutable command; every comm thread loads it at the top
  of its cycle. No mutex sits between the controller and the real-time threads (section 5.3).
- `joint_params_mutex_` (writer-preferring shared mutex): guards the per-joint parameters and filters.
  A comm thread holds it **shared for its whole cycle**; the parameter callback takes it **exclusively**
  only to apply an update (section 5.6).
- `can_communication_mutex_` (shared mutex): guards the bus. Held **shared** during normal
  `send_and_receive`; taken **exclusively** only for origin-service calibration, which first stops the
  comm threads.
- `joint_state_msg_mutex_` (shared mutex): guards the aggregated message buffers. Held **shared** by the
  comm threads (each writing disjoint `msg_idx` slots) and **exclusively** by the publish timer.

**Lock discipline and teardown.** A comm thread takes `joint_params_mutex_` shared for the entire cycle
and the other two shared mutexes only briefly. On teardown the node joins the comm threads **before**
taking `can_communication_mutex_` exclusively, because a thread mid-cycle already holds it shared and a
thread cannot be asked to release a lock while it is being joined; this is the same self-join
consideration that pushes cleanup onto the supervisor (section 5.2).

---

## 6. Build and packaging

The driver this repository uses is built by `CMakeLists.txt` as the executable
**`cubemars_hardware_node`**, compiled from `src/cubemars_can.cpp` and `src/cubemars_hardware_node.cpp`
at **C++23** (it needs `std::atomic<std::shared_ptr>`, `std::format`, and other recent features).
Warnings are treated as errors.

`package.xml` declares the package as an `ament_cmake` package depending on `rclcpp`,
`rclcpp_lifecycle`, `robot_control_msgs`, and related packages. A Python sine tester
(`scripts/motor_sine_tester.py`) is installed alongside.

Supporting files: `.devcontainer/` (container build), `.gitlab-ci.yml` (continuous integration),
`config/test_params.yaml` and `config/test_params_2.yaml` (example node configurations),
`include/.../custom_qos.hpp` (the reliable and best-effort depth-1 QoS profiles), and `test/`.

> Note: `CMakeLists.txt` also builds a second target, a hardware-interface plugin library compiled from
> `src/cubemars_hardware_interface.cpp` (with `cubemars_hardware_interface.xml` and
> `ros2_control/cubemars.ros2_control.xacro`). It is a leftover from the upstream fork, is not used by
> this repository, and is intentionally out of scope for this document. It is retained only so the
> build is unchanged.

---

## 7. Configuration surface (node)

The node is configured by a parameter file (see `config/test_params.yaml`). The structure is:

- **Global parameters**: `joints` (the ordered list of joint names), `default_damping_KD`, `frequency`
  (publish rate), `watchdog_frequency`, `can_socket_timeout_sec`/`_usec`,
  `max_can_errors_before_motor_shutdown`, `can_initial_connection_trials`, `enable_loopback`,
  `enable_tx_timestamping`, `enable_can_error_frames`, `publish_ros2_joint_state`,
  `damping_on_motor_error`, `friction_compensation_sign_steepness`.
- **Per-joint parameters** under `joint_defintions.<joint_name>` (note the spelling used in the code):
  `msg_idx` (slot in the flat command/state arrays), `can_interface`, `can_id`, `motor_type` (keyed
  into the preset map), `invert`, the six friction parameters (`tau_c`, `tau_s`, `v_s`, `k`, `k_a`,
  `b`), `transmission_ratio`, `set_zero_position_on_configure`, `zero_position`, `pos_limit_min`/`_max`,
  `vel_filter_size`, `vel_filter_type`, `alpha`, `beta`, and `pos_median_filter_size`.

One node instance can drive motors spread across several CAN interfaces; the example config uses `can0`
and `can1`.

---

## 8. ROS interface summary (node)

- **Subscribes**: `~/joint_commands` (`robot_control_msgs/JointCommand`) with reliable depth-1 QoS.
- **Publishes**: `~/joint_states` plus the diagnostic and latency topics listed in section 5.8.
- **Services**: `set_all_motors_origin_here` (`std_srvs/Trigger`), `set_motor_origin_here`
  (`robot_control_msgs/srv/SetMotorOriginHere`).
- **Lifecycle**: managed via the standard `configure`/`activate`/`deactivate`/`cleanup` transitions
  (see the README for the operator commands).

---

## 9. Key design decisions and invariants

- **Layer separation by class boundary.** The node depends only on the `CubemarsCan` public interface
  and the `cubemars_com.hpp` types. Swapping the bus protocol means providing a different
  implementation behind that same boundary.
- **Lock-free command handoff.** A single `std::atomic<std::shared_ptr<const JointCommand>>` carries the
  latest command from the subscriber to the real-time threads, avoiding mutexes on the hot path.
- **Dedicated real-time communication threads** per interface, free-running, at `SCHED_FIFO` priority
  80, instead of executor-dispatched timers.
- **Writer-preferring shared mutex** to keep the parameter writer from starving against the
  high-frequency readers.
- **Double-buffered publishing** to keep the message lock held briefly and decouple the bus rate from
  the publish rate.
- **Conservative state stamping** with the oldest successful receive time, so every value in a
  `~/joint_states` message is guaranteed no older than the stamp.
- **Shared timestamp epoch.** Kernel timestamps are `CLOCK_REALTIME`, the same epoch as ROS system
  time, which is what makes the cross-layer latency math valid.
- **Two-clock receive timestamping.** Each reply carries both a kernel software timestamp and the
  card's hardware timestamp, so the diagnostics can attribute latency to the wire versus kernel
  delivery rather than reporting one opaque number.
- **Fail-safe escalation.** Every error path ends in either damping (active) or motor shutdown
  (inactive); communication threads defer cleanup to the executor to avoid self-join deadlock.
