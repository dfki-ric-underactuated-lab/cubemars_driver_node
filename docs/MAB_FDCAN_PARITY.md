# `main` versus `MAB_FDCAN`: Differences and Integration Plan

This document compares the `main` branch with the `MAB_FDCAN` branch and lists the work needed to
**absorb the `MAB_FDCAN` communication layer into `main`**. The end state is a single Robot Operating
System 2 (ROS 2) node on `main` that supports both CubeMars electronics and MAB electronics,
**selectable via a configuration parameter**, while keeping the two communication layers as separate
implementations. `main` remains the home of the code and gains a second backend; `MAB_FDCAN` does not
replace `main`.

Read [ARCHITECTURE.md](ARCHITECTURE.md) first; this document assumes the layer model described there
(a functional layer that depends only on the `cubemars::CubemarsCan` class and the shared types in
`cubemars_com.hpp`).

References: `main` at `837e408` (2026-06-24); `MAB_FDCAN` at `eb397b6` (2026-04-16); common ancestor
`15e21e3` (2026-02-10).

---

## 1. Executive summary (what actually diverged)

The intuitive framing ("MAB stripped the node down") is the opposite of what the history shows. The
precise situation is:

- **`MAB_FDCAN` changed only the communication layer.** It rewrote `cubemars_can.cpp`,
  `cubemars_can.hpp`, and `cubemars_com.hpp` to speak the MAB protocol over CAN with Flexible Data-rate
  (CAN FD), kept the original CubeMars layer as `* copy.*` backup files, and added some standalone
  bring-up programs (`mab_minimal.*`, `minimal_node.cpp`) plus a controller configuration dump
  (`AK10-9_read.cfg`).
- **`MAB_FDCAN` did not touch the functional layer.** Its `src/cubemars_hardware_node.cpp` is
  byte-for-byte identical to the common ancestor; the node is frozen at the fork point (verified:
  `git diff` of the node between the ancestor and the `MAB_FDCAN` tip is empty).
- **`main` advanced both layers.** The node grew from 802 to 1672 lines (dedicated communication
  threads, lock-free command handoff, alpha-beta and median filters, dynamic reconfiguration, an
  extensive latency-instrumentation suite, a per-motor origin service), and the communication layer
  gained V3 support and full transmit/receive timestamping.

So the gap is almost entirely **"the MAB branch is missing everything `main` added to the functional
layer and to the contract since 2026-02-10."** The MAB protocol work itself is real and largely done;
it just needs to be reconciled with `main`'s newer contract and registered as a second backend behind
`main`'s node.

Why this is tractable: the node depends only on the `CubemarsCan` public interface and the
`cubemars_com.hpp` types, and the MAB protocol already implements the same central method,
`send_and_receive(cmds, states)`. Because `main` already has the finished functional layer, the
contract, and the CubeMars backend, the integration is **additive on `main`**: (a) put the
communication layer behind a small abstract interface, (b) port the `MAB_FDCAN` protocol code into
`main` as a second backend implementing that interface, and (c) select the backend from a
configuration parameter. The node does not move; it stays on `main` and is reused unchanged in
substance.

---

## 2. Divergence map

```
                    15e21e3  (2026-02-10, common ancestor)
                   /        \
   main (9 commits)          MAB_FDCAN (5 commits)
   - node 802 -> 1672 ln     - node UNCHANGED from ancestor (802 ln, frozen)
   - comm layer: +V3,         - comm layer: rewritten for MAB FDCAN register protocol
     +tx/rx timestamping      - + mab_minimal.*, minimal_node.cpp (bring-up, not built)
   - +alpha-beta/median       - + AK10-9_read.cfg (MAB controller dump)
     filters, dynamic         - + cubemars_can/com "copy" backups of the CubeMars layer
     reconfigure, latency
     topics, per-motor
     origin service
   tip: 837e408 (2026-06-24)  tip: eb397b6 (2026-04-16)
```

Files changed on `MAB_FDCAN` relative to the ancestor: `cubemars_can.{cpp,hpp}`, `cubemars_com.hpp`
(rewritten); `filters.hpp` (older, fewer filters); the added MAB and scratch files; configuration and
container files. Files NOT changed on `MAB_FDCAN`: `cubemars_hardware_node.{cpp,hpp}` (the node).

---

## 3. Difference detail

### 3.1 Communication protocol

| Aspect | `main` (CubeMars electronics) | `MAB_FDCAN` (MAB electronics) |
| --- | --- | --- |
| Bus mode | Classic CAN (8-byte frames) | CAN FD: `setsockopt(CAN_RAW_FD_FRAMES)`, `canfd_frame`, 64-byte payload, Bit Rate Switch flag (`CANFD_BRS`); data bitrate intended at 8 Mbit/s |
| Frame buffers | `can_frame` | `canfd_frame` |
| Command encoding | MIT-style bit-packing (V2) or extended-frame command IDs (V3); fixed-point with `float_to_uint`, clamped to per-joint ranges | Register-write protocol: each frame starts with `frame_id = 0x40` and carries register-id plus value tuples |
| Command content | `pos/vel/torque/kp/kd` packed to 12-16 bits each | One `MotionCommand_Message` packs five register/value pairs as full IEEE-754 floats (`kp` 0x50, `kd` 0x51, position 0x150, velocity 0x151, torque 0x152). This is why CAN FD is required: the payload exceeds 8 bytes |
| Clamping | yes, to configured min/max | none currently (floats sent verbatim) |
| Reply decoding | V2 range-map decode; V3 physical-unit conversions (electrical revolutions per minute to radians per second, current to torque) | `Legacy_Response` struct copied straight into the state; `quick_status` becomes the device status; no per-field scaling |
| `invert` handling | applied on command and state | not applied |
| Enable / disable | V2 magic bytes / V3 zero-current priming | register writes: mode `0x140 = 0x04`, state `0x142 = 0x27` (disable `0x40`) via `send_config_frames` |
| Zeroing | V2 `SET_ZERO_POSITION` / V3 `SET_ORIGIN_HERE` | `RunZero_Message` register `0x8C` via `send_zero_frame` |
| Control loops | host runs MIT impedance; motor returns raw | MAB controller runs internal position/velocity/impedance loops (see `AK10-9_read.cfg`); host sends targets and gains |

The MAB layer keeps the old V2/V3 receive-filter building code, so it still references `SERIES_TYPE`
and the response-packet constant, but the actual command and reply handling no longer uses the V2/V3
split.

### 3.2 Shared contract differences (`cubemars_com.hpp`)

This is the part that must be reconciled before `main`'s node can compile against the MAB layer.

| Type | `main` | `MAB_FDCAN` | Impact |
| --- | --- | --- | --- |
| `joint_cmd_t` | `float pos, vel, torque, kp, kd` | identical | none |
| `joint_state_t` device status | `ErrorCode device_status` | `int16_t device_status` | must agree on one type |
| `joint_state_t` timestamps | five fields: `rx_timestamp_ns`, `send_timestamp_ns`, `dequeue_timestamp_ns`, `enqueue_timestamp_ns`, `rx_hw_timestamp_ns` | absent | **`main`'s node reads these; absence breaks compilation** |
| `errorFlagToString` | full enum mapping | gutted (only `NO_FAULT`), takes `int16_t` | decode/mapping decision |
| New MAB structs | none | `MotorMode/MotorState/MotionCommand/RunZero/Blink/BaudRate_Message` | MAB-only, keep in MAB layer |
| Motor presets | CubeMars types | adds an `"MAB"` entry cloned from `AK10-9` | extend the preset map |

### 3.3 `CubemarsCan` public-interface differences

`main`'s node calls members that the MAB class does not provide. This is the concrete compile-time gap.

| Member | `main` | `MAB_FDCAN` |
| --- | --- | --- |
| Constructor | 8 parameters, including `bool enable_tx_timestamping = true` and `bool enable_can_error_frames = false` | 6 parameters, no timestamping or error-frame flags |
| `send_and_receive(cmds, states)` | present | present (same signature) |
| `start_motor_control_mode` / `end_motor_control_mode` (single and all) | present | present |
| `GetName`, `get_can_id` | present | present |
| `get_tx_fill_duration_ns()` | present | **absent** |
| `get_rx_duration_ns()` | present | **absent** |
| `get_tx_fill_end_ns()` | present | **absent** |
| `get_last_tx_errq_count()` / `get_last_error_count()` / `get_last_error_canid()` | present | **absent** |
| Extra MAB methods | none | `send_zero_frame(...)`, `send_config_frames(...)` |

### 3.4 Functional-layer (node) feature gap

`MAB_FDCAN`'s node is the 2026-02-10 snapshot. Everything in this table exists on `main` and is absent
on `MAB_FDCAN`:

| Feature | `main` | `MAB_FDCAN` |
| --- | --- | --- |
| Command handoff | lock-free `std::atomic<std::shared_ptr<const JointCommand>>` | shared-mutex plus message copy |
| Bus cycle driver | one dedicated real-time `std::thread` per interface, free-running | executor-dispatched ROS timers with mutually-exclusive callback groups |
| Velocity filter | none / moving-average / alpha-beta (coupled estimator) | moving-average only |
| Position filter | streaming median (outlier rejection) | none |
| Dynamic reconfiguration | parameter callback plus writer-preferring shared mutex | none |
| Latency / statistics topics | about fifteen topics (transmit-fill, receive duration, processing duration, inter-cycle gap, reply-after-transmit, state age, command-to-bus, motor reply, enqueue-to-wire, RX delivery offset, RX hardware timestamp, unfiltered position/velocity, controller latency) | none beyond cycle frequency |
| Transmit/receive timestamping | full kernel plumbing: software and hardware RX timestamps, software TX timestamps, post-`write()` enqueue timestamps | none |
| CAN bus-error frames | optional `CAN_RAW_ERR_FILTER` delivery, counted and skipped, with throttled decoded logging | none |
| Origin services | `set_all_motors_origin_here` and per-motor `set_motor_origin_here` | `set_all_motors_origin_here` only |
| Deferred-cleanup supervisor | `cleanup_requested_` plus supervisor timer (avoids comm-thread self-join) | not present |
| Present on both | lifecycle state machine, multiple interfaces, watchdog into damping, Stribeck friction model, transmission ratios, position software limits, communication-error counting, real-time `SCHED_FIFO` scheduling, `~/joint_states` / temperatures / cycle-frequency / optional standard joint-state publishers | same |

`filters.hpp` on `MAB_FDCAN` is the older version (moving-average only); `main`'s adds the alpha-beta
and median filters.

### 3.5 Build, configuration, and scratch files

- `CMakeLists.txt` (`MAB_FDCAN`): builds the node executable (plus the unused fork-remnant plugin
  library, the same as `main`); the `mab_minimal`/`minimal_node` executable block is commented out. No
  MAB vendor library is linked (all CAN is raw SocketCAN).
- `mab_minimal.{cpp,hpp}`, `minimal_node.cpp`: standalone bring-up programs (the "blinking works" and
  "first working version" commits) that proved the MAB register protocol on real hardware using raw
  SocketCAN. Not part of the production build. Useful to keep as a diagnostic tool, but they belong in
  a clearly-marked tools or examples area, not mixed into `src/`.
- `cubemars_can copy.{cpp,hpp}`, `cubemars_com copy.hpp`: untouched backups of the original CubeMars
  layer. These should be deleted once the layer separation (section 4) preserves the CubeMars code
  properly under version control rather than as copy files.
- `AK10-9_read.cfg`: a MAB controller parameter dump (motor constants, limits, motion profile, output
  encoder, and position/velocity/impedance gains). It documents that the MAB controller runs internal
  loops, which is why the protocol is register/target based. Keep it as reference documentation.
- The README on both branches documents only classic-CAN bring-up. CAN FD needs the interface brought
  up with flexible data-rate enabled and a data bitrate set; the MAB README section must be updated.

---

## 4. Integration strategy: two backends on `main`, selected by configuration

This is the confirmed direction. `main` keeps its functional layer, its contract, and its CubeMars
backend, and gains the MAB backend behind a small abstraction, chosen at configure time from a
parameter.

**The design (a polymorphic communication interface):**

- Define an abstract base (for example `cubemars::CanCommBase`) on `main` declaring exactly the
  node-facing surface from section 3.3: the motor-mode methods, `send_and_receive`, `GetName`,
  `get_can_id`, and the diagnostics getters (the timing getters plus the error/queue counters).
- Make the existing `CubemarsCan` implement that interface (a mechanical change, no behavior change),
  and port the `MAB_FDCAN` protocol code into `main` as a new `MabFdCan` implementing the same
  interface. Each backend lives in its own translation unit, so the two communication layers stay
  fully separate.
- The node holds `std::vector<std::shared_ptr<CanCommBase>>` and a small factory in `on_configure`
  picks each interface's implementation from the launch YAML (the `can_backends` map, below). One
  binary then drives either electronics and can mix CubeMars and MAB buses in the same robot.
- Cost: one virtual call per `send_and_receive`, negligible at these rates. Benefit: a single node and
  binary to maintain.

**Selection granularity: per CAN interface, chosen in the launch YAML (a firm requirement).** The
backend is selected per CAN interface from the node's parameter file at launch, with no recompilation.
One launch can declare, for example, `can0` and `can2` as CubeMars and `can1` and `can3` as MAB, each
with its own motors and motor parameters. This is not merely preferred: classic CAN and CAN FD must not
be mixed on the same physical wire (a classic-only controller faults on CAN FD frames), so the backend
is a property of the interface, not of an individual motor. A single global default is allowed for the
common case where every bus uses the same backend.

The configuration keeps the existing per-joint structure (each joint names its `can_interface`,
`can_id`, `motor_type`, and parameters) and adds a per-interface backend map:

```yaml
cubemars_hardware_node:
  ros__parameters:
    # New keys: which backend to construct for each CAN interface. Interfaces not listed
    # fall back to comm_backend_default.
    comm_backend_default: cubemars
    can_backends:
      can0: cubemars
      can1: mab
      can2: cubemars
      can3: mab

    joints: ["j_a", "j_b", "j_c", "j_d"]
    joint_defintions:
      j_a: { can_interface: "can0", can_id: 1, motor_type: "AK80-9", invert: false }  # CubeMars bus
      j_b: { can_interface: "can1", can_id: 1, motor_type: "MAB",     invert: false }  # MAB bus
      j_c: { can_interface: "can2", can_id: 2, motor_type: "AK10-9",  invert: false }  # CubeMars bus
      j_d: { can_interface: "can3", can_id: 2, motor_type: "MAB",     invert: false }  # MAB bus
```

The node still discovers the set of interfaces from the per-joint `can_interface` fields; the
`can_backends` map only decides which backend to build for each. The `on_configure` factory reads the
map (falling back to `comm_backend_default`) and constructs a `CubemarsCan` or a `MabFdCan` per
interface. Because selection happens in `on_configure`, it is fixed for the node's active session;
changing a bus's electronics means editing the YAML and re-launching (or cleaning up and re-configuring
the lifecycle node), which matches reality since a controller cannot be swapped on a live bus. A
sensible validation is to reject a configuration that puts two joints with different backends, or a
`motor_type` that disagrees with the interface's backend, on the same interface.

**Shared behavioral guarantee: deterministic send order.** Within one `send_and_receive`, every backend
transmits commands in order of increasing motor `can_id` on that bus (lowest `can_id` first, then
ascending), so the on-wire ordering is deterministic and independent of the order joints happen to
appear in the configuration. The natural implementation is inside each backend's `send_and_receive`
write loop: iterate a `can_id`-sorted index list (computed once, since identifiers are fixed). The
receive loop is unaffected, because replies are matched by identifier rather than by position. See the
ToDo in section 5.

**Considered alternatives:**

- *Build-time selection:* two same-named classes selected with a CMake option, or two executables
  sharing the node translation unit through a type alias. No virtual calls, but not runtime-selectable
  and it risks the two copies drifting. Rejected in favor of the runtime parameter the goal calls for.
- *One class branching on a new `SERIES_TYPE`:* would put classic-CAN and CAN FD socket setup and
  framing in one class, exactly the coupling the layer boundary prevents. Rejected.

The contract in `cubemars_com.hpp` stays **a single shared header** so both backends agree on
`joint_cmd_t` and `joint_state_t` byte-for-byte.

---

## 5. Integration ToDo checklist

Ordered by dependency. All work happens on `main` (via a feature branch off `main`). P0 items land a
single node that builds and runs against either backend; P1 items complete the MAB instrumentation and
protocol correctness; P2 items are cleanup and validation.

### P0 - Establish the backend abstraction on `main`

- [ ] On `main`, define the abstract `CanCommBase` interface (the node-facing surface from section
      3.3) and make the existing `CubemarsCan` implement it (`: public CanCommBase`, methods marked
      `override`). No behavior change. This is a safe, standalone first step.
- [ ] Add the per-interface backend selection in `on_configure`: read a `can_backends` map (CAN
      interface name to `cubemars` | `mab`) plus a `comm_backend_default`, and a factory that builds the
      right backend per interface (section 4). Default to the CubeMars backend so existing
      configurations are unaffected; validate that all joints on one interface agree on the backend.
      (The node already holds `std::vector<std::shared_ptr<CanCommBase>>` after the abstraction step.)

### P0 - Port the MAB backend into `main`

- [ ] Bring the `MAB_FDCAN` communication code into `main` as a new `MabFdCan` class implementing
      `CanCommBase`, in its own translation unit (CAN FD socket setup, the register-write
      `send_and_receive`, the mode/state/zero register writes, `MotionCommand_Message` /
      `Legacy_Response`). Do not bring the frozen `MAB_FDCAN` node or the `* copy.*` files; `main`'s
      node and CubeMars backend already exist.
- [ ] Fold the MAB-only additive types into `main`'s `cubemars_com.hpp` (the MAB message structs and
      the MAB motor preset entry). Keep `main`'s `joint_state_t` (with `ErrorCode` and the five timing
      fields `rx_timestamp_ns` / `send_timestamp_ns` / `dequeue_timestamp_ns` / `enqueue_timestamp_ns` /
      `rx_hw_timestamp_ns`) as the single shared contract; map the MAB `quick_status` into
      `device_status` (recommend `ErrorCode`, or a dedicated MAB fault enum if the codes differ) and
      provide a matching status-to-string mapping.
- [ ] Make `MabFdCan` satisfy the full `CanCommBase` surface: accept the `enable_tx_timestamping` and
      `enable_can_error_frames` constructor arguments and implement the six diagnostics getters
      (`get_tx_fill_duration_ns` / `get_rx_duration_ns` / `get_tx_fill_end_ns` plus
      `get_last_tx_errq_count` / `get_last_error_count` / `get_last_error_canid`). Minimum viable
      version: accept the flags and return sentinel/zero from the getters so it builds and runs (the
      latency topics read as not-a-number or zero), then complete real timestamping in P1.
- [ ] Update `CMakeLists.txt` to compile the new backend into the node target; keep the node at C++23,
      warnings-as-errors.

### P1 - Shared backend behavior (both CubeMars and MAB)

- [ ] Transmit commands in order of increasing motor `can_id` within each `send_and_receive` (lowest
      first, then ascending). Implement it once per backend by iterating a precomputed `can_id`-sorted
      index list in the write loop; leave the reply loop untouched (it matches by identifier). This is a
      deliberate behavior change from the current config-order send, so confirm the on-wire order with
      `candump` or a logic analyzer if downstream timing depends on it.

### P1 - Full instrumentation parity for the `MabFdCan` backend

- [ ] Implement the full timestamping plumbing in the MAB CAN FD layer: software receive timestamps
      (`SO_TIMESTAMPNS`), hardware receive timestamps plus software transmit timestamps
      (`SO_TIMESTAMPING` with the hardware-RX and software-TX flags, plus error-queue draining), and the
      post-`write()` enqueue timestamp. These socket options work for `canfd_frame` as well, so the
      receive-delivery, enqueue-to-wire, and motor-reply latency topics can all be populated.
      Optionally wire `CAN_RAW_ERR_FILTER` so the MAB backend reports CAN bus-error frames the way the
      CubeMars backend does.
- [ ] Populate `tx_fill_duration_ns_`, `rx_duration_ns_`, `tx_fill_end_ns_`, the per-state timing
      fields, and the error counters (`last_tx_errq_count_` / `last_error_count_` / `last_error_canid_`)
      inside the MAB `send_and_receive`, mirroring `main`'s timing and error capture.

### P1 - MAB protocol gaps that affect correctness and safety

- [ ] Apply `invert` on commands and states in the MAB layer (currently ignored).
- [ ] Clamp commanded position/velocity/torque/gains to the configured per-joint ranges before
      sending (currently sent verbatim; `main` clamps).
- [ ] Wire the MAB zeroing (`RunZero_Message`) into the node's origin services, including the per-motor
      `set_motor_origin_here` path that `main` adds.
- [ ] Decide and document the semantics of `kp`/`kd`/targets for the MAB controller's internal loops
      versus MIT impedance mode, so controller authors know what the gains mean on each backend.
- [ ] Extend `joint_config_per_motor_type` with the MAB motor presets needed by the deployment.

### P2 - Cleanup, configuration, and documentation

- [ ] When porting, keep `mab_minimal.{cpp,hpp}` and `minimal_node.cpp` out of `main`'s default build;
      if they are useful as a bring-up tool, add them under a tools or examples directory as an
      explicit, clearly-optional target. The `* copy.*` backup files are not carried over to `main` at
      all (version control already preserves the CubeMars backend).
- [ ] Reconcile the example configurations (`config/test_params.yaml`, `test_params_2.yaml`,
      `test_params_mass.yaml`) and add the backend-selection parameter and MAB motor entries.
- [ ] Update `.devcontainer` and the continuous-integration configuration if the MAB build needs
      anything new.
- [ ] Update the README: CAN FD interface bring-up (flexible data-rate on, data bitrate set), the
      backend-selection parameter, and any MAB-specific operational notes.

### P2 - Validation

- [ ] Bring up a MAB motor with the unified node and verify the full lifecycle, the watchdog into
      damping, the friction model, position limits, filters, dynamic reconfiguration, and (once
      implemented) the latency topics.
- [ ] Regression-test a classic-CAN CubeMars motor with the same binary to prove interchangeability and
      that the transplanted node did not regress the existing path.
- [ ] Confirm real-time behavior on the target machine (the `SCHED_FIFO` threads and the achieved CAN
      cycle frequency) under the CAN FD data rate.

---

## 6. Risks and open questions

- **Timestamp parity over CAN FD.** Kernel timestamping should behave the same for `canfd_frame`, but
  this needs verification on the target kernel and CAN FD controller. If hardware transmit timestamps
  are unavailable, the command-to-bus latency topic will rely on software timestamps only (as on
  `main`).
- **Device-status semantics.** The MAB `quick_status` codes may not map onto the CubeMars `ErrorCode`
  enum. Confirm the MAB fault encoding before collapsing them into one type; a separate MAB enum may be
  cleaner.
- **Gain semantics differ by backend.** On CubeMars the host closes the impedance loop (MIT mode); on
  MAB the controller closes internal loops and the host sends targets and gains. The same
  `JointCommand` fields therefore mean different things physically. This must be documented so
  controllers are tuned per backend.
- **No command clamping on MAB today** is a safety gap; clamping should land before any unsupervised
  motion test.
- **Inherited snapshot bugs.** The frozen node carries minor issues from the 2026-02-10 snapshot (for
  example a velocity-filter double-index and an off-by-one in the origin service loop). Adopting
  `main`'s node supersedes these; do not port the old node forward.
- **Where these changes land (decided).** The work happens on `main`, via a feature branch off `main`.
  `main` absorbs the `MAB_FDCAN` communication layer as a second backend and supports both electronics,
  selected by configuration with the CubeMars backend as the default. `MAB_FDCAN` does not replace
  `main`; once its protocol code has been ported in and validated, the branch can be retired. A single
  shared `cubemars_com.hpp` and a single node are the end state.
