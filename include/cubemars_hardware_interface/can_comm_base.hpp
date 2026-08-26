#ifndef CUBEMARS_CAN_COMM_BASE_HPP_
#define CUBEMARS_CAN_COMM_BASE_HPP_

#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include <linux/can.h>

#include "cubemars_com.hpp"

namespace cubemars
{
    // Shared CAN communication exception types. Thrown by any backend and caught by the node;
    // can_interface_error in particular is handled specially in on_configure.
    class can_interface_error : public std::runtime_error
    {
    public:
        explicit can_interface_error(const std::string &__arg) : std::runtime_error(__arg) {};
        explicit can_interface_error(const char *__arg) : std::runtime_error(__arg) {};
    };

    class can_device_error : public std::runtime_error
    {
    public:
        explicit can_device_error(const std::string &__arg) : std::runtime_error(__arg) {};
        explicit can_device_error(const char *__arg) : std::runtime_error(__arg) {};
    };

    class motor_error : public std::runtime_error
    {
    public:
        explicit motor_error(const std::string &__arg) : std::runtime_error(__arg) {};
        explicit motor_error(const char *__arg) : std::runtime_error(__arg) {};
    };

    // Abstract communication backend: the surface the lifecycle node uses to talk to the motors on one
    // CAN interface. Concrete backends implement it, so the node can drive different electronics through
    // one pointer type, selected by configuration:
    //   - CubemarsCan  : classic CAN, CubeMars electronics (V2/V3).
    //   - MabFdCan     : CAN FD register protocol, MAB electronics.
    class CanCommBase
    {
    public:
        virtual ~CanCommBase() = default;

        // Enable / disable one motor (by its local joint index on this interface), optionally zeroing
        // its position at the current pose; and the all-motors-on-this-interface convenience overloads.
        virtual void start_motor_control_mode(unsigned int joint_id, bool set_zero_position_on_enable) = 0;
        virtual void end_motor_control_mode(unsigned int joint_id) = 0;
        virtual void start_motor_control_mode(bool set_zero_position_on_enable) = 0;
        virtual void end_motor_control_mode() = 0;

        // Zero a motor's position at its current pose, without touching its control mode (the motor
        // stays enabled throughout). Used to calibrate an already-active motor.
        virtual void set_zero_position(unsigned int joint_id) = 0;

        // Fire a single zero-setpoint motion command at an already-enabled joint, without waiting
        // for or reading any reply. Meant to be called from a separate keepalive thread DURING
        // on_configure(), while sibling joints on the same interface are still being configured
        // sequentially (which can take well over a drive's motion-command watchdog window) -- so
        // an already-enabled motor isn't disabled again before the cyclic comm thread starts.
        // Backends without such a watchdog (e.g. CubemarsCan) can leave this a no-op.
        virtual void send_zero_motion_keepalive([[maybe_unused]] unsigned int joint_id) {}

        // Discard any frames currently queued in the socket's RX buffer without reading them,
        // e.g. to drop stray keepalive replies left over from configuration before the cyclic
        // send_and_receive() loop starts reading. Backends without such stray traffic (e.g.
        // CubemarsCan) can leave this a no-op.
        virtual void flush_rx_queue() {}

        // One control cycle: write every command, then receive and demultiplex every reply into states.
        // is_active distinguishes the lifecycle's ACTIVE state from INACTIVE; backends that don't need
        // the distinction (e.g. CubemarsCan) simply ignore it.
        virtual void send_and_receive(const std::vector<joint_cmd_t> &cmds, std::vector<joint_state_t> &states, bool is_active) = 0;

        // Identity / addressing.
        virtual const std::string &GetName() = 0;
        virtual canid_t get_can_id(unsigned int joint_index) = 0;

        // Per-cycle timing diagnostics (CLOCK_REALTIME ns) from the last send_and_receive().
        virtual int64_t get_tx_fill_duration_ns() const = 0;
        virtual int64_t get_rx_duration_ns() const = 0;
        virtual int64_t get_tx_fill_end_ns() const = 0;

        // Per-cycle error / queue diagnostics from the last send_and_receive().
        virtual unsigned int get_last_tx_errq_count() const = 0;
        virtual unsigned int get_last_error_count() const = 0;
        virtual canid_t get_last_error_canid() const = 0;
    };
} // namespace cubemars

#endif // CUBEMARS_CAN_COMM_BASE_HPP_
