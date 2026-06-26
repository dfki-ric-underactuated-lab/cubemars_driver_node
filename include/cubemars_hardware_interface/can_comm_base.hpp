#ifndef CUBEMARS_CAN_COMM_BASE_HPP_
#define CUBEMARS_CAN_COMM_BASE_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include <linux/can.h>

#include "cubemars_com.hpp"

namespace cubemars
{
    // Abstract communication backend: the surface the lifecycle node uses to talk to the motors on one
    // CAN interface. Concrete backends implement it, so the node can drive different electronics through
    // one pointer type, selected by configuration:
    //   - CubemarsCan  : classic CAN, CubeMars electronics (V2/V3). The current implementation.
    //   - MabFdCan     : CAN FD register protocol, MAB electronics. To be ported in from MAB_FDCAN.
    //
    // This is exactly the set of methods the node calls on a per-interface communication object; it is
    // intentionally the same signatures CubemarsCan already exposed, so introducing the base class is a
    // behaviour-preserving change. The constructor is deliberately NOT part of the interface: each
    // backend is built concretely (later, by a small factory chosen from a config parameter), and only
    // then used polymorphically through this base. See docs/MAB_FDCAN_PARITY.md for the plan.
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

        // One control cycle: write every command, then receive and demultiplex every reply into states.
        virtual void send_and_receive(const std::vector<joint_cmd_t> &cmds, std::vector<joint_state_t> &states) = 0;

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
