#ifndef CUBEMARS_MAB_FD_CAN_HPP_
#define CUBEMARS_MAB_FD_CAN_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include <linux/can.h>

#include "cubemars_com.hpp"
#include "can_comm_base.hpp"

namespace cubemars
{
    // Communication backend for CubeMars motors driven by MAB electronics, which speak a register-write
    // protocol over CAN FD (Flexible Data-rate). Ported from the MAB_FDCAN branch behind the shared
    // CanCommBase interface so one node can drive CubeMars and MAB buses side by side.
    //
    // SCOPE OF THIS STEP: the constructor opens and configures the CAN FD socket (this is the part that
    // differs from classic CAN: CAN_RAW_FD_FRAMES, a canfd_frame). The register protocol itself
    // (send_and_receive, motor enable/disable, timestamping diagnostics) is ported in a following step;
    // until then those methods throw a clear "not implemented yet" error and the getters return 0. See
    // docs/MAB_FDCAN_PARITY.md.
    class MabFdCan : public CanCommBase
    {
    public:
        MabFdCan(const std::string &can_interface, const int &enable_loopback,
                 const std::vector<joint_config_t> &joint_configs,
                 const long &socket_timeout_sec, const long &socket_timeout_usec,
                 unsigned int max_init_connect_trials,
                 bool enable_tx_timestamping = true, bool enable_can_error_frames = false);
        ~MabFdCan() override;

        void start_motor_control_mode(unsigned int joint_id, bool set_zero_position_on_enable) override;
        void end_motor_control_mode(unsigned int joint_id) override;
        void start_motor_control_mode(bool set_zero_position_on_enable) override;
        void end_motor_control_mode() override;

        void send_and_receive(const std::vector<joint_cmd_t> &cmds, std::vector<joint_state_t> &states) override;

        const std::string &GetName() override { return can_interface_; }
        canid_t get_can_id(unsigned int joint_index) override { return joint_configs_.at(joint_index).can_id; }

        // Diagnostics: not produced yet (the protocol/timestamping port is a following step).
        int64_t get_tx_fill_duration_ns() const override { return 0; }
        int64_t get_rx_duration_ns() const override { return 0; }
        int64_t get_tx_fill_end_ns() const override { return 0; }
        unsigned int get_last_tx_errq_count() const override { return 0; }
        unsigned int get_last_error_count() const override { return 0; }
        canid_t get_last_error_canid() const override { return 0; }

    private:
        std::string can_interface_;
        int enable_loopback_;
        std::vector<joint_config_t> joint_configs_;
        long socket_timeout_sec_;
        long socket_timeout_usec_;
        unsigned int max_initial_connection_trials_; // used by the (pending) connection-priming logic
        bool enable_tx_timestamping_;                // used by the (pending) timestamping logic
        bool enable_can_error_frames_;               // used by the (pending) error-frame handling
        int can_socket_fd_;
        canfd_frame send_frame_;
        canfd_frame recv_frame_;
    };
} // namespace cubemars

#endif // CUBEMARS_MAB_FD_CAN_HPP_
