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
    // MAB register-protocol frames. MAB electronics are commanded by writing <register_id, value>
    // pairs into a CAN FD payload (each frame starts with frame_id 0x40 and a padding byte) and reply
    // with a fixed Legacy_Response. The structs are byte-packed because they are memcpy'd straight
    // to/from the CAN FD frame data. They live in the MAB backend (not the shared cubemars_com.hpp)
    // because they are MAB-implementation detail.
    #pragma pack(push, 1)
    struct Legacy_Response
    {
        uint8_t frame_id = 0;
        int16_t quick_status = 0;
        uint8_t temperature = 0;
        float position = 0.f;
        float velocity = 0.f;
        float torque = 0.f;
        float encoder_position = 0.f;
        float encoder_velocity = 0.f;
    };

    struct MotorMode_Message
    {
        uint8_t frame_id = 0x40;
        uint8_t padding = 0x00;
        int16_t register_id = 0x140;
        int8_t register_value = 0x04;
    };

    struct MotorState_Message
    {
        uint8_t frame_id = 0x40;
        uint8_t padding = 0x00;
        int16_t register_id = 0x142;
        int16_t register_value = 0x27; // 0x40 to disable
    };

    struct RunZero_Message
    {
        uint8_t frame_id = 0x40;
        uint8_t padding = 0x00;
        int16_t register_id = 0x8C;
        int8_t register_value = 1;
    };

    struct MotionCommand_Message
    {
        uint8_t frame_id = 0x40;
        uint8_t padding = 0x00;
        int16_t pk_register_id = 0x50;
        float desired_pk = 0.f;
        int16_t dk_register_id = 0x51;
        float desired_dk = 0.f;
        int16_t position_register_id = 0x150;
        float desired_position = 0.f;
        int16_t velocity_register_id = 0x151;
        float desired_velocity = 0.f;
        int16_t torque_register_id = 0x152;
        float desired_torque = 0.f;
    };
    #pragma pack(pop)

    // Communication backend for CubeMars motors driven by MAB electronics: a register-write protocol
    // over CAN FD (Flexible Data-rate). Ported from the MAB_FDCAN branch behind the shared CanCommBase
    // interface so one node can drive CubeMars and MAB buses side by side.
    //
    // Deferred to following steps (see docs/MAB_FDCAN_PARITY.md): invert handling, per-joint range
    // clamping, a documented MAB quick_status -> fault mapping, and the full RX/TX timestamping suite
    // (the diagnostics getters return 0 until then).
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

        // Diagnostics: not produced yet (the timestamping port is a following step).
        int64_t get_tx_fill_duration_ns() const override { return 0; }
        int64_t get_rx_duration_ns() const override { return 0; }
        int64_t get_tx_fill_end_ns() const override { return 0; }
        unsigned int get_last_tx_errq_count() const override { return 0; }
        unsigned int get_last_error_count() const override { return 0; }
        canid_t get_last_error_canid() const override { return 0; }

    private:
        // Enable/disable a motor by writing the mode + state registers (with a read-back acknowledgement),
        // and zero a motor by writing the RunZero register.
        void send_config_frames(const canid_t &can_id, MotorMode_Message mm, MotorState_Message ms);
        void send_zero_frame(const canid_t &can_id, RunZero_Message zm);

        std::string can_interface_;
        int enable_loopback_;
        std::vector<joint_config_t> joint_configs_;
        long socket_timeout_sec_;
        long socket_timeout_usec_;
        unsigned int max_initial_connection_trials_;
        bool enable_tx_timestamping_;  // used by the (pending) timestamping logic
        bool enable_can_error_frames_; // used by the (pending) error-frame handling
        std::vector<bool> send_ok_;
        std::vector<bool> recv_ok_;
        int can_socket_fd_;
        canfd_frame send_frame_;
        canfd_frame recv_frame_;
    };
} // namespace cubemars

#endif // CUBEMARS_MAB_FD_CAN_HPP_
