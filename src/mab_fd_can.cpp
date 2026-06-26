#include "cubemars_hardware_interface/mab_fd_can.hpp"

#include <algorithm>
#include <cmath>
#include <cerrno>
#include <cstring>
#include <ctime>
#include <format>
#include <stdexcept>

#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can/raw.h>

namespace cubemars
{

namespace
{
// MAB Quick Status (the LEGACY_RESPONSE quick_status field). Bits 0-6 are error categories; bit 15 is
// the "target reached" status (a normal condition, NOT a fault); the remaining bits are reserved. Only
// the error bits indicate a fault.
//   bit 0: main encoder error     bit 1: output encoder error   bit 2: calibration encoder error
//   bit 3: MOSFET bridge error    bit 4: hardware error         bit 5: communication error
//   bit 6: motion error
constexpr uint16_t MAB_QS_ERROR_MASK = 0x007F; // bits 0..6

// Map a MAB Quick Status onto the nearest CubeMars ErrorCode (the shared contract). Coarse by design:
// the encoder and bridge categories map cleanly; hardware/communication/motion errors have no exact
// CubeMars equivalent and report as MOTOR_STALL. The raw Quick Status carries the precise category;
// surfacing it richly is a future improvement (see docs/MAB_FDCAN_PARITY.md).
ErrorCode mab_quick_status_to_error(uint16_t qs)
{
    if ((qs & MAB_QS_ERROR_MASK) == 0)
    {
        return ErrorCode::NO_FAULT; // target-reached (bit 15) and reserved bits are not faults
    }
    if (qs & 0x07u)
    {
        return ErrorCode::ENCODER_FAULT; // bits 0-2: main / output / calibration encoder errors
    }
    if (qs & 0x08u)
    {
        return ErrorCode::MOSFET_OVER_TEMP; // bit 3: MOSFET bridge error
    }
    return ErrorCode::MOTOR_STALL; // bits 4-6: hardware / communication / motion errors
}
} // namespace

MabFdCan::MabFdCan(const std::string &can_interface, const int &enable_loopback,
                   const std::vector<joint_config_t> &joint_configs,
                   const long &socket_timeout_sec, const long &socket_timeout_usec,
                   unsigned int max_init_connect_trials,
                   bool enable_tx_timestamping, bool enable_can_error_frames)
    : can_interface_(can_interface),
      enable_loopback_(enable_loopback),
      joint_configs_(joint_configs),
      socket_timeout_sec_(socket_timeout_sec),
      socket_timeout_usec_(socket_timeout_usec),
      max_initial_connection_trials_(max_init_connect_trials),
      enable_tx_timestamping_(enable_tx_timestamping),
      enable_can_error_frames_(enable_can_error_frames),
      send_ok_(joint_configs_.size()),
      recv_ok_(joint_configs_.size())
{
    // Configuring CAN socket
    struct sockaddr_can addr;
    struct ifreq ifr;
    can_socket_fd_ = socket(AF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket_fd_ < 0)
    {
        throw can_interface_error(std::format("Failed to create CAN socket - {}", std::string(strerror(errno))));
    }

    // Enable CAN FD frames on the socket. This is the defining difference from the CubeMars backend:
    // MAB electronics use CAN FD, so frames are canfd_frame (up to 64 data bytes) rather than can_frame.
    int enable_canfd = 1;
    if (setsockopt(can_socket_fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd)) != 0)
    {
        close(can_socket_fd_);
        throw can_interface_error(std::format("Failed to enable CAN FD frames - {} ", std::string(strerror(errno))));
    }

    // Configure loopback
    if (setsockopt(can_socket_fd_, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &enable_loopback, sizeof(enable_loopback)) < 0)
    {
        close(can_socket_fd_);
        throw can_interface_error(std::format("Failed to set loopback to {} - {} ", enable_loopback, std::string(strerror(errno))));
    }

    // Set CAN RX filters: deliver only the configured motors' reply frames.
    std::vector<can_filter> rfilter;
    bool zero_added = false;
    for (unsigned int i = 0; i < joint_configs_.size(); i++)
    {
        switch (joint_configs_[i].series_type)
        {
        case SERIES_TYPE::V2:
            if (joint_configs_[i].reply_on_own_id)
            {
                rfilter.push_back({joint_configs_[i].can_id, CAN_SFF_MASK | CAN_EFF_FLAG});
            }
            else if (!zero_added)
            {
                rfilter.push_back({0, CAN_SFF_MASK | CAN_EFF_FLAG});
            }
            break;
        case SERIES_TYPE::V3:
            rfilter.push_back({joint_configs_[i].can_id | ((uint32_t)CAN_PACKET_RESPONSE << 8) | CAN_EFF_FLAG, CAN_EFF_MASK | CAN_EFF_FLAG});
            break;
        }
    }
    if (setsockopt(can_socket_fd_, SOL_CAN_RAW, CAN_RAW_FILTER, rfilter.data(), rfilter.size() * sizeof(can_filter)) < 0)
    {
        close(can_socket_fd_);
        throw can_interface_error(std::format("Failed to setup CAN filter - {} ", std::string(strerror(errno))));
    }

    // Find CAN interface index
    memset(&ifr, 0, sizeof(ifr));
    strcpy(ifr.ifr_name, can_interface_.c_str());
    if (ioctl(can_socket_fd_, SIOCGIFINDEX, &ifr))
    {
        close(can_socket_fd_);
        throw can_interface_error(std::format("Failed to find CAN interface {} - {} ", can_interface, std::string(strerror(errno))));
    }

    // Bind the socket to the interface
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(can_socket_fd_, (struct sockaddr *)&addr, sizeof(struct sockaddr)) < 0)
    {
        close(can_socket_fd_);
        throw can_interface_error(std::format("Failed to bind CAN socket - {} ", std::string(strerror(errno))));
    }

    // Set receive timeout so a missing reply surfaces as a timeout instead of blocking forever.
    struct timeval tv;
    tv.tv_sec = socket_timeout_sec_;
    tv.tv_usec = socket_timeout_usec_;
    if (setsockopt(can_socket_fd_, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof(struct timeval)) < 0)
    {
        close(can_socket_fd_);
        throw can_interface_error(std::format("Failed to set socket option for timeout - {} ", std::string(strerror(errno))));
    }

    // setup vars
    memset(&send_frame_, 0, sizeof(send_frame_));
    send_frame_.len = CAN_MAX_DLEN;
}

MabFdCan::~MabFdCan()
{
    close(can_socket_fd_); // If this goes wrong, we cant do anything
}

void MabFdCan::send_config_frames(const canid_t &can_id, MotorMode_Message mm, MotorState_Message ms)
{
    // NOTE: the write sizes mirror the tested MAB_FDCAN code (the mode write uses the full canfd_frame
    // size, the state write uses the classic can_frame size). Kept as-is to preserve tested behaviour;
    // flagged for review in docs/MAB_FDCAN_PARITY.md.
    send_frame_.can_id = can_id;
    send_frame_.len = sizeof(MotorMode_Message);
    std::memcpy(send_frame_.data, &mm, sizeof(MotorMode_Message));
    if (::write(can_socket_fd_, &send_frame_, sizeof(send_frame_)) < 0)
    {
        throw can_device_error(std::format("Failed to write can frame to can_id {} - {}", std::to_string(can_id), std::string(strerror(errno))));
    }
    memset(&recv_frame_.data, 0, sizeof(Legacy_Response));
    int nbytes = ::read(can_socket_fd_, &recv_frame_, sizeof(recv_frame_));
    if (nbytes <= 0)
    {
        throw can_device_error(std::format("Did not receive reply from can_id {} - {} ", std::to_string(can_id), std::string(strerror(errno))));
    }
    if (recv_frame_.can_id != can_id)
    {
        throw can_device_error(std::format("Reply from can_id {} instead of expected {}", recv_frame_.can_id, can_id));
    }

    send_frame_.can_id = can_id;
    send_frame_.len = sizeof(MotorState_Message);
    std::memcpy(send_frame_.data, &ms, sizeof(MotorState_Message));
    if (::write(can_socket_fd_, &send_frame_, sizeof(struct can_frame)) < 0)
    {
        throw can_device_error(std::format("Failed to write can frame to can_id {} - {}", std::to_string(can_id), std::string(strerror(errno))));
    }
    memset(&recv_frame_.data, 0, sizeof(Legacy_Response));
    nbytes = ::read(can_socket_fd_, &recv_frame_, sizeof(recv_frame_));
    if (nbytes <= 0)
    {
        throw can_device_error(std::format("Did not receive reply from can_id {} - {} ", std::to_string(can_id), std::string(strerror(errno))));
    }
    if (recv_frame_.can_id != can_id)
    {
        throw can_device_error(std::format("Reply from can_id {} instead of expected {}", recv_frame_.can_id, can_id));
    }
}

void MabFdCan::send_zero_frame(const canid_t &can_id, RunZero_Message zm)
{
    send_frame_.can_id = can_id;
    send_frame_.len = sizeof(RunZero_Message);
    std::memcpy(send_frame_.data, &zm, sizeof(RunZero_Message));
    if (::write(can_socket_fd_, &send_frame_, sizeof(struct can_frame)) < 0)
    {
        throw can_device_error(std::format("Failed to write can frame to can_id {} - {}", std::to_string(can_id), std::string(strerror(errno))));
    }
    memset(&recv_frame_, 0, sizeof(recv_frame_));
    int nbytes = ::read(can_socket_fd_, &recv_frame_, CAN_MTU);
    if (nbytes <= 0)
    {
        throw can_device_error(std::format("Did not receive reply from can_id {} - {} ", std::to_string(can_id), std::string(strerror(errno))));
    }
    if (recv_frame_.can_id != can_id)
    {
        throw can_device_error(std::format("Reply from can_id {} instead of expected {}", recv_frame_.can_id, can_id));
    }
}

void MabFdCan::start_motor_control_mode(unsigned int joint_id, bool set_zero_position_on_enable)
{
    if (joint_id >= joint_configs_.size())
    {
        throw std::range_error(std::format("joint_id {} has to be one of the indeces of specified joints", std::to_string(joint_id)));
    }

    // V-style connection priming: MAB controllers can be slow/unresponsive right after power-up, so
    // retry the enable handshake up to max_initial_connection_trials_ times.
    unsigned int trial = 0;
    bool success = false;
    std::string error_msg = "";
    MotorMode_Message mm;
    MotorState_Message sm;
    while (!success && trial++ < max_initial_connection_trials_)
    {
        try
        {
            send_config_frames(joint_configs_[joint_id].can_id, mm, sm);
            success = true;
        }
        catch (const can_device_error &e)
        {
            error_msg += std::string("\t") + e.what() + std::string("\n");
        }
    }
    if (!success)
    {
        throw can_device_error(std::format("Failed to enable motor with can id {} on interface {}, after {} trials. Failures where:\n {}", joint_configs_[joint_id].can_id, can_interface_, max_initial_connection_trials_, error_msg));
    }

    if (set_zero_position_on_enable)
    {
        // Zeroing takes a few seconds, so temporarily raise the receive timeout.
        struct timeval tv;
        tv.tv_sec = 10;
        tv.tv_usec = 0;
        if (setsockopt(can_socket_fd_, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof(struct timeval)) < 0)
        {
            throw can_interface_error(std::format("Failed to set socket option for timeout - {} ", std::string(strerror(errno))));
        }
        RunZero_Message zm;
        send_zero_frame(joint_configs_[joint_id].can_id, zm);

        // Restore the configured receive timeout.
        tv.tv_sec = socket_timeout_sec_;
        tv.tv_usec = socket_timeout_usec_;
        if (setsockopt(can_socket_fd_, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof(struct timeval)) < 0)
        {
            throw can_interface_error(std::format("Failed to set socket option for timeout - {} ", std::string(strerror(errno))));
        }
    }
}

void MabFdCan::end_motor_control_mode(unsigned int joint_id)
{
    if (joint_id >= joint_configs_.size())
    {
        throw std::range_error(std::format("joint_id {} has to be one of the indeces of specified joints (highest joint id {})", std::to_string(joint_id), std::to_string(joint_configs_.size())));
    }
    MotorMode_Message mm;
    MotorState_Message sm;
    sm.register_value = 0x40; // disable
    send_config_frames(joint_configs_[joint_id].can_id, mm, sm);
}

void MabFdCan::start_motor_control_mode(bool set_zero_position_on_enable)
{
    for (unsigned int i = 0; i < joint_configs_.size(); i++)
    {
        start_motor_control_mode(i, set_zero_position_on_enable);
    }
}

void MabFdCan::end_motor_control_mode()
{
    for (unsigned int i = 0; i < joint_configs_.size(); i++)
    {
        end_motor_control_mode(i);
    }
}

void MabFdCan::send_and_receive(const std::vector<joint_cmd_t> &cmds, std::vector<joint_state_t> &states)
{
    if (cmds.size() != states.size() && cmds.size() != joint_configs_.size())
    {
        throw std::out_of_range("cmds, states have to have the correct size of " + std::to_string(joint_configs_.size()));
    }

    // Write all commands as MAB MotionCommand register frames (CAN FD, bit-rate switched).
    // NOTE: invert and per-joint range clamping are NOT applied here yet (the CubeMars backend does
    // both); adding them for MAB is a pending correctness step (see docs/MAB_FDCAN_PARITY.md).
    for (unsigned int i = 0; i < joint_configs_.size(); i++)
    {
        const joint_config_t &cfg = joint_configs_[i];

        // Mirror the CubeMars backend's command conditioning: optionally invert pos/vel/torque, then
        // clamp every commanded quantity to the motor's configured range. fminf/fmaxf is NaN-tolerant
        // (matching CubemarsCan), so an unset range leaves the value untouched. The ranges come from the
        // motor_type preset; confirm they are in the units the MAB controller expects.
        float pos = cfg.invert ? -cmds[i].pos : cmds[i].pos;
        float vel = cfg.invert ? -cmds[i].vel : cmds[i].vel;
        float torque = cfg.invert ? -cmds[i].torque : cmds[i].torque;
        pos = fminf(fmaxf(static_cast<float>(cfg.P_MIN), pos), static_cast<float>(cfg.P_MAX));
        vel = fminf(fmaxf(static_cast<float>(cfg.V_MIN), vel), static_cast<float>(cfg.V_MAX));
        torque = fminf(fmaxf(static_cast<float>(cfg.I_MIN), torque), static_cast<float>(cfg.I_MAX));
        const float kp = fminf(fmaxf(static_cast<float>(cfg.KP_MIN), cmds[i].kp), static_cast<float>(cfg.KP_MAX));
        const float kd = fminf(fmaxf(static_cast<float>(cfg.KD_MIN), cmds[i].kd), static_cast<float>(cfg.KD_MAX));

        MotionCommand_Message cmd;
        cmd.desired_pk = kp;
        cmd.desired_dk = kd;
        cmd.desired_position = pos;
        cmd.desired_velocity = vel;
        cmd.desired_torque = torque;

        send_frame_.can_id = joint_configs_[i].can_id;
        send_frame_.len = sizeof(MotionCommand_Message);
        send_frame_.flags = CANFD_BRS;
        std::memcpy(send_frame_.data, &cmd, sizeof(MotionCommand_Message));

        // Timing fields not measured by this backend yet (timestamping is a following step).
        states[i].send_timestamp_ns = 0;
        states[i].enqueue_timestamp_ns = 0;
        states[i].rx_hw_timestamp_ns = 0;

        if (::write(can_socket_fd_, &send_frame_, sizeof(struct canfd_frame)) < 0)
        {
            states[i].com_errno = errno;
            states[i].communication_status = ComStatus::CAN_WRITE_FAILED;
            send_ok_[i] = false;
        }
        else
        {
            send_ok_[i] = true;
            states[i].communication_status = ComStatus::CAN_NO_RESPONSE; // updated when the reply arrives
        }
    }

    // Receive all replies (Legacy_Response), matched to a joint by can_id.
    for (unsigned int i = 0; i < joint_configs_.size(); i++)
    {
        if (!send_ok_[i])
        {
            continue; // no reply expected if the write failed
        }
        recv_ok_[i] = false;

        memset(&recv_frame_, 0, sizeof(recv_frame_));
        int nbytes = ::read(can_socket_fd_, &recv_frame_, sizeof(recv_frame_));
        if (nbytes < 0)
        {
            states[i].communication_status = ComStatus::CAN_READ_FAILED;
            states[i].com_errno = errno;
            continue;
        }

        const canid_t reply_id = recv_frame_.can_id;
        auto it = std::find_if(joint_configs_.begin(), joint_configs_.end(),
                               [&](const auto &conf) { return conf.can_id == reply_id; });
        if (it == joint_configs_.end())
        {
            continue; // reply from an unknown can_id, ignore
        }
        const unsigned int joint_index = it - joint_configs_.begin();
        recv_ok_[joint_index] = true;

        // Userspace reply time. NOTE: this is not a kernel RX timestamp (this backend does not enable
        // SO_TIMESTAMPNS yet), but it is good enough for stamping ~/joint_states. The full
        // hardware/software timestamping suite is a following step.
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        const int64_t now_ns = static_cast<int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;

        Legacy_Response reply;
        std::memcpy(&reply, recv_frame_.data, sizeof(reply));

        states[joint_index].pos = reply.position;
        states[joint_index].vel = reply.velocity;
        states[joint_index].torque = reply.torque;
        // Undo the command inversion on the reported state, mirroring the CubeMars backend.
        if (joint_configs_[joint_index].invert)
        {
            states[joint_index].pos = -states[joint_index].pos;
            states[joint_index].vel = -states[joint_index].vel;
            states[joint_index].torque = -states[joint_index].torque;
        }
        states[joint_index].temp = static_cast<float>(reply.temperature);
        // Decode the MAB Quick Status into a fault. Only the error-category bits count: the
        // "target reached" status (bit 15) and reserved bits are NOT faults, so normal operation no
        // longer trips a spurious deactivation.
        states[joint_index].device_status = mab_quick_status_to_error(reply.quick_status);
        states[joint_index].rx_timestamp_ns = now_ns;
        states[joint_index].dequeue_timestamp_ns = now_ns;

        states[joint_index].communication_status =
            send_ok_[joint_index] ? ComStatus::SUCCESS : ComStatus::CAN_WRITE_FAILED_BUT_RESPONSE_RECEIVED;
    }
}

} // namespace cubemars
