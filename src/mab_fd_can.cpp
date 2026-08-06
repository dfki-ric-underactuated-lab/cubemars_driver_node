#include "cubemars_hardware_interface/mab_fd_can.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cerrno>
#include <cstring>
#include <ctime>
#include <format>
#include <stdexcept>
#include <thread>

#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can/raw.h>
#include <linux/errqueue.h>
#include <linux/net_tstamp.h>

#include <rclcpp/rclcpp.hpp>

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
// future improvement: MAB Status fully exposed.
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

    // Kernel software RX timestamps (CLOCK_REALTIME ns); always on (cheap)
    // Used to stamp ~/joint_states and the latency topics.
    int ts_on = 1;
    if (setsockopt(can_socket_fd_, SOL_SOCKET, SO_TIMESTAMPNS, &ts_on, sizeof(ts_on)) < 0)
    {
        close(can_socket_fd_);
        throw can_interface_error(std::format("Failed to enable SO_TIMESTAMPNS - {} ", std::string(strerror(errno))));
    }
    // SO_TIMESTAMPING: always request the card hardware RX timestamp (ts[2]); add software TX-completion
    // timestamps (reported on the error queue, drained per cycle) only when enabled.
    // The flags coexist with SO_TIMESTAMPNS.
    int ts_flags = SOF_TIMESTAMPING_RX_HARDWARE | SOF_TIMESTAMPING_RAW_HARDWARE;
    if (enable_tx_timestamping_)
    {
        ts_flags |= SOF_TIMESTAMPING_TX_SOFTWARE | SOF_TIMESTAMPING_SOFTWARE;
    }
    if (setsockopt(can_socket_fd_, SOL_SOCKET, SO_TIMESTAMPING, &ts_flags, sizeof(ts_flags)) < 0)
    {
        close(can_socket_fd_);
        throw can_interface_error(std::format("Failed to enable SO_TIMESTAMPING - {} ", std::string(strerror(errno))));
    }

    // setup vars
    memset(&send_frame_, 0, sizeof(send_frame_));
    send_frame_.len = CAN_MAX_DLEN;
}

MabFdCan::~MabFdCan()
{
    close(can_socket_fd_); // If this goes wrong, we cant do anything
}

int MabFdCan::recv_frame_with_timestamp(struct canfd_frame &frame, int64_t &rx_timestamp_ns, int64_t &rx_hw_timestamp_ns)
{
    struct iovec iov;
    iov.iov_base = &frame;
    iov.iov_len = sizeof(frame);
    // Room for both the SO_TIMESTAMPNS cmsg (1 timespec) and the SCM_TIMESTAMPING cmsg (3 timespecs).
    char ctrl[CMSG_SPACE(sizeof(struct timespec)) + CMSG_SPACE(3 * sizeof(struct timespec))];
    struct msghdr msg;
    memset(&msg, 0, sizeof(msg));
    msg.msg_iov = &iov;
    msg.msg_iovlen = 1;
    msg.msg_control = ctrl;
    msg.msg_controllen = sizeof(ctrl);

    int nbytes = ::recvmsg(can_socket_fd_, &msg, 0);
    rx_timestamp_ns = 0;
    rx_hw_timestamp_ns = 0;
    if (nbytes < 0)
    {
        return nbytes;
    }
    for (struct cmsghdr *c = CMSG_FIRSTHDR(&msg); c != nullptr; c = CMSG_NXTHDR(&msg, c))
    {
        if (c->cmsg_level == SOL_SOCKET && c->cmsg_type == SO_TIMESTAMPNS)
        {
            struct timespec ts;
            memcpy(&ts, CMSG_DATA(c), sizeof(ts));
            rx_timestamp_ns = static_cast<int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
        }
        else if (c->cmsg_level == SOL_SOCKET && c->cmsg_type == SCM_TIMESTAMPING)
        {
            struct timespec ts[3]; // ts[0]=software, ts[1]=legacy(unused), ts[2]=raw hardware
            memcpy(&ts, CMSG_DATA(c), sizeof(ts));
            rx_hw_timestamp_ns = static_cast<int64_t>(ts[2].tv_sec) * 1000000000LL + ts[2].tv_nsec;
        }
    }
    return nbytes;
}

void MabFdCan::collect_tx_timestamps(std::vector<joint_state_t> &states)
{
    // Drain the error queue: each entry carries the echoed command frame and an SCM_TIMESTAMPING control
    // message whose software timestamp (ts[0]) marks TX completion.
    last_tx_errq_count_ = 0;
    while (true)
    {
        struct canfd_frame frame;
        struct iovec iov;
        iov.iov_base = &frame;
        iov.iov_len = sizeof(frame);
        char ctrl[256];
        struct msghdr msg;
        memset(&msg, 0, sizeof(msg));
        msg.msg_iov = &iov;
        msg.msg_iovlen = 1;
        msg.msg_control = ctrl;
        msg.msg_controllen = sizeof(ctrl);

        int nbytes = ::recvmsg(can_socket_fd_, &msg, MSG_ERRQUEUE | MSG_DONTWAIT);
        if (nbytes < 0)
        {
            break; // EAGAIN/EWOULDBLOCK: error queue drained
        }
        last_tx_errq_count_++;

        int64_t tx_ns = 0;
        for (struct cmsghdr *c = CMSG_FIRSTHDR(&msg); c != nullptr; c = CMSG_NXTHDR(&msg, c))
        {
            if (c->cmsg_level == SOL_SOCKET && c->cmsg_type == SCM_TIMESTAMPING)
            {
                struct scm_timestamping tss;
                memcpy(&tss, CMSG_DATA(c), sizeof(tss));
                // ts[0] is the software timestamp (CLOCK_REALTIME); ts[2] would be hardware.
                tx_ns = static_cast<int64_t>(tss.ts[0].tv_sec) * 1000000000LL + tss.ts[0].tv_nsec;
            }
        }
        if (tx_ns == 0)
        {
            continue; // no software timestamp on this entry
        }

        // Match the echoed frame to a joint by can_id (same convention as the reply path).
        const canid_t echoed_id = frame.can_id;
        auto it = std::find_if(joint_configs_.begin(), joint_configs_.end(),
                               [&](const auto &conf) { return conf.can_id == echoed_id; });
        if (it != joint_configs_.end())
        {
            states[it - joint_configs_.begin()].send_timestamp_ns = tx_ns;
        }
    }
}

void MabFdCan::send_register_command(const canid_t &can_id, const void *msg, uint8_t msg_len)
{
    send_frame_.can_id = can_id;
    send_frame_.len = msg_len;
    std::memcpy(send_frame_.data, msg, msg_len);
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
    int16_t register_id;
    std::memcpy(&register_id, static_cast<const uint8_t *>(msg) + 2, sizeof(register_id));
    RCLCPP_DEBUG(rclcpp::get_logger("MabFdCan"), "Register 0x%X write to can_id %u acknowledged (%d bytes received)",
                 register_id, static_cast<unsigned int>(can_id), nbytes);
}

void MabFdCan::send_config_frames(const canid_t &can_id, MotorMode_Message mm, MotorState_Message ms)
{
    // Reset the drive and clear any latched warnings/errors before (re-)configuring it, so a motor
    // coming out of a fault state (or power-up) starts from a clean slate. Each write is
    // acknowledged (matching can_id reply) before moving on, same as the mode/state writes below.
    // CanReinit_Message reinit_msg;
    // send_register_command(can_id, &reinit_msg, sizeof(reinit_msg));
    // //std::this_thread::sleep_for(std::chrono::seconds(0.1));
    // ClearWarnings_Message clear_warnings_msg;
    // send_register_command(can_id, &clear_warnings_msg, sizeof(clear_warnings_msg));
    // ClearErrors_Message clear_errors_msg;
    // send_register_command(can_id, &clear_errors_msg, sizeof(clear_errors_msg));

    // Zero out the PID gains and goal position/velocity/torque before (re-)enabling the motor, so it
    // doesn't briefly chase a stale MotionCommand left over from before the mode/state write below.
    // MotionCommand_Message zero_cmd;
    // send_frame_.can_id = can_id;
    // send_frame_.len = sizeof(MotionCommand_Message);
    // std::memcpy(send_frame_.data, &zero_cmd, sizeof(MotionCommand_Message));
    // if (::write(can_socket_fd_, &send_frame_, sizeof(send_frame_)) < 0)
    // {
    //     throw can_device_error(std::format("Failed to write can frame to can_id {} - {}", std::to_string(can_id), std::string(strerror(errno))));
    // }
    // memset(&recv_frame_.data, 0, sizeof(Legacy_Response));
    // int zero_nbytes = ::read(can_socket_fd_, &recv_frame_, sizeof(recv_frame_));
    // if (zero_nbytes <= 0)
    // {
    //     throw can_device_error(std::format("Did not receive reply from can_id {} - {} ", std::to_string(can_id), std::string(strerror(errno))));
    // }
    // if (recv_frame_.can_id != can_id)
    // {
    //     throw can_device_error(std::format("Reply from can_id {} instead of expected {}", recv_frame_.can_id, can_id));
    // }

    send_frame_.can_id = can_id;
    send_frame_.len = sizeof(MotorMode_Message);
    std::memcpy(send_frame_.data, &mm, sizeof(MotorMode_Message));
    if (::write(can_socket_fd_, &send_frame_, sizeof(send_frame_)) < 0)
    {
        throw can_device_error(std::format("Failed to write can frame to can_id {} - {}", std::to_string(can_id), std::string(strerror(errno))));
    }
    // Same frame_id 0x41 readback semantics as the state write above.
    memset(&recv_frame_.data, 0, sizeof(MotorMode_Message));
    int nbytes = ::read(can_socket_fd_, &recv_frame_, sizeof(recv_frame_));
    if (nbytes <= 0)
    {
        throw can_device_error(std::format("Did not receive reply from can_id {} - {} ", std::to_string(can_id), std::string(strerror(errno))));
    }
    if (recv_frame_.can_id != can_id)
    {
        throw can_device_error(std::format("Reply from can_id {} instead of expected {}", recv_frame_.can_id, can_id));
    }
    MotorMode_Message mm_reply;
    std::memcpy(&mm_reply, recv_frame_.data, sizeof(MotorMode_Message));
    RCLCPP_DEBUG(rclcpp::get_logger("MabFdCan"), "MotorMode write to can_id %u acknowledged (%d bytes received, register_value=%d)",
                 static_cast<unsigned int>(can_id), nbytes, mm_reply.register_value);
    if (mm_reply.register_value != mm.register_value)
    {
        throw can_device_error(std::format("Motor mode register readback mismatch for can_id {}: expected {}, got {}", can_id, mm.register_value, mm_reply.register_value));
    }

    //////////

    send_frame_.can_id = can_id;
    send_frame_.len = sizeof(MotorState_Message);
    std::memcpy(send_frame_.data, &ms, sizeof(MotorState_Message));
    if (::write(can_socket_fd_, &send_frame_, sizeof(send_frame_)) < 0)
    {
        throw can_device_error(std::format("Failed to write can frame to can_id {} - {}", std::to_string(can_id), std::string(strerror(errno))));
    }
    // frame_id 0x41: the reply mirrors the request's struct layout (not a Legacy_Response), with
    // register_value holding the register's current value, so we confirm the write stuck rather
    // than just confirming *a* reply arrived.
    memset(&recv_frame_.data, 0, sizeof(MotorState_Message));
    nbytes = ::read(can_socket_fd_, &recv_frame_, sizeof(recv_frame_));
    if (nbytes <= 0)
    {
        throw can_device_error(std::format("Did not receive reply from can_id {} - {} ", std::to_string(can_id), std::string(strerror(errno))));
    }
    if (recv_frame_.can_id != can_id)
    {
        throw can_device_error(std::format("Reply from can_id {} instead of expected {}", recv_frame_.can_id, can_id));
    }
    MotorState_Message ms_reply;
    std::memcpy(&ms_reply, recv_frame_.data, sizeof(MotorState_Message));
    RCLCPP_DEBUG(rclcpp::get_logger("MabFdCan"), "MotorState write to can_id %u acknowledged (%d bytes received, register_value=%d)",
                 static_cast<unsigned int>(can_id), nbytes, ms_reply.register_value);
    if (ms_reply.register_value != ms.register_value)
    {   
        throw can_device_error(std::format("Motor state register readback mismatch for can_id {}: expected {}, got {}", can_id, ms.register_value, ms_reply.register_value));
    }

}

void MabFdCan::flush_rx_queue()
{
    // Discard any frames already sitting in the socket RX queue. During cyclic operation replies
    // that arrive after a cycle's receive timeout stay queued; a one-shot transaction (e.g. a
    // set-zero handshake) would otherwise read such a stale reply instead of its own acknowledgement.
    struct canfd_frame frame;
    while (::recv(can_socket_fd_, &frame, sizeof(frame), MSG_DONTWAIT) > 0)
    {
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
    // Wait for this motor's acknowledgement, skipping stale replies from other motors that may
    // still trickle in from the last cyclic cycle.
    while (true)
    {
        memset(&recv_frame_, 0, sizeof(recv_frame_));
        int nbytes = ::read(can_socket_fd_, &recv_frame_, CAN_MTU);
        if (nbytes <= 0)
        {
            throw can_device_error(std::format("Did not receive reply from can_id {} - {} ", std::to_string(can_id), std::string(strerror(errno))));
        }
        if (recv_frame_.can_id == can_id)
        {
            return;
        }
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
        set_zero_position(joint_id);
    }
}

void MabFdCan::set_zero_position(unsigned int joint_id)
{
    if (joint_id >= joint_configs_.size())
    {
        throw std::range_error(std::format("joint_id {} has to be one of the indeces of specified joints", std::to_string(joint_id)));
    }

    // Zeroing takes a few seconds, so temporarily raise the receive timeout.
    struct timeval tv;
    tv.tv_sec = 10;
    tv.tv_usec = 0;
    if (setsockopt(can_socket_fd_, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof(struct timeval)) < 0)
    {
        throw can_interface_error(std::format("Failed to set socket option for timeout - {} ", std::string(strerror(errno))));
    }
    flush_rx_queue();
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

void MabFdCan::send_and_receive(const std::vector<joint_cmd_t> &cmds, std::vector<joint_state_t> &states, bool is_active)
{
    if (cmds.size() != states.size() && cmds.size() != joint_configs_.size())
    {
        throw std::out_of_range("cmds, states have to have the correct size of " + std::to_string(joint_configs_.size()));
    }

    struct timespec ts_now;
    clock_gettime(CLOCK_REALTIME, &ts_now);
    const int64_t tx_fill_start_ns = static_cast<int64_t>(ts_now.tv_sec) * 1000000000LL + ts_now.tv_nsec;

    // Write all commands as MAB MotionCommand register frames (CAN FD, bit-rate switched), applying
    // invert and per-joint range clamping (below).
    for (unsigned int i = 0; i < joint_configs_.size(); i++)
    {
        const joint_config_t &cfg = joint_configs_[i];

        // Optionally invert pos/vel/torque, then clamp every commanded quantity to the motor's
        // configured range. fminf/fmaxf is NaN-tolerant so an unset range leaves the value
        // untouched. The ranges come from the motor_type preset;
        //  confirm they are in the units the MAB controller expects.
        float pos = cfg.invert ? -cmds[i].pos : cmds[i].pos;
        float vel = cfg.invert ? -cmds[i].vel : cmds[i].vel;
        float torque = cfg.invert ? -cmds[i].torque : cmds[i].torque;
        pos = fminf(fmaxf(static_cast<float>(cfg.P_MIN), pos), static_cast<float>(cfg.P_MAX));
        vel = fminf(fmaxf(static_cast<float>(cfg.V_MIN), vel), static_cast<float>(cfg.V_MAX));
        torque = fminf(fmaxf(static_cast<float>(cfg.I_MIN), torque), static_cast<float>(cfg.I_MAX));
        const float kp = fminf(fmaxf(static_cast<float>(cfg.KP_MIN), cmds[i].kp), static_cast<float>(cfg.KP_MAX));
        const float kd = fminf(fmaxf(static_cast<float>(cfg.KD_MIN), cmds[i].kd), static_cast<float>(cfg.KD_MAX));
        
        // Default-constructed desired_* fields are already 0.f, so when inactive we simply leave
        // them unset and write zero register values instead of the actual command.
        MotionCommand_Message cmd;
        if (is_active)
        {
            cmd.desired_pk = kp;
            cmd.desired_dk = kd;
            cmd.desired_position = pos;
            cmd.desired_velocity = vel;
            cmd.desired_torque = torque;
        }
        send_frame_.can_id = joint_configs_[i].can_id;
        send_frame_.len = sizeof(MotionCommand_Message);
        send_frame_.flags = CANFD_BRS;
        std::memcpy(send_frame_.data, &cmd, sizeof(MotionCommand_Message));

        // send_timestamp_ns is filled from the error queue by collect_tx_timestamps() once TX completes.
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
            // Userspace moment this frame finished being written into the TX buffer.
            clock_gettime(CLOCK_REALTIME, &ts_now);
            states[i].enqueue_timestamp_ns = static_cast<int64_t>(ts_now.tv_sec) * 1000000000LL + ts_now.tv_nsec;
            send_ok_[i] = true;
            states[i].communication_status = ComStatus::CAN_NO_RESPONSE; // updated when the reply arrives
        }
    }
    // TX buffer is now filled with all command frames.
    clock_gettime(CLOCK_REALTIME, &ts_now);
    tx_fill_end_ns_ = static_cast<int64_t>(ts_now.tv_sec) * 1000000000LL + ts_now.tv_nsec;
    tx_fill_duration_ns_ = tx_fill_end_ns_ - tx_fill_start_ns;

    // Receive all replies (Legacy_Response), matched to a joint by can_id.
    for (unsigned int i = 0; i < joint_configs_.size(); i++)
    {
        if (!send_ok_[i])
        {
            continue; // no reply expected if the write failed
        }
        recv_ok_[i] = false;

        int64_t rx_ns = 0;
        int64_t rx_hw_ns = 0;
        memset(&recv_frame_, 0, sizeof(recv_frame_));
        int nbytes = recv_frame_with_timestamp(recv_frame_, rx_ns, rx_hw_ns);
        if (nbytes < 0)
        {
            states[i].communication_status = ComStatus::CAN_READ_FAILED;
            states[i].com_errno = errno;
            continue;
        }
        // Userspace moment the reply was read into node space.
        clock_gettime(CLOCK_REALTIME, &ts_now);
        const int64_t deq_ns = static_cast<int64_t>(ts_now.tv_sec) * 1000000000LL + ts_now.tv_nsec;

        const canid_t reply_id = recv_frame_.can_id;
        auto it = std::find_if(joint_configs_.begin(), joint_configs_.end(),
                               [&](const auto &conf) { return conf.can_id == reply_id; });
        if (it == joint_configs_.end())
        {
            continue; // reply from an unknown can_id, ignore
        }
        const unsigned int joint_index = it - joint_configs_.begin();
        recv_ok_[joint_index] = true;

        Legacy_Response reply;
        std::memcpy(&reply, recv_frame_.data, sizeof(reply));

        states[joint_index].pos = reply.position;
        states[joint_index].vel = reply.velocity;
        states[joint_index].torque = reply.torque;
        // Undo the command inversion on the reported state.
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
        states[joint_index].rx_timestamp_ns = rx_ns;
        states[joint_index].rx_hw_timestamp_ns = rx_hw_ns;
        states[joint_index].dequeue_timestamp_ns = deq_ns;

        states[joint_index].communication_status =
            send_ok_[joint_index] ? ComStatus::SUCCESS : ComStatus::CAN_WRITE_FAILED_BUT_RESPONSE_RECEIVED;
    }

    // Time spent in the receive phase, and (if enabled) drain the software TX-completion timestamps.
    clock_gettime(CLOCK_REALTIME, &ts_now);
    rx_duration_ns_ = (static_cast<int64_t>(ts_now.tv_sec) * 1000000000LL + ts_now.tv_nsec) - tx_fill_end_ns_;
    if (enable_tx_timestamping_)
    {
        collect_tx_timestamps(states);
    }
}

} // namespace cubemars
