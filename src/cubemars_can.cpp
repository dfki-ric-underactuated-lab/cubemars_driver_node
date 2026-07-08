#include "cubemars_hardware_interface/cubemars_can.hpp"
#include <iostream>
#include <linux/errqueue.h>
#include <linux/net_tstamp.h>
#include <linux/can/error.h>

cubemars::CubemarsCan::CubemarsCan(const std::string &can_interface, const int &enable_loopback, const std::vector<joint_config_t> &joint_configs, const long &socket_timeout_sec, const long &socket_timeout_usec, unsigned int max_init_connect_trials, bool enable_tx_timestamping, bool enable_can_error_frames) : can_interface_(can_interface),
                                                                                                                                                                                                                      enable_loopback_(enable_loopback),
                                                                                                                                                                                                                      enable_tx_timestamping_(enable_tx_timestamping),
                                                                                                                                                                                                                      enable_can_error_frames_(enable_can_error_frames),
                                                                                                                                                                                                                      joint_configs_(joint_configs),
                                                                                                                                                                                                                      socket_timeout_sec_(socket_timeout_sec),
                                                                                                                                                                                                                      socket_timeout_usec_(socket_timeout_usec),
                                                                                                                                                                                                                      max_initial_connection_trials_(max_init_connect_trials),
                                                                                                                                                                                                                      send_ok_(joint_configs_.size()),
                                                                                                                                                                                                                      recv_ok_(joint_configs_.size())
{
    // Configuring CAN socket
    struct sockaddr_can addr;
    struct ifreq ifr;
    can_socket_fd_ = socket(AF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket_fd_ < 0)
    {
        throw cubemars::can_interface_error(std::format("Failed to create CAN socket - {}", std::string(strerror(errno))));
    }
    // Configure loopback
    if (setsockopt(can_socket_fd_, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &enable_loopback, sizeof(enable_loopback)) < 0)
    {
        // Trying to close socket - Ignore failures since we cant do anyways
        close(can_socket_fd_);
        throw cubemars::can_interface_error(std::format("Failed to set loopback to {} - {} ", enable_loopback, std::string(strerror(errno))));
    }
    // Set CAN filter
    std::vector<can_filter> rfilter;
    bool zero_added = false;
    for (unsigned int i = 0; i < joint_configs.size(); i++)
    {
        switch (joint_configs[i].series_type)
        {
        case SERIES_TYPE::V2:
            if (joint_configs[i].reply_on_own_id)
            {
                rfilter.push_back({joint_configs[i].can_id, CAN_SFF_MASK | CAN_EFF_FLAG});
            }
            else if (!zero_added)
            {
                rfilter.push_back({0, CAN_SFF_MASK | CAN_EFF_FLAG});
            }
            break;
        case SERIES_TYPE::V3:
            rfilter.push_back({joint_configs[i].can_id | ((uint32_t)CAN_PACKET_RESPONSE << 8) | CAN_EFF_FLAG, CAN_EFF_MASK | CAN_EFF_FLAG});
            break;
        }
    }

    if (setsockopt(can_socket_fd_, SOL_CAN_RAW, CAN_RAW_FILTER, rfilter.data(), rfilter.size() * sizeof(can_filter)) < 0)
    {
        // Trying to close socket - Ignore failures since we cant do anyways
        close(can_socket_fd_);
        throw cubemars::can_interface_error(std::format("Failed to setup CAN filter - {} ", std::string(strerror(errno))));
    }
    // find CAN Interface index
    memset(&ifr, 0, sizeof(ifr));
    strcpy(ifr.ifr_name, can_interface_.c_str());
    if (ioctl(can_socket_fd_, SIOCGIFINDEX, &ifr))
    {
        // Trying to close socket - Ignore failures since we cant do anyways
        close(can_socket_fd_);
        throw cubemars::can_interface_error(std::format("Failed to find CAN interface {} - {} ", can_interface, std::string(strerror(errno))));
    }
    // bind CAN sock
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (
        bind(can_socket_fd_,
             (struct sockaddr *)&addr,
             sizeof(struct sockaddr)) < 0)
    {
        // Trying to close socket - Ignore failures since we cant do anyways
        close(can_socket_fd_);
        throw cubemars::can_interface_error(std::format("Failed to bind CAN socket - {} ", std::string(strerror(errno))));
    }
    // set socket timeout
    struct timeval tv;
    tv.tv_sec = socket_timeout_sec;
    tv.tv_usec = socket_timeout_usec;
    if (
        setsockopt(can_socket_fd_,
                   SOL_SOCKET,
                   SO_RCVTIMEO,
                   (const char *)&tv,
                   sizeof(struct timeval)) < 0)
    {
        // Trying to close socket - Ignore failures since we cant do anyways
        close(can_socket_fd_);
        throw cubemars::can_interface_error(std::format("Failed to set socket option for timeout - {} ", std::string(strerror(errno))));
    }
    // enable per-frame kernel RX timestamps (CLOCK_REALTIME ns)
    int ts_on = 1;
    if (setsockopt(can_socket_fd_, SOL_SOCKET, SO_TIMESTAMPNS, &ts_on, sizeof(ts_on)) < 0)
    {
        close(can_socket_fd_);
        throw cubemars::can_interface_error(std::format("Failed to enable SO_TIMESTAMPNS - {} ", std::string(strerror(errno))));
    }
    // Enable SO_TIMESTAMPING. The hardware RX timestamp (raw card clock, reported as ts[2]) is always
    // requested: it captures when the card actually received a reply off the wire, so comparing it to
    // the software RX timestamp (SO_TIMESTAMPNS, above) separates a reply that was late on the wire
    // from one the kernel delivered late. Software TX timestamps (CLOCK_REALTIME ns of TX completion,
    // reported on the error queue) are added only when requested; when off, no error-queue entries are
    // generated so the per-cycle drain is skipped too. These flags coexist with SO_TIMESTAMPNS.
    int ts_flags = SOF_TIMESTAMPING_RX_HARDWARE | SOF_TIMESTAMPING_RAW_HARDWARE;
    if (enable_tx_timestamping_)
    {
        ts_flags |= SOF_TIMESTAMPING_TX_SOFTWARE | SOF_TIMESTAMPING_SOFTWARE;
    }
    if (setsockopt(can_socket_fd_, SOL_SOCKET, SO_TIMESTAMPING, &ts_flags, sizeof(ts_flags)) < 0)
    {
        close(can_socket_fd_);
        throw cubemars::can_interface_error(std::format("Failed to enable SO_TIMESTAMPING - {} ", std::string(strerror(errno))));
    }
    // Optionally deliver CAN bus-error frames (bus-off, ACK errors, controller problems, ...) on the
    // RX queue. They arrive mixed with replies and are recognized/skipped in send_and_receive().
    if (enable_can_error_frames_)
    {
        can_err_mask_t err_mask = CAN_ERR_MASK; // all error classes
        if (setsockopt(can_socket_fd_, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof(err_mask)) < 0)
        {
            close(can_socket_fd_);
            throw cubemars::can_interface_error(std::format("Failed to set CAN_RAW_ERR_FILTER - {} ", std::string(strerror(errno))));
        }
    }

    // setup vars
    memset(&send_frame_, 0, sizeof(send_frame_));
    send_frame_.len = CAN_MAX_DLEN;
}

int cubemars::CubemarsCan::recv_frame_with_timestamp(struct can_frame &frame, int64_t &rx_timestamp_ns, int64_t &rx_hw_timestamp_ns)
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

cubemars::CubemarsCan::~CubemarsCan()
{
    close(can_socket_fd_); // If this goes wrong, we cant do anything
}

void cubemars::CubemarsCan::send_control_frameV2(const canid_t &can_id, const std::array<uint8_t, CAN_MAX_DLEN> &control_sequence)
{

    send_frame_.can_id = can_id;
    if (control_sequence.data() != send_frame_.data)
    { // Only copy if nececarry
        std::copy(control_sequence.begin(), control_sequence.end(), send_frame_.data);
    }
    if (::write(can_socket_fd_, &send_frame_, sizeof(struct can_frame)) < 0)
    {
        throw cubemars::can_device_error(std::format("Failed to write can frame to can_id {} - {}", std::to_string(can_id), std::string(strerror(errno))));
    }
    // Receive answer
    memset(&recv_frame_, 0, sizeof(recv_frame_));
    int nbytes = ::read(can_socket_fd_, &recv_frame_, CAN_MTU);
    if (nbytes <= 0)
    {
        throw cubemars::can_device_error(std::format("Did not receive reply from can_id {} - {} ", std::to_string(can_id), std::string(strerror(errno))));
    }
    auto id = recv_frame_.data[0];
    if (id != can_id)
    {
        throw cubemars::can_device_error(std::format("Reply from can_id {} instead of expected {}", recv_frame_.can_id, can_id));
    }
    auto err_code = static_cast<cubemars::ErrorCode>(recv_frame_.data[7]);
    if (err_code != cubemars::ErrorCode::NO_FAULT)
    {
        throw cubemars::motor_error(std::format("Error on motor with can_id {} - {} ", std::to_string(can_id), errorFlagToString(err_code)));
    }
}

void cubemars::CubemarsCan::send_and_receive(const std::vector<joint_cmd_t> &cmds, std::vector<joint_state_t> &states)
{
    if (cmds.size() != states.size() && cmds.size() != joint_configs_.size())
    {
        throw std::out_of_range("cmds, states have to have the correct size of " + joint_configs_.size());
    }
    struct timespec ts_now;
    clock_gettime(CLOCK_REALTIME, &ts_now);
    int64_t tx_fill_start_ns = static_cast<int64_t>(ts_now.tv_sec) * 1000000000LL + ts_now.tv_nsec;
    // Write all cmds
    for (unsigned int i = 0; i < joint_configs_.size(); i++)
    {
        float p_des_f;
        float v_des_f;
        float t_ff_f;

        if (joint_configs_[i].invert)
        {
            p_des_f = -cmds[i].pos;
            v_des_f = -cmds[i].vel;
            t_ff_f = -cmds[i].torque;
        }
        else
        {
            p_des_f = cmds[i].pos;
            v_des_f = cmds[i].vel;
            t_ff_f = cmds[i].torque;
        }

        uint16_t p_des = float_to_uint(
            fminf(
                fmaxf(joint_configs_[i].P_MIN, p_des_f),
                joint_configs_[i].P_MAX),
            joint_configs_[i].P_MIN,
            joint_configs_[i].P_MAX,
            16);
        uint16_t v_des = float_to_uint(
            fminf(
                fmaxf(joint_configs_[i].V_MIN, v_des_f),
                joint_configs_[i].V_MAX),
            joint_configs_[i].V_MIN,
            joint_configs_[i].V_MAX,
            12);
        uint16_t t_ff = float_to_uint(
            fminf(
                fmaxf(joint_configs_[i].I_MIN, t_ff_f),
                joint_configs_[i].I_MAX),
            joint_configs_[i].I_MIN,
            joint_configs_[i].I_MAX,
            12);
        uint16_t kp = float_to_uint(
            fminf(
                fmaxf(joint_configs_[i].KP_MIN, cmds[i].kp),
                joint_configs_[i].KP_MAX),
            joint_configs_[i].KP_MIN,
            joint_configs_[i].KP_MAX,
            12);
        uint16_t kd = float_to_uint(
            fminf(
                fmaxf(joint_configs_[i].KD_MIN, cmds[i].kd),
                joint_configs_[i].KD_MAX),
            joint_configs_[i].KD_MIN,
            joint_configs_[i].KD_MAX,
            12);

        switch (joint_configs_[i].series_type)
        {
        case V2:
            send_frame_.data[0] = p_des >> 8;                       // Position High 8
            send_frame_.data[1] = p_des & 0xFF;                     // Position Low 8
            send_frame_.data[2] = v_des >> 4;                       // Speed High 8 bits
            send_frame_.data[3] = ((v_des & 0xF) << 4) | (kp >> 8); // Speed Low 4 bits KP High 4 bits
            send_frame_.data[4] = kp & 0xFF;                        // KP Low 8 bits
            send_frame_.data[5] = kd >> 4;                          // kp High 8 bits
            send_frame_.data[6] = ((kd & 0xF) << 4) | (t_ff >> 8);  // KP Low 4 bits Torque High 4 bits
            send_frame_.data[7] = t_ff & 0xff;                      // Torque Low 8 bits
            send_frame_.can_id = joint_configs_[i].can_id;
            break;
        case V3:
            send_frame_.data[0] = kp >> 4;                            // KP high 8 bits
            send_frame_.data[1] = ((kp & 0xF) << 4) | (kd >> 8);      // KP Low 4 bits, Kd High 4 bits
            send_frame_.data[2] = kd & 0xFF;                          // Kd low 8 bits
            send_frame_.data[3] = p_des >> 8;                         // position high 8 bits
            send_frame_.data[4] = p_des & 0xFF;                       // position low 8 bits
            send_frame_.data[5] = v_des >> 4;                         // speed high 8 bits
            send_frame_.data[6] = ((v_des & 0xF) << 4) | (t_ff >> 8); // speed low 4 bits torque high 4 bits
            send_frame_.data[7] = t_ff & 0xff;                        // torque low 8 bits
            send_frame_.can_id = ((joint_configs_[i].can_id | ((uint32_t)CAN_PACKET_SET_MIT << 8)) & CAN_EFF_MASK) | CAN_EFF_FLAG;
            break;
        }

        states[i].send_timestamp_ns = 0; // Filled from the error queue in collect_tx_timestamps() once TX completes
        states[i].dequeue_timestamp_ns = 0; // Filled in the receive loop below when the reply is read
        states[i].enqueue_timestamp_ns = 0;
        if (::write(can_socket_fd_, &send_frame_, sizeof(struct can_frame)) < 0)
        {
            states[i].com_errno = errno;
            states[i].communication_status = ComStatus::CAN_WRITE_FAILED;
            send_ok_[i] = false;
        }
        else
        {
            // Userspace moment this frame finished being written into the TX buffer.
            struct timespec enq_ts;
            clock_gettime(CLOCK_REALTIME, &enq_ts);
            states[i].enqueue_timestamp_ns = static_cast<int64_t>(enq_ts.tv_sec) * 1000000000LL + enq_ts.tv_nsec;
            send_ok_[i] = true;
            states[i].communication_status = ComStatus::CAN_NO_RESPONSE; // Will be updated when reply is there
        }
    }
    // TX buffer is now filled with all command frames.
    clock_gettime(CLOCK_REALTIME, &ts_now);
    tx_fill_end_ns_ = static_cast<int64_t>(ts_now.tv_sec) * 1000000000LL + ts_now.tv_nsec;
    tx_fill_duration_ns_ = tx_fill_end_ns_ - tx_fill_start_ns;
    // Receive all commands
    last_error_count_ = 0;
    for (unsigned int i = 0; i < joint_configs_.size(); i++)
    {
        if (!send_ok_[i])
        {
            continue; // No point in waiting for reply if send failed
        }
        recv_ok_[i] = false; // Will be set when reply is there

        int64_t rx_ns = 0;
        int64_t rx_hw_ns = 0;
        int nbytes;
        // Read the next non-error frame. When CAN error frames are enabled they arrive here mixed
        // with replies; record and skip them (read one more), bounded against an error storm. When
        // disabled, CAN_ERR_FLAG is never set so this reads exactly one frame as before.
        unsigned int err_skips = 0;
        while (true)
        {
            memset(&recv_frame_, 0, CAN_MTU);
            nbytes = recv_frame_with_timestamp(recv_frame_, rx_ns, rx_hw_ns);
            if (nbytes < 0)
            {
                break; // timeout / read error
            }
            if (recv_frame_.can_id & CAN_ERR_FLAG)
            {
                last_error_canid_ = recv_frame_.can_id;
                last_error_count_++;
                if (++err_skips >= 16)
                {
                    nbytes = -1; // storm guard: stop reading and treat this reply as missing
                    break;
                }
                continue; // read another frame for the actual reply
            }
            break; // a real (non-error) frame
        }
        if (nbytes < 0)
        {
            states[i].communication_status = ComStatus::CAN_READ_FAILED;
            states[i].com_errno = errno;
            continue;
        }
        // Userspace arrival ("node space") of this reply
        clock_gettime(CLOCK_REALTIME, &ts_now);
        int64_t deq_ns = static_cast<int64_t>(ts_now.tv_sec) * 1000000000LL + ts_now.tv_nsec;
        unsigned int joint_index = 0;
        // The v2 motors reply with standard CAN IDs, the v3 motors with extended CAN IDs
        if ((recv_frame_.can_id & CAN_EFF_FLAG))
        { // V3 motor
            if (((recv_frame_.can_id & CAN_EFF_MASK) >> 8) != CAN_PACKET_RESPONSE)
            {

                continue; // Not a reply frame, ignore
            }
            auto can_id = recv_frame_.can_id & 0xFF; // Getting only the lower 8 bits since thats the actual can_id
            auto it = std::find_if(
                joint_configs_.begin(),
                joint_configs_.end(),
                [&](const auto &conf)
                { return conf.can_id == can_id; });

            if (it == joint_configs_.end())
            {
                continue; // There is an reply from unknown can_id, ignore and continue
            }
            joint_index = it - joint_configs_.begin();
            recv_ok_[joint_index] = true;
            states[joint_index].rx_timestamp_ns = rx_ns;
            states[joint_index].rx_hw_timestamp_ns = rx_hw_ns;
            states[joint_index].dequeue_timestamp_ns = deq_ns;

            int16_t p_int = recv_frame_.data[0] << 8 | recv_frame_.data[1];
            int16_t v_int = recv_frame_.data[2] << 8 | recv_frame_.data[3];
            int16_t i_int = recv_frame_.data[4] << 8 | recv_frame_.data[5];
            int8_t temp_int = recv_frame_.data[6]; // Motor temperature
            cubemars::ErrorCode error_code = static_cast<cubemars::ErrorCode>(recv_frame_.data[7]);
            states[joint_index].pos = (float)(p_int * 0.1f) * (M_PI / 180.0);       // Motor position in DEG (to RAD)
            states[joint_index].vel = (float)(v_int) * 10 * ((2 * M_PI) / (joint_configs_[joint_index].numer_of_pole_pairs * joint_configs_[joint_index].gear_ratio * 60.0)); // Motor speed in ERPM (to RAD/s)
            states[joint_index].torque = (float)(i_int * 0.01f) * joint_configs_[joint_index].torque_constant;   // Motor current in AMPS (to Torque)
            states[joint_index].temp = temp_int;
            states[joint_index].device_status = error_code;
        }
        else
        { // V2 motor
            // The V2 motors carry their can_id in the first data byte (and depending on the minor version the reply id is either the same as the can_id or 0, but this is handled in the filter)
            auto can_id = recv_frame_.data[0];
            auto it = std::find_if(
                joint_configs_.begin(),
                joint_configs_.end(),
                [&](const auto &conf)
                { return conf.can_id == can_id; });

            if (it == joint_configs_.end())
            {

                continue; // There is an reply from unknown can_id, ignore and continue
            }
            joint_index = it - joint_configs_.begin();
            recv_ok_[joint_index] = true;
            states[joint_index].rx_timestamp_ns = rx_ns;
            states[joint_index].rx_hw_timestamp_ns = rx_hw_ns;
            states[joint_index].dequeue_timestamp_ns = deq_ns;

            uint16_t p_int = (recv_frame_.data[1] << 8) | recv_frame_.data[2];         // Motor Position Data
            uint16_t v_int = (recv_frame_.data[3] << 4) | (recv_frame_.data[4] >> 4);  // Motor Speed Data
            uint16_t i_int = ((recv_frame_.data[4] & 0xF) << 8) | recv_frame_.data[5]; // Motor Torque Data
            uint8_t temp_int = recv_frame_.data[6];
            cubemars::ErrorCode error_code = static_cast<cubemars::ErrorCode>(recv_frame_.data[7]);

            /// convert ints to floats ///
            states[joint_index].pos = uint_to_float(p_int, it->P_MIN, it->P_MAX, 16);
            states[joint_index].vel = uint_to_float(v_int, it->V_MIN, it->V_MAX, 12);
            states[joint_index].torque = uint_to_float(i_int, it->I_MIN, it->I_MAX, 12);
            states[joint_index].temp = temp_int - 40;
            states[joint_index].device_status = error_code;
        }

        if (joint_configs_[joint_index].invert)
        {
            states[joint_index].pos = -states[joint_index].pos;
            states[joint_index].vel = -states[joint_index].vel;
            states[joint_index].torque = -states[joint_index].torque;
        }
        if (send_ok_[joint_index])
        {
            states[joint_index].communication_status = ComStatus::SUCCESS;
        }
        else
        {
            states[joint_index].communication_status = ComStatus::CAN_WRITE_FAILED_BUT_RESPONSE_RECEIVED;
        }
    }
    // All replies have been received (or timed out): duration of the receive phase since the TX buffer was filled.
    clock_gettime(CLOCK_REALTIME, &ts_now);
    rx_duration_ns_ = (static_cast<int64_t>(ts_now.tv_sec) * 1000000000LL + ts_now.tv_nsec) - tx_fill_end_ns_;
    // Replies are in, so the command frames have completed transmission: pull their TX timestamps.
    if (enable_tx_timestamping_)
    {
        collect_tx_timestamps(states);
    }
}

void cubemars::CubemarsCan::collect_tx_timestamps(std::vector<joint_state_t> &states)
{
    // Drain the error queue: each entry carries the original (echoed) command frame in the iov
    // and an SCM_TIMESTAMPING control message whose software timestamp (ts[0]) marks TX completion.
    last_tx_errq_count_ = 0;
    while (true)
    {
        struct can_frame frame;
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
        last_tx_errq_count_++; // an entry was present on the error queue

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
            continue; // No software timestamp on this entry
        }

        // Match the echoed frame back to a joint by its can_id (same convention as the reply path).
        canid_t can_id = (frame.can_id & CAN_EFF_FLAG) ? (frame.can_id & 0xFF) : (frame.can_id & CAN_SFF_MASK);
        auto it = std::find_if(
            joint_configs_.begin(),
            joint_configs_.end(),
            [&](const auto &conf)
            { return conf.can_id == can_id; });
        if (it != joint_configs_.end())
        {
            states[it - joint_configs_.begin()].send_timestamp_ns = tx_ns;
        }
    }
}

void cubemars::CubemarsCan::start_motor_control_mode(unsigned int joint_id, bool set_zero_postion_on_enable)
{
    if (joint_id >= joint_configs_.size())
    {
        throw std::range_error(std::format("joint_id {} has to be one of the indeces of specified joints", std::to_string(joint_id)));
    }


    // The v3 motors can (on power disconnect while being active, i.e. the emergency stop case) get stuck in their response. To ommit this they need a few read cycles toget active again, this we will do here
    if(joint_configs_[joint_id].series_type == V3){
        unsigned int trial = 0;
        bool success = false;
        std::string error_msg = "";
        while(!success && trial++ < max_initial_connection_trials_){
            try
            {
                send_control_frameV3<4>(joint_configs_[joint_id].can_id, CAN_PACKET_SET_CURRENT, {0x00, 0x00, 0x00, 0x00}); // Zero current command
                success = true; //Sucess if we reach here
            }
            catch(const cubemars::can_device_error& e)
            {
                error_msg += std::string("\t") + e.what() + std::string("\n");
            }

        }
        if(!success){
            throw cubemars::can_device_error(std::format("Failed to enable motor with can id {} on interface {}, after {} trials. Failures where:\n {}", joint_configs_[joint_id].can_id, can_interface_, max_initial_connection_trials_, error_msg));
        }
    
    }

    if (set_zero_postion_on_enable)
    {
        // Increasing timeout as this takes a few seconds
        // set socket timeout
        struct timeval tv;
        tv.tv_sec = 10;
        tv.tv_usec = 0;
        if (
            setsockopt(can_socket_fd_,
                       SOL_SOCKET,
                       SO_RCVTIMEO,
                       (const char *)&tv,
                       sizeof(struct timeval)) < 0)
        {
            throw cubemars::can_interface_error(std::format("Failed to set socket option for timeout - {} ", std::string(strerror(errno))));
        }
        switch (joint_configs_[joint_id].series_type)
        {
        case V2:
            send_control_frameV2(joint_configs_[joint_id].can_id, cubemars::SET_ZERO_POSITION);
            break;
        case V3:
            send_control_frameV3<1>(joint_configs_[joint_id].can_id, CAN_PACKET_SET_ORIGIN_HERE, {0x1});
            // the V3 motors will responde once (thats why this call comes back, but then they need time to set the zero position internally)
            break;
        }
    }
    switch (joint_configs_[joint_id].series_type)
    {
    case V2:
        send_control_frameV2(joint_configs_[joint_id].can_id, cubemars::START_MOTOR_CONTROL_MODE);
        break;

    case V3:
        // V3 motors do not need an extra command to start motor control mode
        // But we send a "do nothing command" to make sure the motor is available
        send_control_frameV3<4>(joint_configs_[joint_id].can_id, CAN_PACKET_SET_CURRENT, {0x00, 0x00, 0x00, 0x00}); // Zero current command
        break;
    }
    if(set_zero_postion_on_enable){
        // Resetting timeout
        struct timeval tv;
        tv.tv_sec = socket_timeout_sec_;
        tv.tv_usec = socket_timeout_usec_;
        if (
            setsockopt(can_socket_fd_,
                       SOL_SOCKET,
                       SO_RCVTIMEO,
                       (const char *)&tv,
                       sizeof(struct timeval)) < 0)
        {
            throw cubemars::can_interface_error(std::format("Failed to set socket option for timeout - {} ", std::string(strerror(errno))));
        }
    }
}

void cubemars::CubemarsCan::set_zero_position(unsigned int joint_id)
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
        throw cubemars::can_interface_error(std::format("Failed to set socket option for timeout - {} ", std::string(strerror(errno))));
    }
    switch (joint_configs_[joint_id].series_type)
    {
    case V2:
        send_control_frameV2(joint_configs_[joint_id].can_id, cubemars::SET_ZERO_POSITION);
        break;
    case V3:
        send_control_frameV3<1>(joint_configs_[joint_id].can_id, CAN_PACKET_SET_ORIGIN_HERE, {0x1});
        // the V3 motors will responde once (thats why this call comes back, but then they need time to set the zero position internally)
        break;
    }
    // Restore the configured receive timeout.
    tv.tv_sec = socket_timeout_sec_;
    tv.tv_usec = socket_timeout_usec_;
    if (setsockopt(can_socket_fd_, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof(struct timeval)) < 0)
    {
        throw cubemars::can_interface_error(std::format("Failed to set socket option for timeout - {} ", std::string(strerror(errno))));
    }
}

void cubemars::CubemarsCan::end_motor_control_mode(unsigned int joint_id)
{
    if (joint_id >= joint_configs_.size())
    {
        throw std::range_error(std::format("joint_id {} has to be one of the indeces of specified joints (highest joint id {})", std::to_string(joint_id), std::to_string(joint_configs_.size())));
    }
    switch (joint_configs_[joint_id].series_type)
    {
    case V2:
        send_control_frameV2(joint_configs_[joint_id].can_id, cubemars::EXIT_MOTOR_CONTROL_MODE);
        break;

    case V3:
        // V3 motors do not need an extra command to end motor control cycle, but we send a "do nothing command" to make sure the motor is off
        send_control_frameV3<4>(joint_configs_[joint_id].can_id, CAN_PACKET_SET_CURRENT, {0x00, 0x00, 0x00, 0x00}); // Zero current command
        break;
    }
}

void cubemars::CubemarsCan::start_motor_control_mode(bool set_zero_postion_on_enable)
{
    for (unsigned int i = 0; i < joint_configs_.size(); i++)
    {
        start_motor_control_mode(i, set_zero_postion_on_enable);
    }
}

void cubemars::CubemarsCan::end_motor_control_mode()
{
    for (unsigned int i = 0; i < joint_configs_.size(); i++)
    {
        end_motor_control_mode(i);
    }
}
