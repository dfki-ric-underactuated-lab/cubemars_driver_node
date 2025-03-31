#include "cubemars_hardware_interface/cubemars_can.hpp"
#include <iostream>

cubemars::CubemarsCan::CubemarsCan(const std::string &can_interface, const int &enable_loopback, const std::vector<joint_config_t> &joint_configs, const long &socket_timeout_usec, bool set_zero_postion_on_enable) : can_interface_(can_interface),
                                                                                                                                                                                                                       enable_loopback_(enable_loopback),
                                                                                                                                                                                                                       joint_configs_(joint_configs),
                                                                                                                                                                                                                       set_zero_postion_on_enable_(set_zero_postion_on_enable)
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
    rfilter.resize(joint_configs.size());
    for (unsigned int i = 0; i < joint_configs.size(); i++)
    {
        rfilter[i].can_id = joint_configs[i].can_id;
        rfilter[i].can_mask = CAN_SFF_MASK;
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
    tv.tv_sec = 0;
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

    // setup vars
    memset(&send_frame_, 0, sizeof(send_frame_));
    send_frame_.len = CAN_MAX_DLEN;
}

cubemars::CubemarsCan::~CubemarsCan()
{
    close(can_socket_fd_); // If this goes wrong, we cant do anything
}

void cubemars::CubemarsCan::send_control_frame(const canid_t &can_id, const std::array<uint8_t, CAN_MAX_DLEN> &control_sequence)
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
    memset(&recv_frame_, 0, CAN_MTU);
    int nbytes = ::read(can_socket_fd_, &recv_frame_, CAN_MTU);
    if (nbytes <= 0)
    {
        throw cubemars::can_device_error(std::format("Did not receive reply from can_id {} - {} ", std::to_string(can_id), std::string(strerror(errno))));
    }
    if (recv_frame_.can_id != can_id)
    {
        throw cubemars::can_device_error(std::format("Reply from can_id {} instead of expected {}", recv_frame_.can_id, can_id));
    }
    auto err_code = static_cast<cubemars::ErrorCode>(recv_frame_.data[7]);
    if (err_code != cubemars::ErrorCode::FAULT_CODE_NONE)
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

        send_frame_.data[0] = p_des >> 8;                       // Position High 8
        send_frame_.data[1] = p_des & 0xFF;                     // Position Low 8
        send_frame_.data[2] = v_des >> 4;                       // Speed High 8 bits
        send_frame_.data[3] = ((v_des & 0xF) << 4) | (kp >> 8); // Speed Low 4 bits KP High 4 bits
        send_frame_.data[4] = kp & 0xFF;                        // KP Low 8 bits
        send_frame_.data[5] = kd >> 4;                          // kp High 8 bits
        send_frame_.data[6] = ((kd & 0xF) << 4) | (t_ff >> 8);  // KP Low 4 bits Torque High 4 bits
        send_frame_.data[7] = t_ff & 0xff;                      // Torque Low 8 bits

        send_frame_.can_id = joint_configs_[i].can_id;
        if (::write(can_socket_fd_, &send_frame_, sizeof(struct can_frame)) < 0)
        {
            throw cubemars::can_device_error("Failed to write can frame to can_id " + std::to_string(joint_configs_[i].can_id) + std::string(strerror(errno)));
        }
    }
    // Receive all commands
    for (unsigned int i = 0; i < joint_configs_.size(); i++)
    {
        memset(&recv_frame_, 0, CAN_MTU);
        int nbytes = ::read(can_socket_fd_, &recv_frame_, CAN_MTU);
        if (nbytes < 0)
        {
            throw cubemars::can_device_error(std::format("Failed to read from can id {} on interface {} - {} ", std::to_string(joint_configs_[i].can_id), can_interface_ ,std::string(strerror(errno))));
        }
        if (recv_frame_.can_id == 0)
        {
            // TODO: (taken from MT) more sophisticated error handling here
            throw cubemars::can_device_error(std::format("Wrong can id received during read from can interface {} - {} ", std::to_string(joint_configs_[i].can_id), std::string(strerror(errno))));
        }
        // Lookup can_id
        auto it = std::find_if(
            joint_configs_.begin(),
            joint_configs_.end(),
            [&](const auto &conf)
            { return conf.can_id == recv_frame_.can_id; });

        if (it == joint_configs_.end())
        {
            throw cubemars::can_device_error("Received reply from unknown can device - " + recv_frame_.can_id);
        }
        unsigned int joint_index = it - joint_configs_.begin();

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

        if (it->invert)
        {
            states[joint_index].pos = -states[joint_index].pos;
            states[joint_index].vel = -states[joint_index].vel;
            states[joint_index].torque = -states[joint_index].torque;
            states[joint_index].status = error_code;
        }
    }
}

void cubemars::CubemarsCan::start_motor_control_mode(unsigned int joint_id)
{
    if (joint_id >= joint_configs_.size())
    {
        throw std::range_error(std::format("joint_id {} has to be one of the indeces of specified joints", std::to_string(joint_id)));
    }
    if (set_zero_postion_on_enable_)
    {
        send_control_frame(joint_configs_[joint_id].can_id, cubemars::SET_ZERO_POSITION);
    }
    send_control_frame(joint_configs_[joint_id].can_id, cubemars::START_MOTOR_CONTROL_MODE);
}

void cubemars::CubemarsCan::end_motor_control_mode(unsigned int joint_id)
{
    if (joint_id >= joint_configs_.size())
    {
        throw std::range_error(std::format("joint_id {} has to be one of the indeces of specified joints", std::to_string(joint_id)));
    }
    send_control_frame(joint_configs_[joint_id].can_id, cubemars::EXIT_MOTOR_CONTROL_MODE);
}

void cubemars::CubemarsCan::start_motor_control_mode()
{
    for (unsigned int i = 0; i < joint_configs_.size(); i++)
    {
        start_motor_control_mode(i);
    }
}

void cubemars::CubemarsCan::end_motor_control_mode()
{
    for (unsigned int i = 0; i < joint_configs_.size(); i++)
    {
        end_motor_control_mode(i);
    }
}
