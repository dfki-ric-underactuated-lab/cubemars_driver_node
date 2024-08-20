#include "cubemars_hardware_interface/cubemars_hardware_interface.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace cubemars_hardware_interface
{
    hardware_interface::CallbackReturn CubemarsHardwareInterface::on_init(
        const hardware_interface::HardwareInfo &info)
    {
        if (
            hardware_interface::SystemInterface::on_init(info) !=
            hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }
        can_interface_ = info_.hardware_parameters["can_interface"];

        if (info_.hardware_parameters["enable_loopback"].empty())
        {
            enable_loopback_ = 0;
        }
        else
        {
            enable_loopback_ = stoi(info_.hardware_parameters["enable_loopback"]);
        }
        hw_joint_configs_.clear();
        for (auto joint : info_.joints)
        {
            cubemars::joint_config_t conf;
            conf.name = joint.name;
            conf.can_id = stoi(joint.parameters["can_id"]);
            conf.kd = stod(joint.parameters["kd"]);
            conf.kp = stod(joint.parameters["kp"]);
            conf.kd_MIN = stod(joint.parameters["kd_min"]);
            conf.kd_MAX = stod(joint.parameters["kd_max"]);
            conf.kp_MIN = stod(joint.parameters["kp_min"]);
            conf.kp_MAX = stod(joint.parameters["kp_max"]);
            for (auto command_interface : joint.command_interfaces)
            {
                if (command_interface.name == hardware_interface::HW_IF_POSITION)
                {
                    conf.P_MIN = stod(command_interface.min);
                    conf.P_MAX = stod(command_interface.max);
                }
                else if (command_interface.name == hardware_interface::HW_IF_VELOCITY)
                {
                    conf.V_MIN = stod(command_interface.min);
                    conf.V_MAX = stod(command_interface.max);
                }
                else if (command_interface.name == hardware_interface::HW_IF_EFFORT)
                {
                    conf.I_MIN = stod(command_interface.min);
                    conf.I_MAX = stod(command_interface.max);
                }
            }

            hw_joint_configs_.push_back(conf);
            RCLCPP_INFO(
                rclcpp::get_logger("CubemarsHardwareInterface"), "Using joint %s (id=%i)", conf.name.c_str(), conf.can_id);
        }

        hw_commands_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_commands_velocity_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_commands_effort_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_states_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_states_velocity_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_states_effort_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_states_temperature_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_control_level_.resize(info_.joints.size(), cubemars::JointMode::UNDEFINED);
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn CubemarsHardwareInterface::on_configure(
        const rclcpp_lifecycle::State &)
    {

        RCLCPP_INFO(
            rclcpp::get_logger("CubemarsHardwareInterface"), "Configuring interface %s", can_interface_.c_str());

        struct sockaddr_can addr;
        struct ifreq ifr;
        int ret_val;

        CHECK_SC(
            socket(AF_CAN, SOCK_RAW, CAN_RAW),
            "Failed to create CAN socket");
        can_socket_fd_ = ret_val;

        RCLCPP_DEBUG(
            rclcpp::get_logger("CubemarsHardwareInterface"), "Got enable_loopback_: '%i'", enable_loopback_);

        // disable loopback
        // loopback = 0; /* 0 = disabled, 1 = enabled  */
        CHECK_SC(
            setsockopt(can_socket_fd_, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &enable_loopback_, sizeof(enable_loopback_)),
            "Failed to set loopack of CAN socket");

        memset(&ifr, 0, sizeof(ifr));
        strcpy(ifr.ifr_name, can_interface_.c_str());

        CHECK_SC(
            ioctl(can_socket_fd_, SIOCGIFINDEX, &ifr),
            "Failed to find CAN interface index");

        memset(&addr, 0, sizeof(addr));

        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        CHECK_SC(
            bind(can_socket_fd_, (struct sockaddr *)&addr, sizeof(struct sockaddr)),
            "Failed to bind CAN socket");

        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 1000;
        
        CHECK_SC(
            setsockopt(can_socket_fd_, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof(struct timeval)),
            "Failed to set socket option for timeout");

        // CHECK_SC(
        //     fcntl(can_socket_fd_, F_SETFL, O_NONBLOCK),
        //     "Failed to set CAN socket nonblocking");

        for (uint i = 0; i < hw_commands_position_.size(); i++)
        {
            hw_commands_position_[i] = 0;
            hw_commands_velocity_[i] = 0;
            hw_commands_effort_[i] = 0;
            hw_states_position_[i] = 0;
            hw_states_velocity_[i] = 0;
            hw_states_effort_[i] = 0;
            hw_states_temperature_[i] = 0;
        }

        RCLCPP_INFO(rclcpp::get_logger("CubemarsHardwareInterface"), "Successfully configured!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn CubemarsHardwareInterface::on_activate(
        const rclcpp_lifecycle::State &)
    {
        for (auto joint : hw_joint_configs_)
        {
            // if (CubemarsHardwareInterface::set_zero_position(joint.can_id) != hardware_interface::return_type::OK){
            //     RCLCPP_ERROR(rclcpp::get_logger("CubemarsHardwareInterface"), "Failed to set zero position on joint %s (id=%i)!", joint.name.c_str(), joint.can_id);
            //     return hardware_interface::CallbackReturn::ERROR;
            // }

            if (CubemarsHardwareInterface::start_motor_control_mode(joint.can_id) != hardware_interface::return_type::OK)
            {
                RCLCPP_ERROR(rclcpp::get_logger("CubemarsHardwareInterface"), "Failed to start motor control mode on joint %s (id=%i): %s", joint.name.c_str(), joint.can_id, strerror(errno));
                return hardware_interface::CallbackReturn::ERROR;
            }

            RCLCPP_INFO(rclcpp::get_logger("CubemarsHardwareInterface"), "Joint %s (id=%i) started!", joint.name.c_str(), joint.can_id);
        }
        // int ret_val;
        // CHECK_SC(
        //     fcntl(can_socket_fd_, F_SETFL, O_NONBLOCK),
        //     "Failed to set CAN socket nonblocking");

        RCLCPP_INFO(rclcpp::get_logger("CubemarsHardwareInterface"), "Successfully activated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn CubemarsHardwareInterface::on_cleanup(
        const rclcpp_lifecycle::State &)
    {

        RCLCPP_INFO(rclcpp::get_logger("CubemarsHardwareInterface"), "Successfully cleaned!");
        ::close(can_socket_fd_);

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn CubemarsHardwareInterface::on_deactivate(
        const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(
            rclcpp::get_logger("CubemarsHardwareInterface"), "Deactivating ...please wait...");
        for (auto joint : hw_joint_configs_)
        {
            CubemarsHardwareInterface::exit_motor_control_mode(joint.can_id);
            RCLCPP_INFO(rclcpp::get_logger("CubemarsHardwareInterface"), "Joint %s (id=%i) deactivated!", joint.name.c_str(), joint.can_id);
        }
        RCLCPP_INFO(rclcpp::get_logger("CubemarsHardwareInterface"), "Successfully deactivated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type CubemarsHardwareInterface::read(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        // int nbytes;
        // struct can_frame frame;

        // while ((nbytes = ::read(can_socket_fd_, &frame, CAN_MTU)) > 0)
        // {
        //     // struct timeval tv;
        //     // ioctl(s, SIOCGSTAMP, &tv);

        //     int ret_val = CubemarsHardwareInterface::unpack_reply(frame);
        //     if (ret_val != cubemars::ErrorCode::FAULT_CODE_NONE)
        //     {
        //         return hardware_interface::return_type::ERROR;
        //     }
        //     return hardware_interface::return_type::OK;
        // }

        // if (nbytes < 0)
        // {
        //     RCLCPP_WARN(
        //         rclcpp::get_logger("CubemarsHardwareInterface"),
        //         "Could not read from can on interface: '%s': '%s'.", can_interface_.c_str(), strerror(errno));
        //     return hardware_interface::return_type::OK;
        // }
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type CubemarsHardwareInterface::write(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        struct can_frame frame;
        memset(&frame, 0, CAN_MTU);

        // RCLCPP_INFO(
        //     rclcpp::get_logger("CubemarsHardwareInterface"), "Writing...");

        for (uint i = 0; i < hw_joint_configs_.size(); i++)
        {
            memset(&frame, 0, CAN_MTU);
            CubemarsHardwareInterface::pack_cmd(
                &frame,
                hw_commands_position_[i],
                hw_commands_velocity_[i],
                hw_commands_effort_[i],
                hw_joint_configs_[i],
                hw_control_level_[i]);

            if (CubemarsHardwareInterface::write_to_can(frame) != hardware_interface::return_type::OK)
            {
                return hardware_interface::return_type::ERROR;
            }

            memset(&frame, 0, CAN_MTU);
            int nbytes = ::read(can_socket_fd_, &frame, CAN_MTU);

            // struct timeval tv;
            // ioctl(s, SIOCGSTAMP, &tv);

            int ret_val = CubemarsHardwareInterface::unpack_reply(frame);
            if (ret_val != cubemars::ErrorCode::FAULT_CODE_NONE)
            {
                return hardware_interface::return_type::ERROR;
            }

            if (nbytes < 0)
            {
                RCLCPP_WARN(
                    rclcpp::get_logger("CubemarsHardwareInterface"),
                    "Could not read from can on interface: '%s': '%s'.", can_interface_.c_str(), strerror(errno));
                return hardware_interface::return_type::OK;
            }
        }
        return hardware_interface::return_type::OK;
    }

    std::vector<hardware_interface::StateInterface>
    CubemarsHardwareInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (uint i = 0; i < info_.joints.size(); i++)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_states_effort_[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_TEMPERATURE, &hw_states_temperature_[i]));
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface>
    CubemarsHardwareInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (uint i = 0; i < info_.joints.size(); i++)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_position_[i]));
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocity_[i]));
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_commands_effort_[i]));
        }

        return command_interfaces;
    }

    hardware_interface::return_type CubemarsHardwareInterface::prepare_command_mode_switch(
        const std::vector<std::string> &start_interfaces,
        const std::vector<std::string> &stop_interfaces)
    {
        RCLCPP_INFO(
            rclcpp::get_logger("CubemarsHardwareInterface"), "Preparing mode switch...");

        std::vector<cubemars::JointMode> new_modes = {};
        for (std::string key : start_interfaces)
        {
            for (std::size_t i = 0; i < info_.joints.size(); i++)
            {
                if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION)
                {
                    new_modes.push_back(cubemars::JointMode::POSITION);
                }
                if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY)
                {
                    new_modes.push_back(cubemars::JointMode::VELOCITY);
                }
                if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_EFFORT)
                {
                    new_modes.push_back(cubemars::JointMode::EFFORT);
                }
            }
        }

        if (new_modes.size() != info_.joints.size())
        {
            return hardware_interface::return_type::ERROR;
        }

        // Stop motion on all relevant joints that are stopping
        for (std::string key : stop_interfaces)
        {
            for (std::size_t i = 0; i < info_.joints.size(); i++)
            {
                if (key.find(info_.joints[i].name) != std::string::npos)
                {
                    hw_commands_position_[i] = hw_states_position_[i];
                    hw_commands_velocity_[i] = 0;
                    hw_commands_effort_[i] = 0;
                    hw_control_level_[i] = cubemars::JointMode::UNDEFINED; // Revert to undefined
                }
            }
        }
        // Set the new command modes
        for (std::size_t i = 0; i < info_.joints.size(); i++)
        {
            if (hw_control_level_[i] != cubemars::JointMode::UNDEFINED)
            {
                // Something else is using the joint! Abort!
                return hardware_interface::return_type::ERROR;
            }
            hw_control_level_[i] = new_modes[i];
        }
        return hardware_interface::return_type::OK;
    }
    hardware_interface::return_type CubemarsHardwareInterface::write_to_can(can_frame frame)
    {
        if (::write(can_socket_fd_, &frame, sizeof(struct can_frame)) == -1)
        {
            RCLCPP_WARN(
                rclcpp::get_logger("CubemarsHardwareInterface"),
                "Could not write to can on interface: '%s': '%s'.", can_interface_.c_str(), strerror(errno));
            return hardware_interface::return_type::ERROR;
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type CubemarsHardwareInterface::start_motor_control_mode(int can_id)
    {
        struct can_frame frame;
        // frame.can_id = can_id;
        // memcpy(frame.data, cubemars::START_MOTOR_CONTROL_MODE.data(), cubemars::START_MOTOR_CONTROL_MODE.size());
        // frame.len = cubemars::START_MOTOR_CONTROL_MODE.size();

        // std::string data = "\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFC";

        memset(&frame, 0, sizeof(frame));
        frame.can_id = can_id;
        frame.len = 8;
        // frame.flags = 0;
        memcpy(frame.data, cubemars::START_MOTOR_CONTROL_MODE.data(), cubemars::START_MOTOR_CONTROL_MODE.size());
        if (CubemarsHardwareInterface::write_to_can(frame) != hardware_interface::return_type::OK)
        {
            return hardware_interface::return_type::ERROR;
        }

        memset(&frame, 0, sizeof(frame));
        int nbytes = ::read(can_socket_fd_, &frame, CAN_MTU);
        if (nbytes <= 0)
        {
            return hardware_interface::return_type::ERROR;
        }
        if (frame.can_id != can_id)
        {
            return hardware_interface::return_type::ERROR;
        }

        if (CubemarsHardwareInterface::unpack_reply(frame) != cubemars::ErrorCode::FAULT_CODE_NONE)
        {
            return hardware_interface::return_type::ERROR;
        }
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type CubemarsHardwareInterface::exit_motor_control_mode(int can_id)
    {
        struct can_frame frame = {};
        frame.can_id = can_id;
        frame.len = 8;
        memcpy(frame.data, cubemars::EXIT_MOTOR_CONTROL_MODE.data(), cubemars::EXIT_MOTOR_CONTROL_MODE.size());
        // frame.data = cubemars::EXIT_MOTOR_CONTROL_MODE.data();
        frame.len = cubemars::EXIT_MOTOR_CONTROL_MODE.size();
        if (CubemarsHardwareInterface::write_to_can(frame) != hardware_interface::return_type::OK)
        {
            return hardware_interface::return_type::ERROR;
        }

        memset(&frame, 0, sizeof(frame));
        int nbytes = ::read(can_socket_fd_, &frame, CAN_MTU);
        if (nbytes <= 0)
        {
            return hardware_interface::return_type::ERROR;
        }
        if (frame.can_id != can_id)
        {
            return hardware_interface::return_type::ERROR;
        }
        if (CubemarsHardwareInterface::unpack_reply(frame) != cubemars::ErrorCode::FAULT_CODE_NONE)
        {
            return hardware_interface::return_type::ERROR;
        }
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type CubemarsHardwareInterface::set_zero_position(int can_id)
    {

        struct can_frame frame;
        memset(&frame, 0, sizeof(frame));
        frame.can_id = can_id;
        frame.len = 8;

        memcpy(frame.data, cubemars::SET_ZERO_POSITION.data(), cubemars::SET_ZERO_POSITION.size());
        frame.len = cubemars::SET_ZERO_POSITION.size();

        if (CubemarsHardwareInterface::write_to_can(frame) != hardware_interface::return_type::OK)
        {
            return hardware_interface::return_type::ERROR;
        }

        memset(&frame, 0, sizeof(frame));
        int nbytes = ::read(can_socket_fd_, &frame, CAN_MTU);
        if (nbytes <= 0)
        {
            return hardware_interface::return_type::ERROR;
        }
        if (frame.can_id != can_id)
        {
            return hardware_interface::return_type::ERROR;
        }
        if (CubemarsHardwareInterface::unpack_reply(frame) != cubemars::ErrorCode::FAULT_CODE_NONE)
        {
            return hardware_interface::return_type::ERROR;
        }

        return hardware_interface::return_type::OK;
    }

    void CubemarsHardwareInterface::pack_cmd(
        can_frame *frame,
        float p_des,
        float v_des,
        float t_ff,
        cubemars::joint_config_t joint_config,
        cubemars::JointMode control_mode)
    {
        double kp = 0;
        double kd = 0;
        if (control_mode == cubemars::JointMode::POSITION)
        {
            kp = fminf(fmaxf(joint_config.kd_MIN, joint_config.kd), joint_config.kd_MAX);
            kd = fminf(fmaxf(joint_config.kp_MIN, joint_config.kp), joint_config.kp_MAX);
        }
        else if (control_mode == cubemars::JointMode::VELOCITY)
        {
            kd = fminf(fmaxf(joint_config.kp_MIN, joint_config.kp), joint_config.kp_MAX);
        }

        /// limit data to be within bounds ///
        p_des = fminf(fmaxf(joint_config.P_MIN, p_des), joint_config.P_MAX);
        v_des = fminf(fmaxf(joint_config.V_MIN, v_des), joint_config.V_MAX);
        t_ff = fminf(fmaxf(joint_config.I_MIN, t_ff), joint_config.I_MAX);

        /// convert floats to unsigned ints ///
        uint16_t p_int = float_to_uint(p_des, joint_config.P_MIN, joint_config.P_MAX, 16);
        uint16_t v_int = float_to_uint(v_des, joint_config.V_MIN, joint_config.V_MAX, 12);
        uint16_t kp_int = float_to_uint(kp, joint_config.kd_MIN, joint_config.kd_MAX, 12);
        uint16_t kd_int = float_to_uint(kd, joint_config.kp_MIN, joint_config.kp_MAX, 12);
        uint16_t t_int = float_to_uint(t_ff, joint_config.I_MIN, joint_config.I_MAX, 12);

        /// pack ints into the can buffer ///
        frame->data[0] = p_int >> 8;                           // Position High 8
        frame->data[1] = p_int & 0xFF;                         // Position Low 8
        frame->data[2] = v_int >> 4;                           // Speed High 8 bits
        frame->data[3] = ((v_int & 0xF) << 4) | (kp_int >> 8); // Speed Low 4 bits KP High 4 bits
        frame->data[4] = kp_int & 0xFF;                        // KP Low 8 bits
        frame->data[5] = kd_int >> 4;                          // kp High 8 bits
        frame->data[6] = ((kd_int & 0xF) << 4) | (t_int >> 8); // KP Low 4 bits Torque High 4 bits
        frame->data[7] = t_int & 0xff;                         // Torque Low 8 bits

        frame->can_id = joint_config.can_id;
        frame->len = 8;

        // RCLCPP_INFO(rclcpp::get_logger("CubemarsHardwareInterface"), "Writing msg to '%i': Position: '%i (%f)', Velocity: '%i (%f)', Current: '%i (%f)' kd: %i, kp: %i", frame->can_id, p_int, p_des, v_int, v_des, t_int, t_ff, kp_int, kd_int);
    }

    int CubemarsHardwareInterface::float_to_uint(float x, float x_min, float x_max, unsigned int bits)
    {
        /// Converts a float to an unsigned int, given range and number of bits ///
        float span = x_max - x_min;
        if (x < x_min)
            x = x_min;
        else if (x > x_max)
            x = x_max;

        return (int)((x - x_min) * ((float)((1 << bits) / span)));
    }

    cubemars::ErrorCode CubemarsHardwareInterface::unpack_reply(struct can_frame frame)
    {
        cubemars::ErrorCode error_code = cubemars::ErrorCode::FAULT_CODE_NONE;
        auto it = std::find_if(
            hw_joint_configs_.begin(),
            hw_joint_configs_.end(),
            [&](const auto &conf)
            { return conf.can_id == frame.can_id; });

        if (it != hw_joint_configs_.end())
        {
            int index = it - hw_joint_configs_.begin();
            uint8_t id = frame.data[0];                                    // Driver ID
            uint16_t p_int = (frame.data[1] << 8) | frame.data[2];         // Motor Position Data
            uint16_t v_int = (frame.data[3] << 4) | (frame.data[4] >> 4);  // Motor Speed Data
            uint16_t i_int = ((frame.data[4] & 0xF) << 8) | frame.data[5]; // Motor Torque Data
            uint8_t temp_int = frame.data[6];
            error_code = static_cast<cubemars::ErrorCode>(frame.data[7]);

            /// convert ints to floats ///
            float p = uint_to_float(p_int, it->P_MIN, it->P_MAX, 16);
            float v = uint_to_float(v_int, it->V_MIN, it->V_MAX, 12);
            float i = uint_to_float(i_int, -it->I_MIN, it->I_MIN, 12);
            float temp = temp_int - 40;
            float temp_celsius = (temp - 32) * 5 / 9 - 40;

            hw_states_position_[index] = p;
            hw_states_velocity_[index] = v;
            hw_states_effort_[index] = i;
            hw_states_temperature_[index] = temp;

            // RCLCPP_INFO(rclcpp::get_logger("CubemarsHardwareInterface"), "Got msg from '%i' ('%i'): Position: '%f' ('%i'), Velocity: '%f' ('%i'), Current: '%f' ('%i'), Temperature: '%f'", id, frame.can_id, p, p_int, v, v_int, i, i_int, temp);
            if (error_code != cubemars::ErrorCode::FAULT_CODE_NONE)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("CubemarsHardwareInterface"),
                    "Joint '%s': %s.", it->name.c_str(), cubemars::errorFlagToString(error_code));
            }
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("CubemarsHardwareInterface"), "Device with can id %i (%i) unknwon", frame.can_id, frame.data[0]);
        }

        return error_code;

        // float P_MIN = -12.5f;
        // float P_MAX = 12.5f;
        // float V_MIN = -30.0f;
        // float V_MAX = 30.0f;
        // float T_MIN = -18.0f;
        // float T_MAX = 18.0f;
        // float kd_MIN = 0;
        // float kd_MAX = 500.0f;
        // float kd_MIN = 0;
        // float kd_MAX = 5.0f;

        // float I_MIN = 0.0f;
        // float I_MAX = 5.0f;

        /// unpack ints from can buffer ///

        // if(id == 1){
        //   postion = p; // Read corresponding data based on ID
        //   speed = v;
        //   torque = i;
        //   Temperature = T-40; // Temperature range: -40~215
        // }
    }

    float CubemarsHardwareInterface::uint_to_float(int x_int, float x_min, float x_max, int bits)
    {
        /// converts unsigned int to float, given range and number of bits ///
        float span = x_max - x_min;
        float offset = x_min;
        return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
    }

} // namespace

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    cubemars_hardware_interface::CubemarsHardwareInterface, hardware_interface::SystemInterface)
