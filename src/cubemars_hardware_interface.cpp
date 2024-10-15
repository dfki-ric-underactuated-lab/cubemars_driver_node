#include "cubemars_hardware_interface/cubemars_hardware_interface.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace hw = hardware_interface;

namespace cubemars_hardware_interface
{
    // Ensure on_deactivate is called and joints are deactivated on SIGINT
    // https://github.com/ros-controls/ros2_control/issues/472
    CubemarsHardwareInterface::~CubemarsHardwareInterface()
    {
        on_deactivate(rclcpp_lifecycle::State());
    }

    hw::CallbackReturn CubemarsHardwareInterface::on_init(
        const hw::HardwareInfo &info)
    {
        if (
            hw::SystemInterface::on_init(info) !=
            hw::CallbackReturn::SUCCESS)
        {
            return hw::CallbackReturn::ERROR;
        }

        if (info_.hardware_parameters["can_interface"].empty())
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("CubemarsHardwareInterface"), 
                "Parameter to select CAN interface empty!");
            return hw::CallbackReturn::ERROR;
        }
        can_interface_ = info_.hardware_parameters["can_interface"];

        enable_loopback_ = info_.hardware_parameters["enable_loopback"] == "True" 
            ||info_.hardware_parameters["enable_loopback"] == "true" 
            ||info_.hardware_parameters["enable_loopback"] == "1";

        set_zero_pos_on_startup_ = info_.hardware_parameters["set_zero_pos_on_startup"] == "True" 
            ||info_.hardware_parameters["set_zero_pos_on_startup"] == "true" 
            ||info_.hardware_parameters["set_zero_pos_on_startup"] == "1";

        // set_zero_pos_on_startup_ = true;

        hw_joint_configs_.clear();
        for (auto joint : info_.joints)
        {
            cubemars::joint_config_t conf;
            conf.name = joint.name;
            conf.can_id = stoi(joint.parameters["can_id"]);
            conf.KD = stod(joint.parameters["kd"]);
            conf.KP = stod(joint.parameters["kp"]);
            conf.KP_MIN = stod(joint.parameters["kd_min"]);
            conf.KD_MAX = stod(joint.parameters["kd_max"]);
            conf.KP_MIN = stod(joint.parameters["kp_min"]);
            conf.KP_MAX = stod(joint.parameters["kp_max"]);

            conf.P_MIN = stod(joint.parameters["pos_min"]);
            conf.P_MAX = stod(joint.parameters["pos_max"]);
            conf.V_MIN = stod(joint.parameters["vel_min"]);
            conf.V_MAX = stod(joint.parameters["vel_max"]);
            conf.I_MIN = stod(joint.parameters["effort_min"]);
            conf.I_MAX = stod(joint.parameters["effort_max"]);

            conf.invert = joint.parameters["invert"] == "True"
              || joint.parameters["invert"] == "true"
              || joint.parameters["invert"] == "1";

            for (auto command_interface : joint.command_interfaces)
            {
                if (command_interface.name == hw::HW_IF_POSITION)
                {
                    conf.POSITION_COMMAND_SOFT_LIMIT_MIN = stod(command_interface.min);
                    conf.POSITION_COMMAND_SOFT_LIMIT_MAX = stod(command_interface.max);
                }
                else if (command_interface.name == hw::HW_IF_VELOCITY)
                {
                    conf.VELOCITY_COMMAND_SOFT_LIMIT_MIN = stod(command_interface.min);
                    conf.VELOCITY_COMMAND_SOFT_LIMIT_MAX = stod(command_interface.max);
                }
                else if (command_interface.name == hw::HW_IF_EFFORT)
                {
                    conf.EFFORT_COMMAND_SOFT_LIMIT_MIN = stod(command_interface.min);
                    conf.EFFORT_COMMAND_SOFT_LIMIT_MAX = stod(command_interface.max);
                }
            }

            hw_joint_configs_.push_back(conf);
            RCLCPP_INFO(
                rclcpp::get_logger("CubemarsHardwareInterface"), 
                "Using joint %s (id=%i)", conf.name.c_str(), conf.can_id);
        }

        // Create a sorted vector in descending order of indices to start 
        // communication with joint with lowest priority (highest CAN id).
        sorted_idx_.resize(hw_joint_configs_.size());
        iota(sorted_idx_.begin(), sorted_idx_.end(), 0);

        std::sort(sorted_idx_.begin(), sorted_idx_.end(),
                  [&](size_t i1, size_t i2)
                  {
                      return hw_joint_configs_[i1].can_id > hw_joint_configs_[i2].can_id;
                  });

        hw_commands_position_.resize(
            info_.joints.size(), 
            std::numeric_limits<double>::quiet_NaN());
        hw_commands_velocity_.resize(
            info_.joints.size(), 
            std::numeric_limits<double>::quiet_NaN());
        hw_commands_effort_.resize(
            info_.joints.size(), 
            std::numeric_limits<double>::quiet_NaN());
        hw_states_position_.resize(
            info_.joints.size(), 
            std::numeric_limits<double>::quiet_NaN());
        hw_states_velocity_.resize(
            info_.joints.size(), 
            std::numeric_limits<double>::quiet_NaN());
        hw_states_effort_.resize(
            info_.joints.size(), 
            std::numeric_limits<double>::quiet_NaN());
        hw_states_temperature_.resize(
            info_.joints.size(), 
            std::numeric_limits<double>::quiet_NaN());
        hw_control_level_.resize(
            info_.joints.size(), 
            cubemars::JointMode::UNDEFINED);

        return hw::CallbackReturn::SUCCESS;
    }

    hw::CallbackReturn CubemarsHardwareInterface::setup_socket()
    {
        RCLCPP_INFO(
            rclcpp::get_logger("CubemarsHardwareInterface"), 
            "Configuring interface %s", can_interface_.c_str());

        struct sockaddr_can addr;
        struct ifreq ifr;
        int ret_val;

        CHECK_SC(
            socket(AF_CAN, SOCK_RAW, CAN_RAW),
            "Failed to create CAN socket");
        can_socket_fd_ = ret_val;

        RCLCPP_DEBUG(
            rclcpp::get_logger("CubemarsHardwareInterface"), 
            "Got enable_loopback_: '%i'", enable_loopback_);

        // disable loopback
        // loopback = 0; /* 0 = disabled, 1 = enabled  */
        CHECK_SC(
            setsockopt(can_socket_fd_, 
                       SOL_CAN_RAW, 
                       CAN_RAW_LOOPBACK, 
                       &enable_loopback_, 
                       sizeof(enable_loopback_)),
            "Failed to set loopack of CAN socket");


        //only receive CAN messages from specified joints
        // struct can_filter rfilter[hw_joint_configs_.size()];
        std::vector<can_filter> rfilter;
        rfilter.resize(hw_joint_configs_.size());

        for (uint i = 0; i < hw_joint_configs_.size(); i++)
        {
            rfilter[i].can_id   = hw_joint_configs_[i].can_id;
            rfilter[i].can_mask = CAN_SFF_MASK;
        }
        CHECK_SC(
            setsockopt(can_socket_fd_, 
                       SOL_CAN_RAW, 
                       CAN_RAW_FILTER, 
                       rfilter.data(), 
                       rfilter.size() * sizeof(can_filter)),
            "Failed to set CAN filter");

        memset(&ifr, 0, sizeof(ifr));
        strcpy(ifr.ifr_name, can_interface_.c_str());

        CHECK_SC(
            ioctl(can_socket_fd_, SIOCGIFINDEX, &ifr),
            "Failed to find CAN interface index");

        memset(&addr, 0, sizeof(addr));

        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        CHECK_SC(
            bind(can_socket_fd_, 
                 (struct sockaddr *)&addr, 
                 sizeof(struct sockaddr)),
            "Failed to bind CAN socket");

        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 500000;

        CHECK_SC(
            setsockopt(can_socket_fd_, 
                       SOL_SOCKET, 
                       SO_RCVTIMEO, 
                       (const char *)&tv, 
                       sizeof(struct timeval)),
            "Failed to set socket option for timeout");

        // CHECK_SC(
        //     fcntl(can_socket_fd_, F_SETFL, O_NONBLOCK),
        //     "Failed to set CAN socket nonblocking");

        return hw::CallbackReturn::SUCCESS;
    }


    hw::CallbackReturn CubemarsHardwareInterface::on_configure(
        const rclcpp_lifecycle::State &)
    {
        hw::CallbackReturn ret_val;
        if ((ret_val = setup_socket()) != hw::CallbackReturn::SUCCESS){
            return ret_val;
        }


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

        RCLCPP_INFO(rclcpp::get_logger("CubemarsHardwareInterface"), 
                    "Successfully configured!");

        return hw::CallbackReturn::SUCCESS;
    }

    hw::CallbackReturn CubemarsHardwareInterface::on_activate(
        const rclcpp_lifecycle::State &)
    {
        for (auto joint : hw_joint_configs_)
        {
            if (set_zero_pos_on_startup_)
            {
                if (set_zero_position(joint.can_id) != hw::return_type::OK){
                    RCLCPP_ERROR(rclcpp::get_logger("CubemarsHardwareInterface"), 
                                    "Failed to set zero position on joint %s (id=%i)!", 
                                    joint.name.c_str(), 
                                    joint.can_id);
                    return hw::CallbackReturn::ERROR;
                }
            }

            if (start_motor_control_mode(joint.can_id) != hw::return_type::OK)
            {
                RCLCPP_ERROR(rclcpp::get_logger("CubemarsHardwareInterface"), 
                    "Failed to start motor control mode on joint %s (id=%i): %s", 
                    joint.name.c_str(), joint.can_id, strerror(errno));
                return hw::CallbackReturn::ERROR;
            }

            RCLCPP_INFO(rclcpp::get_logger("CubemarsHardwareInterface"), 
                        "Joint %s (id=%i) activated!", 
                        joint.name.c_str(), 
                        joint.can_id);
        }

        RCLCPP_INFO(rclcpp::get_logger("CubemarsHardwareInterface"), 
                    "All joints successfully activated!");

        return hw::CallbackReturn::SUCCESS;
    }

    hw::CallbackReturn CubemarsHardwareInterface::on_cleanup(
        const rclcpp_lifecycle::State &)
    {

        RCLCPP_INFO(rclcpp::get_logger("CubemarsHardwareInterface"), 
                    "Successfully cleaned!");

        ::close(can_socket_fd_);

        return hw::CallbackReturn::SUCCESS;
    }

    hw::CallbackReturn CubemarsHardwareInterface::on_deactivate(
        const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(
            rclcpp::get_logger("CubemarsHardwareInterface"), 
            "Deactivating joints...");

        hw::CallbackReturn ret_val = hw::CallbackReturn::SUCCESS;
        for (auto joint : hw_joint_configs_)
        {
            if (exit_motor_control_mode(joint.can_id) == hw::return_type::OK)
            {
                RCLCPP_INFO(rclcpp::get_logger("CubemarsHardwareInterface"), 
                            "Joint %s (id=%i) deactivated!", 
                            joint.name.c_str(), joint.can_id);
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("CubemarsHardwareInterface"), 
                             "Failed to deactivate Joint %s (id=%i)!", 
                             joint.name.c_str(), joint.can_id);

                ret_val = hw::CallbackReturn::ERROR;
            }
        }
        if (ret_val == hw::CallbackReturn::SUCCESS)
        {
            RCLCPP_INFO(rclcpp::get_logger("CubemarsHardwareInterface"), "All joints successfully deactivated!");
        }

        return ret_val;
    }

    hw::return_type CubemarsHardwareInterface::read(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        // we are reading in write to ensure alternating writes/reads
        // with no CAN id collisions on the bus (cubemars joints use the same CAN id 
        // for the response frame)

        return hw::return_type::OK;
    }

    hw::return_type CubemarsHardwareInterface::write(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        struct can_frame frame;

        for (uint i : sorted_idx_)
        {
            memset(&frame, 0, CAN_MTU);
            CubemarsHardwareInterface::pack_cmd(
                &frame,
                hw_commands_position_[i],
                hw_commands_velocity_[i],
                hw_commands_effort_[i],
                hw_joint_configs_[i],
                hw_control_level_[i]);

            if (CubemarsHardwareInterface::write_to_can(frame) != hw::return_type::OK)
            {
                return hw::return_type::ERROR;
            }
        }

        for ([[maybe_unused]] uint i : sorted_idx_)
        {
            memset(&frame, 0, CAN_MTU);

            int nbytes = ::read(can_socket_fd_, &frame, CAN_MTU);

            if (nbytes < 0)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("CubemarsHardwareInterface"),
                    "Could not read from can on interface: '%s': '%s'", 
                    can_interface_.c_str(), strerror(errno));
                return hw::return_type::ERROR;
            }

            if (frame.can_id == 0)
            {
                // TODO more sophisticated error handling here
                return hw::return_type::ERROR;
            }

            int ret_val = CubemarsHardwareInterface::unpack_reply(frame);
            if (ret_val != cubemars::ErrorCode::FAULT_CODE_NONE)
            {
                return hw::return_type::ERROR;
            }
        }
        return hw::return_type::OK;
    }

    std::vector<hw::StateInterface>
    CubemarsHardwareInterface::export_state_interfaces()
    {
        std::vector<hw::StateInterface> state_interfaces;
        for (uint i = 0; i < info_.joints.size(); i++)
        {
            state_interfaces.emplace_back(hw::StateInterface(
                info_.joints[i].name, 
                hw::HW_IF_POSITION, 
                &hw_states_position_[i]));
            state_interfaces.emplace_back(hw::StateInterface(
                info_.joints[i].name, 
                hw::HW_IF_VELOCITY, 
                &hw_states_velocity_[i]));
            state_interfaces.emplace_back(hw::StateInterface(
                info_.joints[i].name, 
                hw::HW_IF_EFFORT, 
                &hw_states_effort_[i]));
            state_interfaces.emplace_back(hw::StateInterface(
                info_.joints[i].name, 
                hw::HW_IF_TEMPERATURE, 
                &hw_states_temperature_[i]));
        }

        return state_interfaces;
    }

    std::vector<hw::CommandInterface>
    CubemarsHardwareInterface::export_command_interfaces()
    {
        std::vector<hw::CommandInterface> command_interfaces;
        for (uint i = 0; i < info_.joints.size(); i++)
        {
            command_interfaces.emplace_back(hw::CommandInterface(
                info_.joints[i].name, 
                hw::HW_IF_POSITION, 
                &hw_commands_position_[i]));
            command_interfaces.emplace_back(hw::CommandInterface(
                info_.joints[i].name, 
                hw::HW_IF_VELOCITY, 
                &hw_commands_velocity_[i]));
            command_interfaces.emplace_back(hw::CommandInterface(
                info_.joints[i].name, 
                hw::HW_IF_EFFORT, 
                &hw_commands_effort_[i]));
        }

        return command_interfaces;
    }

    hw::return_type CubemarsHardwareInterface::prepare_command_mode_switch(
        const std::vector<std::string> &start_interfaces,
        const std::vector<std::string> &stop_interfaces)
    {
        RCLCPP_INFO(
            rclcpp::get_logger("CubemarsHardwareInterface"), 
            "Preparing mode switch...");

        std::vector<cubemars::JointMode> new_modes = {};
        for (std::string key : start_interfaces)
        {
            for (std::size_t i = 0; i < info_.joints.size(); i++)
            {
                if (key == info_.joints[i].name + "/" + hw::HW_IF_POSITION)
                {
                    new_modes.push_back(cubemars::JointMode::POSITION);
                }
                if (key == info_.joints[i].name + "/" + hw::HW_IF_VELOCITY)
                {
                    new_modes.push_back(cubemars::JointMode::VELOCITY);
                }
                if (key == info_.joints[i].name + "/" + hw::HW_IF_EFFORT)
                {
                    new_modes.push_back(cubemars::JointMode::EFFORT);
                }
            }
        }

        if (new_modes.size() != info_.joints.size())
        {
            return hw::return_type::ERROR;
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
                    hw_control_level_[i] = cubemars::JointMode::UNDEFINED;
                }
            }
        }
        // Set the new command modes
        for (std::size_t i = 0; i < info_.joints.size(); i++)
        {
            if (hw_control_level_[i] != cubemars::JointMode::UNDEFINED)
            {
                // Something else is using the joint! Abort!
                return hw::return_type::ERROR;
            }
            hw_control_level_[i] = new_modes[i];
        }
        return hw::return_type::OK;
    }

    hw::return_type CubemarsHardwareInterface::write_to_can(can_frame frame)
    {
        if (::write(can_socket_fd_, &frame, sizeof(struct can_frame)) == -1)
        {
            RCLCPP_WARN(
                rclcpp::get_logger("CubemarsHardwareInterface"),
                "Could not write to can on interface: '%s': '%s'.", 
                can_interface_.c_str(), strerror(errno));
            return hw::return_type::ERROR;
        }

        return hw::return_type::OK;
    }

    hw::return_type CubemarsHardwareInterface::send_control_frame(
        canid_t can_id, std::array<uint8_t, 8> control_sequence)
    {
        struct can_frame frame;

        memset(&frame, 0, sizeof(frame));
        frame.can_id = can_id;
        std::copy(control_sequence.begin(), control_sequence.end(), frame.data);
        frame.len = control_sequence.size();
        // memcpy(frame.data, cubemars::START_MOTOR_CONTROL_MODE.data(), cubemars::START_MOTOR_CONTROL_MODE.size());
        if (CubemarsHardwareInterface::write_to_can(frame) != hw::return_type::OK)
        {
            return hw::return_type::ERROR;
        }

        memset(&frame, 0, sizeof(frame));
        int nbytes = ::read(can_socket_fd_, &frame, CAN_MTU);
        if (nbytes <= 0)
        {
            return hw::return_type::ERROR;
        }
        if (frame.can_id != can_id)
        {
            return hw::return_type::ERROR;
        }

        if (CubemarsHardwareInterface::unpack_reply(frame) != cubemars::ErrorCode::FAULT_CODE_NONE)
        {
            return hw::return_type::ERROR;
        }
        return hw::return_type::OK;
    }
    

    hw::return_type CubemarsHardwareInterface::start_motor_control_mode(
        canid_t can_id)
    {
        return send_control_frame(can_id, cubemars::START_MOTOR_CONTROL_MODE);
    }

    hw::return_type CubemarsHardwareInterface::exit_motor_control_mode(canid_t can_id)
    {
        return send_control_frame(can_id, cubemars::EXIT_MOTOR_CONTROL_MODE);
    }

    hw::return_type CubemarsHardwareInterface::set_zero_position(canid_t can_id)
    {
        return send_control_frame(can_id, cubemars::SET_ZERO_POSITION);
    }

    void CubemarsHardwareInterface::pack_cmd(
        can_frame *frame,
        float p_des,
        float v_des,
        float t_ff,
        cubemars::joint_config_t joint_config,
        cubemars::JointMode control_mode)
    {
        if (joint_config.invert){
            p_des *= -1;
            v_des *= -1;
            t_ff *= -1;
        }

        double kp = 0;
        double kd = 0;
        if (control_mode == cubemars::JointMode::POSITION)
        {
            kp = fminf(
                    fmaxf(joint_config.KP_MIN, joint_config.KP), 
                    joint_config.KP_MAX);

            kd = fminf(
                    fmaxf(joint_config.KD_MIN, joint_config.KD), 
                    joint_config.KD_MAX);
        }
        else if (control_mode == cubemars::JointMode::VELOCITY)
        {
            kd = fminf(
                    fmaxf(joint_config.KD_MIN, joint_config.KD), 
                    joint_config.KD_MAX);
        }

        /// limit data to be within bounds ///
        p_des = fminf(
                    fmaxf(joint_config.P_MIN, p_des), 
                    joint_config.P_MAX);
        v_des = fminf(
                    fmaxf(joint_config.V_MIN, v_des), 
                    joint_config.V_MAX);
        t_ff = fminf(
                    fmaxf(joint_config.I_MIN, t_ff), 
                    joint_config.I_MAX);

        /// limit data to be within soft limits ///
        p_des = std::clamp(
            p_des, 
            joint_config.POSITION_COMMAND_SOFT_LIMIT_MIN,
            joint_config.POSITION_COMMAND_SOFT_LIMIT_MAX);

        v_des = std::clamp(
            v_des, 
            joint_config.VELOCITY_COMMAND_SOFT_LIMIT_MIN,
            joint_config.VELOCITY_COMMAND_SOFT_LIMIT_MAX);

        t_ff = std::clamp(
            t_ff, 
            joint_config.EFFORT_COMMAND_SOFT_LIMIT_MIN,
            joint_config.EFFORT_COMMAND_SOFT_LIMIT_MAX);

        /// convert floats to unsigned ints ///
        uint16_t p_int = float_to_uint(
            p_des, 
            joint_config.P_MIN, 
            joint_config.P_MAX, 
            16);
        uint16_t v_int = float_to_uint(
            v_des, 
            joint_config.V_MIN, 
            joint_config.V_MAX, 
            12);
        uint16_t kp_int = float_to_uint(
            kp, 
            joint_config.KP_MIN, 
            joint_config.KP_MAX, 
            12);
        uint16_t kd_int = float_to_uint(
            kd, 
            joint_config.KD_MIN, 
            joint_config.KD_MAX, 
            12);
        uint16_t t_int = float_to_uint(
            t_ff, 
            joint_config.I_MIN, 
            joint_config.I_MAX, 
            12);

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

    }

    int CubemarsHardwareInterface::float_to_uint(
        float x, float x_min, float x_max, unsigned int bits)
    {
        /// Converts a float to an unsigned int, given range and number of bits ///
        float span = x_max - x_min;
        if (x < x_min)
            x = x_min;
        else if (x > x_max)
            x = x_max;

        return (int)((x - x_min) * ((float)((1 << bits) / span)));
    }

    cubemars::ErrorCode CubemarsHardwareInterface::unpack_reply(
        struct can_frame frame)
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
            // canid_t id = frame.data[0];                                    // Driver ID
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

            if (it->invert){
                p *= -1;
                v_int *= -1;
                i *= -1;
            }

            hw_states_position_[index] = p;
            hw_states_velocity_[index] = v;
            hw_states_effort_[index] = i;
            hw_states_temperature_[index] = temp;

            if (error_code != cubemars::ErrorCode::FAULT_CODE_NONE)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("CubemarsHardwareInterface"),
                    "Joint '%s': %s.", it->name.c_str(), cubemars::errorFlagToString(error_code));
            }
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("CubemarsHardwareInterface"), 
                        "Device with can id %i (%i) unknown", frame.can_id, frame.data[0]);
        }

        return error_code;
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
    cubemars_hardware_interface::CubemarsHardwareInterface, hw::SystemInterface)
