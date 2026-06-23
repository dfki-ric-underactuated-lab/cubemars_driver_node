#include "cubemars_hardware_interface/cubemars_hardware_node.hpp"

CubeMarsHardwareNode::CubeMarsHardwareNode() : rclcpp_lifecycle::LifecycleNode("cubemars_hardware_node")
{
}

LifecycleNodeInterface::CallbackReturn CubeMarsHardwareNode::on_configure([[maybe_unused]] const rclcpp_lifecycle::State &previous_state)
{
    /**Declare and read parameters */
    this->declare_parameter_if_undeclared("joints", rclcpp::PARAMETER_STRING_ARRAY);
    this->declare_parameter_if_undeclared("default_damping_KD", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter_if_undeclared("enable_loopback", true);
    this->declare_parameter_if_undeclared("can_socket_timeout_usec", 1000);
    this->declare_parameter_if_undeclared("can_socket_timeout_sec", 0);
    this->declare_parameter_if_undeclared("frequency", 1000);
    this->declare_parameter_if_undeclared("watchdog_frequency", 100);
    this->declare_parameter_if_undeclared("friction_compensation_sign_steepness", 100.);
    this->declare_parameter_if_undeclared("publish_ros2_joint_state", false);
    this->declare_parameter_if_undeclared("damping_on_motor_error", true);
    this->declare_parameter_if_undeclared("max_can_errors_before_motor_shutdown", 1);
    this->declare_parameter_if_undeclared("can_initial_connection_trials", 10);
    this->declare_parameter_if_undeclared("enable_tx_timestamping", true);

    std::set<std::string> can_interfaces_names_;
    std::unordered_map<std::string, std::set<int>> can_id_per_interface;
    std::set<int> msg_idxs;
    std::set<std::string> motor_types;
    std::vector<std::vector<cubemars::joint_config_t>> joint_configs_per_can_interface;
    try
    {
        damping_on_motor_error_ =this->get_parameter("damping_on_motor_error").as_bool();
        max_can_errors_before_motor_shutdown_ = this->get_parameter("max_can_errors_before_motor_shutdown").as_int();
        enable_tx_timestamping_ = this->get_parameter("enable_tx_timestamping").as_bool();

        auto joint_names = this->get_parameter("joints").as_string_array();
        default_damping_KD_ = this->get_parameter("default_damping_KD").as_double();
        frequency_ = std::chrono::duration<double>(1.0 / this->get_parameter("frequency").as_int());
        watchdog_frequency_ = std::chrono::duration<double>(1.0 / this->get_parameter("watchdog_frequency").as_int());
        publish_ros2_joint_state_ = this->get_parameter("publish_ros2_joint_state").as_bool();

        // Publishers //TODO: add option to disabke 'debug topics'
        joint_state_pub_ = this->create_publisher<robot_control_msgs::msg::JointState>("~/joint_states", QOS_BEST_EFFORT_NO_DEPTH);
        joint_temp_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("joint_temperatures", QOS_BEST_EFFORT_NO_DEPTH);
        can_interface_frequency_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("can_cycle_frequencies", QOS_BEST_EFFORT_NO_DEPTH);
        can_tx_fill_duration_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("can_tx_fill_durations_us", QOS_BEST_EFFORT_NO_DEPTH);
        can_rx_duration_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("can_rx_durations_us", QOS_BEST_EFFORT_NO_DEPTH);
        joint_reply_after_tx_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("joint_reply_after_tx_us", QOS_BEST_EFFORT_NO_DEPTH);
        joint_state_age_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("joint_state_ages_us", QOS_BEST_EFFORT_NO_DEPTH);
        if (enable_tx_timestamping_)
        {
            // These two depend on the software TX timestamp; only published when it's enabled.
            joint_cmd_to_bus_latency_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("joint_cmd_to_bus_latencies_us", QOS_BEST_EFFORT_NO_DEPTH);
            joint_motor_reply_latency_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("joint_motor_reply_latencies_us", QOS_BEST_EFFORT_NO_DEPTH);
        }
        unfiltered_velocity_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("joint_velocities_unfiltered", QOS_BEST_EFFORT_NO_DEPTH);
        unfiltered_position_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("joint_positions_unfiltered", QOS_BEST_EFFORT_NO_DEPTH);
        controller_latency_pub_ = this->create_publisher<std_msgs::msg::Float32>("controller_latency_us", QOS_BEST_EFFORT_NO_DEPTH);
        if (publish_ros2_joint_state_)
        {
            ros2_joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("~/ros2_joint_state", QOS_BEST_EFFORT_NO_DEPTH);
        }

        // For each joint create default parameters and  validat them them
        unsigned int max_msg_idx = 0;
        for (unsigned int i = 0; i < joint_names.size(); i++)
        {
            // Declare joint definitions
            this->declare_parameter_if_undeclared("joint_defintions." + joint_names[i] + ".msg_idx", static_cast<int>(i));
            this->declare_parameter_if_undeclared("joint_defintions." + joint_names[i] + ".can_interface", rclcpp::PARAMETER_STRING);
            this->declare_parameter_if_undeclared("joint_defintions." + joint_names[i] + ".can_id", rclcpp::PARAMETER_INTEGER);
            this->declare_parameter_if_undeclared("joint_defintions." + joint_names[i] + ".motor_type", rclcpp::PARAMETER_STRING);
            this->declare_parameter_if_undeclared("joint_defintions." + joint_names[i] + ".invert", false);
            this->declare_parameter_if_undeclared("joint_defintions." + joint_names[i] + ".tau_c", 0.0);
            this->declare_parameter_if_undeclared("joint_defintions." + joint_names[i] + ".tau_s", 0.0);
            this->declare_parameter_if_undeclared("joint_defintions." + joint_names[i] + ".v_s", 1.0);
            this->declare_parameter_if_undeclared("joint_defintions." + joint_names[i] + ".k", 1.0);
            this->declare_parameter_if_undeclared("joint_defintions." + joint_names[i] + ".k_a", 1.0);
            this->declare_parameter_if_undeclared("joint_defintions." + joint_names[i] + ".b", 0.0);
            this->declare_parameter_if_undeclared("joint_defintions." + joint_names[i] + ".transmission_ratio", 1.0);
            this->declare_parameter_if_undeclared("joint_defintions." + joint_names[i] + ".set_zero_position_on_configure", false);
            this->declare_parameter_if_undeclared("joint_defintions." + joint_names[i] + ".zero_position", 0.0);
            this->declare_parameter_if_undeclared("joint_defintions." + joint_names[i] + ".pos_limit_min", std::numeric_limits<double>::lowest());
            this->declare_parameter_if_undeclared("joint_defintions." + joint_names[i] + ".pos_limit_max", std::numeric_limits<double>::max());
            this->declare_parameter_if_undeclared("joint_defintions." + joint_names[i] + ".vel_filter_size", 0);
            this->declare_parameter_if_undeclared("joint_defintions." + joint_names[i] + ".vel_filter_type", std::string("moving_average"));
            this->declare_parameter_if_undeclared("joint_defintions." + joint_names[i] + ".alpha", 0.85);
            this->declare_parameter_if_undeclared("joint_defintions." + joint_names[i] + ".beta", 0.005);
            this->declare_parameter_if_undeclared("joint_defintions." + joint_names[i] + ".pos_median_filter_size", 0);

            // Validate joint defintions
            auto can_interface_name = this->get_parameter("joint_defintions." + joint_names[i] + ".can_interface").as_string();
            can_interfaces_names_.insert(can_interface_name);
            auto msg_idx = this->get_parameter("joint_defintions." + joint_names[i] + ".msg_idx").as_int();
            if (!msg_idxs.insert(msg_idx).second)
            {
                RCLCPP_ERROR(this->get_logger(), "Joint %s has msg_idx %li which is already used by another joint", joint_names[i].c_str(), msg_idx);
                return LifecycleNodeInterface::CallbackReturn::FAILURE;
            }
            if (msg_idx < 0)
            {
                RCLCPP_ERROR(this->get_logger(), "Joint %s has negative msg_idx %li", joint_names[i].c_str(), msg_idx);
                return LifecycleNodeInterface::CallbackReturn::FAILURE;
            }
            if (msg_idx > max_msg_idx)
            {
                max_msg_idx = msg_idx;
            }
            auto can_id = this->get_parameter("joint_defintions." + joint_names[i] + ".can_id").as_int();
            if (!can_id_per_interface[can_interface_name].insert(can_id).second)
            {
                RCLCPP_ERROR(this->get_logger(), "Joint %s has can_id %li which is already used can interface %s", joint_names[i].c_str(), can_id, can_interface_name.c_str());
                return LifecycleNodeInterface::CallbackReturn::FAILURE;
            }
            auto motor_type = this->get_parameter("joint_defintions." + joint_names[i] + ".motor_type").as_string();
            motor_types.insert(motor_type);
            auto transmission_ratio = this->get_parameter("joint_defintions." + joint_names[i] + ".transmission_ratio").as_double();
            if (transmission_ratio <= 0)
            {
                RCLCPP_ERROR(this->get_logger(), "Transmissions should be > 0, but joint %s has transmission ratio %f", joint_names[i].c_str(), transmission_ratio);
                return LifecycleNodeInterface::CallbackReturn::FAILURE;
            }
        }
        joint_msg_length_ = max_msg_idx + 1;
        num_joints_ = joint_names.size();
        // Create joint configs per can interfaces and create state and message mapping
        unsigned int num_can_interfaces = can_interfaces_names_.size();
        joint_configs_per_can_interface.resize(num_can_interfaces);
        joint_commands_per_can_interface_.resize(num_can_interfaces);
        joint_states_per_can_interface_.resize(num_can_interfaces);
        joint_parameters_per_can_interface_.resize(num_can_interfaces);
        joint_vel_filters_per_can_interface_.resize(num_can_interfaces);
        joint_ab_filters_per_can_interface_.resize(num_can_interfaces);
        joint_pos_median_filters_per_can_interface_.resize(num_can_interfaces);
        last_joint_rx_ns_per_can_interface_.resize(num_can_interfaces);
        num_can_errors_per_interfaces_.resize(num_can_interfaces, 0);
        can_interfaces_.resize(num_can_interfaces);
        last_can_cycle_times_.resize(num_can_interfaces, this->get_clock()->now());
        // One comm thread object per interface (threads spawned later, after motors are enabled).
        comm_threads_.resize(num_can_interfaces);
        for (unsigned int i = 0; i < num_can_interfaces; i++)
        {
            comm_threads_[i] = std::make_unique<CommThread>();
        }
        can_interface_frequency_msg_.data.resize(num_can_interfaces, 0.0);
        can_tx_fill_duration_msg_.data.resize(num_can_interfaces, std::nanf(""));
        can_rx_duration_msg_.data.resize(num_can_interfaces, std::nanf(""));

        // Prepare messages
        // Initial command: all zero (free / no torque) until the first real command arrives.
        auto init_cmd = std::make_shared<robot_control_msgs::msg::JointCommand>();
        init_cmd->effort.assign(max_msg_idx + 1, 0.0);
        init_cmd->velocity.assign(max_msg_idx + 1, 0.0);
        init_cmd->position.assign(max_msg_idx + 1, 0.0);
        init_cmd->kd.assign(max_msg_idx + 1, 0.0);
        init_cmd->kp.assign(max_msg_idx + 1, 0.0);
        joint_cmd_ptr_.store(init_cmd);
        joint_state_msg_.position.resize(max_msg_idx + 1, 0.0);
        joint_state_msg_.velocity.resize(max_msg_idx + 1, 0.0);
        joint_state_msg_.effort.resize(max_msg_idx + 1, 0.0);
        joint_temp_msg_.data.resize(max_msg_idx + 1, 0.0);
        joint_state_age_msg_.data.resize(max_msg_idx + 1, std::nanf(""));
        joint_reply_after_tx_msg_.data.resize(max_msg_idx + 1, std::nanf(""));
        joint_cmd_to_bus_latency_msg_.data.resize(max_msg_idx + 1, std::nanf(""));
        joint_motor_reply_latency_msg_.data.resize(max_msg_idx + 1, std::nanf(""));
        joint_rx_ns_.assign(max_msg_idx + 1, 0);
        unfiltered_velocity_msg_.data.resize(max_msg_idx + 1, std::nanf(""));
        unfiltered_position_msg_.data.resize(max_msg_idx + 1, std::nanf(""));

        if (ros2_joint_state_pub_)
        {
            ros2_joint_state_msg_.position.resize(max_msg_idx + 1, 0.0);
            ros2_joint_state_msg_.velocity.resize(max_msg_idx + 1, 0.0);
            ros2_joint_state_msg_.effort.resize(max_msg_idx + 1, 0.0);
            ros2_joint_state_msg_.name.resize(max_msg_idx + 1, "");
        }

        for (unsigned int i = 0; i < joint_names.size(); i++)
        {
            // Declare joint definitions
            auto joint_config = cubemars::joint_config_per_motor_type.at(this->get_parameter("joint_defintions." + joint_names[i] + ".motor_type").as_string());
            joint_config.can_id = this->get_parameter("joint_defintions." + joint_names[i] + ".can_id").as_int();
            joint_config.invert = this->get_parameter("joint_defintions." + joint_names[i] + ".invert").as_bool();
            auto msg_idx = this->get_parameter("joint_defintions." + joint_names[i] + ".msg_idx").as_int();
            auto can_interface = this->get_parameter("joint_defintions." + joint_names[i] + ".can_interface").as_string();
            auto can_interface_id = std::distance(can_interfaces_names_.begin(), can_interfaces_names_.find(can_interface));
            unsigned int vel_filter_size = this->get_parameter("joint_defintions." + joint_names[i] + ".vel_filter_size").as_int();
            auto vel_filter_type_str = this->get_parameter("joint_defintions." + joint_names[i] + ".vel_filter_type").as_string();
            VelFilterType vel_filter_type;
            if (vel_filter_type_str == "none")
            {
                vel_filter_type = VelFilterType::NONE;
            }
            else if (vel_filter_type_str == "moving_average")
            {
                vel_filter_type = VelFilterType::MOVING_AVERAGE;
            }
            else if (vel_filter_type_str == "alpha_beta")
            {
                vel_filter_type = VelFilterType::ALPHA_BETA;
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Joint %s has unknown vel_filter_type '%s' (expected: none, moving_average, alpha_beta)", joint_names[i].c_str(), vel_filter_type_str.c_str());
                return LifecycleNodeInterface::CallbackReturn::FAILURE;
            }
            double alpha = this->get_parameter("joint_defintions." + joint_names[i] + ".alpha").as_double();
            double beta = this->get_parameter("joint_defintions." + joint_names[i] + ".beta").as_double();
            int pos_median_filter_size_raw = this->get_parameter("joint_defintions." + joint_names[i] + ".pos_median_filter_size").as_int();
            if (pos_median_filter_size_raw < 0)
            {
                RCLCPP_ERROR(this->get_logger(), "Joint %s has negative pos_median_filter_size %d", joint_names[i].c_str(), pos_median_filter_size_raw);
                return LifecycleNodeInterface::CallbackReturn::FAILURE;
            }
            unsigned int pos_median_filter_size = static_cast<unsigned int>(pos_median_filter_size_raw);
            joint_configs_per_can_interface[can_interface_id].push_back(joint_config);
            joint_commands_per_can_interface_[can_interface_id].push_back({0, 0, 0, 0, 0});
            joint_states_per_can_interface_[can_interface_id].push_back({0, 0, 0, 0, cubemars::ErrorCode::NO_FAULT, cubemars::ComStatus::SUCCESS, 0, 0, 0, 0});
            joint_parameters_per_can_interface_[can_interface_id].push_back({this->get_parameter("joint_defintions." + joint_names[i] + ".pos_limit_min").as_double(),
                                                                             this->get_parameter("joint_defintions." + joint_names[i] + ".pos_limit_max").as_double(),
                                                                             this->get_parameter("joint_defintions." + joint_names[i] + ".transmission_ratio").as_double(),
                                                                             {this->get_parameter("joint_defintions." + joint_names[i] + ".tau_c").as_double(),
                                                                              this->get_parameter("joint_defintions." + joint_names[i] + ".tau_s").as_double(),
                                                                              this->get_parameter("joint_defintions." + joint_names[i] + ".v_s").as_double(),
                                                                              this->get_parameter("joint_defintions." + joint_names[i] + ".k").as_double(),
                                                                              this->get_parameter("joint_defintions." + joint_names[i] + ".k_a").as_double(),
                                                                              this->get_parameter("joint_defintions." + joint_names[i] + ".b").as_double()},
                                                                                vel_filter_size,
                                                                             vel_filter_type,
                                                                             pos_median_filter_size,
                                                                             this->get_parameter("joint_defintions." + joint_names[i] + ".zero_position").as_double(),
                                                                             this->get_parameter("joint_defintions." + joint_names[i] + ".set_zero_position_on_configure").as_bool(),
                                                                             static_cast<unsigned int>(msg_idx),
                                                                             joint_names[i]});
            joint_vel_filters_per_can_interface_[can_interface_id].push_back(joint_parameters_per_can_interface_[can_interface_id].back().vel_filter_size);
            joint_ab_filters_per_can_interface_[can_interface_id].emplace_back(alpha, beta);
            joint_pos_median_filters_per_can_interface_[can_interface_id].emplace_back(pos_median_filter_size);
            last_joint_rx_ns_per_can_interface_[can_interface_id].push_back(0);
            joint_name_to_can_iface_and_idx_[joint_names[i]] = {
                static_cast<unsigned int>(can_interface_id),
                static_cast<unsigned int>(joint_parameters_per_can_interface_[can_interface_id].size() - 1)};

            if (ros2_joint_state_pub_)
            {
                ros2_joint_state_msg_.name[msg_idx] = joint_names[i];
            }
        }

        // Now create can devices and callback
        for (auto can_interface_name : can_interfaces_names_)
        {
            auto can_interface_id = std::distance(can_interfaces_names_.begin(), can_interfaces_names_.find(can_interface_name));
            can_interfaces_[can_interface_id] = std::make_shared<cubemars::CubemarsCan>(
                can_interface_name,
                this->get_parameter("enable_loopback").as_bool(),
                joint_configs_per_can_interface[can_interface_id],
                this->get_parameter("can_socket_timeout_sec").as_int(),
                this->get_parameter("can_socket_timeout_usec").as_int(),
                this->get_parameter("can_initial_connection_trials").as_int(),
                enable_tx_timestamping_
            );
        }

        // Goes to default callback group
        publish_timer_ = this->create_timer(frequency_, std::bind(&CubeMarsHardwareNode::joint_state_publish_callback, this));
        // Create subscriber
        joint_cmd_sub_ = this->create_subscription<robot_control_msgs::msg::JointCommand>("~/joint_commands", QOS_BEST_EFFORT_NO_DEPTH, std::bind(&CubeMarsHardwareNode::joint_cmd_msg_callback, this, std::placeholders::_1));

        // Create services
        set_all_motors_origin_here_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "set_all_motors_origin_here",
            std::bind(&CubeMarsHardwareNode::set_all_motors_origin_here_callback, this, std::placeholders::_1, std::placeholders::_2));
        set_motor_origin_here_srv_ = this->create_service<robot_control_msgs::srv::SetMotorOriginHere>(
            "set_motor_origin_here",
            std::bind(&CubeMarsHardwareNode::set_motor_origin_here_callback, this, std::placeholders::_1, std::placeholders::_2));

        // Now create can devices and callback
        bool failure = false;
        std::string error_string = "";
        for (unsigned int i = 0; i < can_interfaces_.size(); i++)
        { 
            for (unsigned int j = 0; j < joint_parameters_per_can_interface_[i].size(); j++)
            {
                try // We catch this to actually now which all are missing and if it is a bus problem or a motor problem
                {
                    can_interfaces_[i]->start_motor_control_mode(j, joint_parameters_per_can_interface_[i][j].set_zero_position_on_startup);
                    RCLCPP_INFO(this->get_logger(), "Succesfully enabled motor on can_interface %s with can_id %i", can_interfaces_[i]->GetName().c_str(), joint_configs_per_can_interface[i][j].can_id);
                }
                catch(const std::exception & e)
                {
                    error_string += std::string("\t") + e.what() + std::string("\n");
                    failure = true;
                }
            }
        }

        if(failure){
        
                // Notify users
                RCLCPP_ERROR(this->get_logger(), "Device error while enabling motor: \n %s", error_string.c_str());
                RCLCPP_WARN(this->get_logger(), "Try to disable motors, might not work");
                for (unsigned int i = 0; i < can_interfaces_.size(); i++)
                { 
                for (unsigned int j = 0; j < joint_parameters_per_can_interface_[i].size(); j++)
                    {
                     try
                        {
                            can_interfaces_[i]->end_motor_control_mode(j);
                        }
                        catch (const std::exception &e)
                        {
                            RCLCPP_WARN(this->get_logger(), "Device error while disabling motor on can_interface %s with can_id %i: %s\n BE CAREFULL WITH STILL ENABLED MOTORS!", can_interfaces_[i]->GetName().c_str(), joint_configs_per_can_interface[i][j].can_id, e.what());
                        }
                    }
                }
                can_interfaces_.clear();
                return LifecycleNodeInterface::CallbackReturn::ERROR;
        }
        
        // Register runtime parameter callback now that all per-joint state is in place
        on_set_parameters_handle_ = this->add_on_set_parameters_callback(
            std::bind(&CubeMarsHardwareNode::on_set_parameters_callback, this, std::placeholders::_1));

        // Supervisor on the executor: performs cleanup() when a comm thread requests it (a comm
        // thread can't cleanup() itself, as that would join its own thread). Created once and
        // kept for the node's lifetime; harmless while unconfigured (the flag is false).
        if (!supervisor_timer_)
        {
            supervisor_timer_ = this->create_timer(watchdog_frequency_, std::bind(&CubeMarsHardwareNode::supervisor_callback, this));
        }

        // Start one dedicated comm thread per interface (each loops send_and_receive back-to-back).
        for (unsigned int i = 0; i < can_interfaces_.size(); i++)
        {
            start_comm_thread(i);
        }
    }
    catch (rclcpp::exceptions::ParameterUninitializedException &exception)
    {
        // Notify user
        RCLCPP_ERROR(this->get_logger(), "A nececarry parameter is not set: %s", exception.what());
        return LifecycleNodeInterface::CallbackReturn::ERROR;
    }
    catch (cubemars::can_interface_error &exception)
    {
        // Notidy users
        RCLCPP_ERROR(this->get_logger(), "A CAN communication error occured: %s", exception.what());
        // This only happens during CAN creation, hence it should be enough to just deactivate the interfaces
        can_interfaces_.clear();
        return LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn CubeMarsHardwareNode::on_cleanup([[maybe_unused]] const rclcpp_lifecycle::State &previous_state)
{
    RCLCPP_WARN(this->get_logger(), "Cleaning up");
    // Unregister parameter callback first so it cannot fire during teardown
    on_set_parameters_handle_.reset();
    // Stop all timers and comm threads. Joining the comm threads BEFORE taking the mutexes is
    // essential: a thread mid-cycle holds can_communication_mutex_ shared, so locking it
    // exclusive before the join would deadlock.
    publish_timer_.reset();
    stop_all_comm_threads();
    comm_threads_.clear();

    can_communication_mutex_.lock(); // Nobody can enter
    joint_state_msg_mutex_.lock();
    bool success = true;
    // Disbale all motors
    for (unsigned int i = 0; i < can_interfaces_.size(); i++)
    {
        // Disable all motors one by one to avoid any motors being stuck
        for (unsigned int j = 0; j < joint_commands_per_can_interface_[i].size(); j++)
        {
            try
            {
                can_interfaces_[i]->end_motor_control_mode(j);
            }
            catch (std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Error when disabling motor %s on can interface %s (joint with msg_idx %i): %s", joint_parameters_per_can_interface_[i][j].name.c_str(), can_interfaces_[i]->GetName().c_str(), joint_parameters_per_can_interface_[i][j].msg_idx, e.what());
                success = false;
            }
        }
    }
    // Now delete everything
    try
    {
        can_interfaces_.clear();
    }
    catch (std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error when deactivating can_interfaces: %s", e.what());
        success = false;
    }

    joint_parameters_per_can_interface_.clear();
    joint_vel_filters_per_can_interface_.clear();
    joint_ab_filters_per_can_interface_.clear();
    joint_pos_median_filters_per_can_interface_.clear();
    last_joint_rx_ns_per_can_interface_.clear();
    joint_name_to_can_iface_and_idx_.clear();
    num_can_errors_per_interfaces_.clear();
    joint_states_per_can_interface_.clear();
    joint_commands_per_can_interface_.clear();
    can_interfaces_names_.clear();
    joint_cmd_sub_.reset();
    joint_state_pub_.reset();
    joint_temp_pub_.reset();
    can_interface_frequency_pub_.reset();
    can_tx_fill_duration_pub_.reset();
    can_rx_duration_pub_.reset();
    joint_reply_after_tx_pub_.reset();
    joint_state_age_pub_.reset();
    joint_cmd_to_bus_latency_pub_.reset();
    joint_motor_reply_latency_pub_.reset();
    controller_latency_pub_.reset();
    can_tx_fill_duration_msg_.data.clear();
    can_rx_duration_msg_.data.clear();
    joint_reply_after_tx_msg_.data.clear();
    joint_state_age_msg_.data.clear();
    joint_cmd_to_bus_latency_msg_.data.clear();
    joint_motor_reply_latency_msg_.data.clear();
    joint_rx_ns_.clear();
    unfiltered_velocity_pub_.reset();
    unfiltered_velocity_msg_.data.clear();
    unfiltered_position_pub_.reset();
    unfiltered_position_msg_.data.clear();
    last_can_cycle_times_.clear();
    joint_state_msg_mutex_.unlock();
    can_communication_mutex_.unlock();
    joint_cmd_ptr_.store(nullptr); // comm threads already joined, so no reader is active
    joint_state_msg_.position.clear();
    joint_state_msg_.velocity.clear();
    joint_state_msg_.effort.clear();
    joint_temp_msg_.data.clear();
    set_all_motors_origin_here_srv_.reset();
    set_motor_origin_here_srv_.reset();
    if (publish_ros2_joint_state_)
    {
        ros2_joint_state_pub_.reset();
        ros2_joint_state_msg_.position.clear();
        ros2_joint_state_msg_.velocity.clear();
        ros2_joint_state_msg_.effort.clear();
        ros2_joint_state_msg_.name.clear();
    }
    // Always return success, since then the driver is unconfigured(). And from there we can try to start over again.
    (void)success;
    RCLCPP_WARN(this->get_logger(), "Clean up done ok");
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn CubeMarsHardwareNode::on_activate([[maybe_unused]] const rclcpp_lifecycle::State &previous_state)
{
    if (!msg_received_)
    {
        RCLCPP_ERROR(this->get_logger(), "Trying to activate HW without valid joint message received");
        return LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
    // Start watchdog
    // Goes to default callback group
    watchdog_timer_ = this->create_timer(watchdog_frequency_, std::bind(&CubeMarsHardwareNode::watchdog_timer_callback, this));
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn CubeMarsHardwareNode::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State &previous_state)
{
    // The motors have to be send into damping: publish a fresh command with kd = damping, rest 0.
    auto damping = std::make_shared<robot_control_msgs::msg::JointCommand>();
    damping->effort.assign(joint_msg_length_, 0.0);
    damping->velocity.assign(joint_msg_length_, 0.0);
    damping->position.assign(joint_msg_length_, 0.0);
    damping->kp.assign(joint_msg_length_, 0.0);
    damping->kd.assign(joint_msg_length_, default_damping_KD_);
    joint_cmd_ptr_.store(damping);
    watchdog_timer_.reset();
    msg_received_ = false;
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn CubeMarsHardwareNode::on_shutdown(const rclcpp_lifecycle::State &previous_state)
{
    RCLCPP_INFO(this->get_logger(), "Fully shutting down driver node from state %s", previous_state.label().c_str());
    switch (previous_state.id())
    {
    case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE:
        try
        {
            on_deactivate(previous_state);
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(this->get_logger(), "Error during deactvation (%s), procceding", e.what());
        }
        [[fallthrough]];
    case lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE:
        try
        {
            on_cleanup(previous_state);
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(this->get_logger(), "Error during cleanup() (%s), procceding", e.what());
        }
        [[fallthrough]];
    default:
        break;
    }
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn CubeMarsHardwareNode::on_error(const rclcpp_lifecycle::State &previous_state)
{
    RCLCPP_WARN(this->get_logger(), "Error handling from previous state %s", previous_state.label().c_str());
    switch (previous_state.id())
    {
    case lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED:
        // Assumption: Something went wrong during configure()
        // Configure is handling the can realted stuff so here we just have to reset the data structures
        on_set_parameters_handle_.reset();
        joint_cmd_sub_.reset();
        publish_timer_.reset();
        joint_state_pub_.reset();
        joint_temp_pub_.reset();
        can_interface_frequency_pub_.reset();
        can_tx_fill_duration_pub_.reset();
        can_rx_duration_pub_.reset();
        joint_reply_after_tx_pub_.reset();
        joint_state_age_pub_.reset();
        joint_cmd_to_bus_latency_pub_.reset();
        joint_motor_reply_latency_pub_.reset();
        controller_latency_pub_.reset();
        unfiltered_velocity_pub_.reset();
        unfiltered_position_pub_.reset();
        can_interfaces_names_.clear();
        stop_all_comm_threads(); // join any comm threads started before configure() failed
        comm_threads_.clear();
        joint_cmd_ptr_.store(nullptr);
        joint_state_msg_.position.clear();
        joint_state_msg_.velocity.clear();
        joint_state_msg_.effort.clear();
        joint_temp_msg_.data.clear();
        joint_states_per_can_interface_.clear();
        joint_commands_per_can_interface_.clear();
        num_can_errors_per_interfaces_.clear();
        joint_parameters_per_can_interface_.clear();
        joint_vel_filters_per_can_interface_.clear();
        joint_ab_filters_per_can_interface_.clear();
        joint_pos_median_filters_per_can_interface_.clear();
        last_joint_rx_ns_per_can_interface_.clear();
        joint_name_to_can_iface_and_idx_.clear();
        can_tx_fill_duration_msg_.data.clear();
        can_rx_duration_msg_.data.clear();
        joint_reply_after_tx_msg_.data.clear();
        joint_state_age_msg_.data.clear();
        joint_cmd_to_bus_latency_msg_.data.clear();
        joint_motor_reply_latency_msg_.data.clear();
        joint_rx_ns_.clear();
        unfiltered_velocity_msg_.data.clear();
        unfiltered_position_msg_.data.clear();
        set_all_motors_origin_here_srv_.reset();
        set_motor_origin_here_srv_.reset();
        if (publish_ros2_joint_state_)
        {
            ros2_joint_state_pub_.reset();
            ros2_joint_state_msg_.position.clear();
            ros2_joint_state_msg_.velocity.clear();
            ros2_joint_state_msg_.effort.clear();
            ros2_joint_state_msg_.name.clear();
        }
        RCLCPP_INFO(this->get_logger(), "Handling error in PRIMARY_STATE_UNCONFIGURED sucessfull");
        break;
    case lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE:
        // Assumption: Something went wrong during activate() or cleanup()
        // During activate nothing can really go wrong so it must be cleanup in both cases something is really fucked up, hence we wont allow anything than finalizing this
        return LifecycleNodeInterface::CallbackReturn::FAILURE;
    case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE:
        // Something went wrong during deactivating() or shuttingDown()
        // deactivate(), nothing can really go wrong, but if something has happened here the motors are not in damping
        RCLCPP_ERROR(this->get_logger(), "Motors might be active, be careful!");
        // shuttingDown() is invovling all the other tasks and is trying to handle all errors by itself. If we end up here we cant to anything
        return LifecycleNodeInterface::CallbackReturn::FAILURE;
    default:
        RCLCPP_ERROR(this->get_logger(), "Error handling in undefined state, call Franek: +49 421 17845 4112 :)");
        return LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void CubeMarsHardwareNode::watchdog_timer_callback()
{
    if (!msg_received_)
    {
        RCLCPP_ERROR(this->get_logger(), "No msg received inbetween two watchdog cycles - deactivating hardware into damping");
        deactivate();
    }
    msg_received_ = false;
}

void CubeMarsHardwareNode::joint_cmd_msg_callback(const robot_control_msgs::msg::JointCommand::ConstSharedPtr &joint_cmd_msg)
{

    if (joint_cmd_msg->position.size() != joint_msg_length_ ||
        joint_cmd_msg->velocity.size() != joint_msg_length_ ||
        joint_cmd_msg->effort.size() != joint_msg_length_ ||
        joint_cmd_msg->kp.size() != joint_msg_length_ ||
        joint_cmd_msg->kd.size() != joint_msg_length_)
    {
        RCLCPP_ERROR(this->get_logger(), "Received joint_cmd_msg with array sizes unequal to %i, skipping this message", joint_msg_length_);
        return;
    }

    msg_received_ = true;
    // Only accept message if active
    if (this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
        // Record when this command was received on the subscriber (CLOCK_REALTIME, comparable with
        // the software TX timestamp) so can_cycle_callback can report the command->wire latency.
        struct timespec cmd_rx_ts;
        clock_gettime(CLOCK_REALTIME, &cmd_rx_ts);
        cmd_rx_ns_.store(static_cast<int64_t>(cmd_rx_ts.tv_sec) * 1000000000LL + cmd_rx_ts.tv_nsec, std::memory_order_relaxed);

        // Publish the new command lock-free: store the shared_ptr rclcpp gave us directly (no copy).
        joint_cmd_ptr_.store(joint_cmd_msg);
    }

    int64_t cmd_stamp_ns = static_cast<int64_t>(joint_cmd_msg->stamp.sec) * 1000000000LL + joint_cmd_msg->stamp.nanosec;
    if (cmd_stamp_ns > 0) // Skip unstamped commands (stamp left at zero)
    {
        std_msgs::msg::Float32 latency_msg;
        latency_msg.data = static_cast<float>((this->get_clock()->now().nanoseconds() - cmd_stamp_ns) / 1e3);
        controller_latency_pub_->publish(latency_msg);
    }
}

void CubeMarsHardwareNode::joint_state_publish_callback()
{
    // CLOCK_REALTIME 'now' for the per-joint state age (shares the epoch with the RX timestamps).
    int64_t pub_now_ns = this->get_clock()->now().nanoseconds();
    joint_state_msg_mutex_.lock();
    joint_state_msg_to_pub_ = joint_state_msg_;
    joint_temp_msg_to_pub_ = joint_temp_msg_;
    can_interface_frequency_msg_to_pub_ = can_interface_frequency_msg_;
    can_tx_fill_duration_msg_to_pub_ = can_tx_fill_duration_msg_;
    can_rx_duration_msg_to_pub_ = can_rx_duration_msg_;
    joint_reply_after_tx_msg_to_pub_ = joint_reply_after_tx_msg_;
    joint_cmd_to_bus_latency_msg_to_pub_ = joint_cmd_to_bus_latency_msg_;
    joint_motor_reply_latency_msg_to_pub_ = joint_motor_reply_latency_msg_;
    unfiltered_velocity_msg_to_pub_ = unfiltered_velocity_msg_;
    unfiltered_position_msg_to_pub_ = unfiltered_position_msg_;
    // Stamp with the oldest successful reply currently in the message: a conservative
    // freshness bound (every joint value is no older than `stamp`). The kernel RX timestamp
    // (SO_TIMESTAMPNS, CLOCK_REALTIME ns) shares the epoch with ROS SYSTEM_TIME.
    int64_t oldest_rx_ns = 0;
    for (unsigned int idx = 0; idx < joint_rx_ns_.size(); idx++)
    {
        int64_t rx_ns = joint_rx_ns_[idx];
        if (rx_ns > 0 && (oldest_rx_ns == 0 || rx_ns < oldest_rx_ns))
        {
            oldest_rx_ns = rx_ns;
        }
        // Per-joint age of this joint's state at publish time; NaN until the first reply.
        joint_state_age_msg_.data[idx] = (rx_ns > 0) ? static_cast<float>((pub_now_ns - rx_ns) / 1e3) : std::nanf("");
    }
    joint_state_msg_mutex_.unlock();
    if (oldest_rx_ns > 0)
    {
        joint_state_msg_to_pub_.stamp.sec = static_cast<int32_t>(oldest_rx_ns / 1000000000LL);
        joint_state_msg_to_pub_.stamp.nanosec = static_cast<uint32_t>(oldest_rx_ns % 1000000000LL);
    }
    else
    {
        // No reply received yet (e.g. first cycles) - fall back to current node time.
        joint_state_msg_to_pub_.stamp = this->get_clock()->now();
    }
    joint_state_pub_->publish(joint_state_msg_to_pub_); // TODO: check if this blocks and maybe avoid block during mutex
    joint_temp_pub_->publish(joint_temp_msg_to_pub_);
    can_interface_frequency_pub_->publish(can_interface_frequency_msg_to_pub_);
    can_tx_fill_duration_pub_->publish(can_tx_fill_duration_msg_to_pub_);
    can_rx_duration_pub_->publish(can_rx_duration_msg_to_pub_);
    joint_reply_after_tx_pub_->publish(joint_reply_after_tx_msg_to_pub_);
    joint_state_age_pub_->publish(joint_state_age_msg_);
    if (joint_cmd_to_bus_latency_pub_) // only created when TX timestamping is enabled
    {
        joint_cmd_to_bus_latency_pub_->publish(joint_cmd_to_bus_latency_msg_to_pub_);
        joint_motor_reply_latency_pub_->publish(joint_motor_reply_latency_msg_to_pub_);
    }
    unfiltered_velocity_pub_->publish(unfiltered_velocity_msg_to_pub_);
    unfiltered_position_pub_->publish(unfiltered_position_msg_to_pub_);
    if (publish_ros2_joint_state_)
    {
        ros2_joint_state_msg_.position = joint_state_msg_to_pub_.position;
        ros2_joint_state_msg_.velocity = joint_state_msg_to_pub_.velocity;
        ros2_joint_state_msg_.effort = joint_state_msg_to_pub_.effort;
        ros2_joint_state_pub_->publish(ros2_joint_state_msg_);
    }
}

void CubeMarsHardwareNode::comm_loop(unsigned int can_interface_idx)
{
    // Run the send/receive cycle back-to-back until asked to stop. Free-runs at the bus's
    // natural rate; no executor dispatch between cycles.
    while (comm_threads_[can_interface_idx]->running.load(std::memory_order_relaxed))
    {
        can_cycle_callback(can_interface_idx);
    }
}

void CubeMarsHardwareNode::start_comm_thread(unsigned int can_interface_idx)
{
    auto &ct = comm_threads_[can_interface_idx];
    if (ct->running.load())
    {
        return; // already running
    }
    if (ct->thread.joinable())
    {
        ct->thread.join(); // join a previous (stopped) instance before respawning
    }
    ct->running.store(true);
    ct->thread = std::thread(&CubeMarsHardwareNode::comm_loop, this, can_interface_idx);
    // Run the comm loop at real-time priority (best effort, like main()).
    sched_param sch;
    sch.sched_priority = 80;
    if (pthread_setschedparam(ct->thread.native_handle(), SCHED_FIFO, &sch) != 0)
    {
        RCLCPP_WARN(this->get_logger(), "Failed to set RT priority on comm thread for can interface %i", can_interface_idx);
    }
}

void CubeMarsHardwareNode::stop_comm_thread(unsigned int can_interface_idx)
{
    auto &ct = comm_threads_[can_interface_idx];
    ct->running.store(false);
    if (ct->thread.joinable())
    {
        ct->thread.join();
    }
}

void CubeMarsHardwareNode::stop_all_comm_threads()
{
    // Signal all to stop first, then join, so they wind down in parallel (not one-by-one).
    for (auto &ct : comm_threads_)
    {
        if (ct)
        {
            ct->running.store(false);
        }
    }
    for (auto &ct : comm_threads_)
    {
        if (ct && ct->thread.joinable())
        {
            ct->thread.join();
        }
    }
}

void CubeMarsHardwareNode::supervisor_callback()
{
    // A comm thread cannot run cleanup() (it would join itself); it requests it here and the
    // executor thread performs the transition.
    if (cleanup_requested_.exchange(false))
    {
        RCLCPP_ERROR(this->get_logger(), "Comm thread requested cleanup after a fatal error; cleaning up");
        cleanup();
    }
}

void CubeMarsHardwareNode::can_cycle_callback(unsigned int can_interface_idx)
{
    if (!(this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE || this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE))
    {
        return;
    }

    // Block runtime parameter updates from interleaving with this cycle's reads/writes of joint params and filters.
    std::shared_lock<WritePreferringSharedMutex> params_lock(joint_params_mutex_);

    auto &joint_cmds = joint_commands_per_can_interface_[can_interface_idx];
    auto &joint_states = joint_states_per_can_interface_[can_interface_idx];
    auto &joint_params = joint_parameters_per_can_interface_[can_interface_idx];

    // Calculate timings
    auto current_time = this->get_clock()->now();
    double dt = (current_time - last_can_cycle_times_[can_interface_idx]).nanoseconds() / 1e9;
    double can_cyle_frequency = 1.0 / dt;
    last_can_cycle_times_[can_interface_idx] = current_time;

    // Collect data: lock-free load of the latest command (an immutable snapshot).
    if (!comm_threads_[can_interface_idx]->running.load(std::memory_order_relaxed))
    {
        return; // Stop was requested mid-cycle
    }
    auto cmd = joint_cmd_ptr_.load();
    if (!cmd)
    {
        return; // No command published yet (e.g. torn down)
    }
    for (unsigned int i = 0; i < joint_cmds.size(); i++)
    {
        joint_cmds[i].kd = cmd->kd[joint_params[i].msg_idx];
        joint_cmds[i].kp = cmd->kp[joint_params[i].msg_idx];
        joint_cmds[i].pos = cmd->position[joint_params[i].msg_idx] - joint_params[i].zero_position;
        joint_cmds[i].vel = cmd->velocity[joint_params[i].msg_idx];
        joint_cmds[i].torque = cmd->effort[joint_params[i].msg_idx];
    }

    if (this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
        // friction model
        for (unsigned int i = 0; i < joint_cmds.size(); i++)
        {
            /**
             * Stribeck friction model with 5 parameters (tau_c, tau_s, v_s, k, b):
             * tau_f =  tau_c*tanh(k_a*v) + (tau_s - tau_c)*exp(-(|v|/v_s)^k)*tanh(k_a*v) + b*v
             */
            double tau_c = joint_params[i].friction_parameters.tau_c; // Coulomb friction
            double tau_s = joint_params[i].friction_parameters.tau_s; // Static friction
            double v_s = joint_params[i].friction_parameters.v_s;     // Stribeck Velocity
            double k = joint_params[i].friction_parameters.k;         // Exponential term
            double k_a = joint_params[i].friction_parameters.k_a;     // Steepness factor for tanh (instead of sign)
            double b = joint_params[i].friction_parameters.b;         // Viscous friction coefficient
            double vel = joint_states[i].vel;                         // Actual velocity
            // double cmd_vel = joint_cmds[i].vel;                       // Commanded velocity (used to argue about motion direction)

            double sign_vel = tanh(k_a * vel);
            joint_cmds[i].torque += tau_c * sign_vel + (tau_s - tau_c) * exp(-pow(fabs(vel) / v_s, k)) * sign_vel + b * vel;
        }
        // transmission ratios
        for (unsigned int i = 0; i < joint_cmds.size(); i++)
        {
            joint_cmds[i].pos *= joint_params[i].transmission_ratio;
            joint_cmds[i].vel *= joint_params[i].transmission_ratio;
            joint_cmds[i].torque *= 1.0 / joint_params[i].transmission_ratio;
        }
    }

    // Send and receive
    can_communication_mutex_.lock_shared();
    try
    {
        can_interfaces_[can_interface_idx]->send_and_receive(joint_cmds, joint_states);
        can_communication_mutex_.unlock_shared();
    }
    catch (const std::exception &e)
    {
        // This can only happen when something really bad happend (as all other erros are handled somehow else)
        can_communication_mutex_.unlock_shared();
        RCLCPP_ERROR(this->get_logger(), "%s", e.what());
        if (this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
        {
            // Try to go into damping
            RCLCPP_WARN(this->get_logger(), "For safety reasons deactivate() motors into DAMPING");
            deactivate();
            return;
        }
        else
        {
            // TODO: would make sense to keep damping active it is not all motors that lost comms, but for spmilcity we unconfigure here
            RCLCPP_ERROR(this->get_logger(), "For safety reasons requesting cleanup() motors into OFF (can process %i)", can_interface_idx);
            cleanup_requested_.store(true); // supervisor (executor thread) runs cleanup(); a comm thread must not join itself
            return;
        }
    }

    unsigned int can_errors_in_this_cycle = 0;

    // Check status
    for (unsigned int i = 0; i < joint_cmds.size(); i++)
    {
        if (joint_states[i].device_status != cubemars::ErrorCode::NO_FAULT)
        {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Joint %s on can_interface %s with can_id %i has error %s", joint_params[i].name.c_str(), can_interfaces_[can_interface_idx]->GetName().c_str(), can_interfaces_[can_interface_idx]->get_can_id(i), cubemars::errorFlagToString(joint_states[i].device_status));
            if (this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE && damping_on_motor_error_)
            {
                RCLCPP_ERROR(this->get_logger(), "Deactivating hardware into damping due to error on joint %s", joint_params[i].name.c_str());
                deactivate();
            }
        }
        if (joint_states[i].communication_status != cubemars::ComStatus::SUCCESS)
        {
            RCLCPP_WARN(this->get_logger(), "Joint %s on can_interface %s with can_id %i has communication issues %s: %s", joint_params[i].name.c_str(), can_interfaces_[can_interface_idx]->GetName().c_str(), can_interfaces_[can_interface_idx]->get_can_id(i), cubemars::comStatusToString(joint_states[i].communication_status), strerror(joint_states[i].com_errno));
            can_errors_in_this_cycle++;
        }
    }

    if (can_errors_in_this_cycle == 0 && num_can_errors_per_interfaces_[can_interface_idx] > 0)
    {
        num_can_errors_per_interfaces_[can_interface_idx] = 0; // Reset counter
        RCLCPP_INFO(this->get_logger(), "Can interface %s communication with all motors okay again", can_interfaces_[can_interface_idx]->GetName().c_str());
    }
    else if ((num_can_errors_per_interfaces_[can_interface_idx] + can_errors_in_this_cycle) >= max_can_errors_before_motor_shutdown_)
    {
        RCLCPP_ERROR(this->get_logger(), "Can interface %s exceeded maximum number of CAN errors (%i)", can_interfaces_[can_interface_idx]->GetName().c_str(), max_can_errors_before_motor_shutdown_);
        if (this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
        {
            // Try to go into damping
            RCLCPP_WARN(this->get_logger(), "For safety reasons deactivate() motors into DAMPING");
            deactivate();
            return;
        }
        else
        {
            // TODO: would make sense to keep damping active it is not all motors that lost comms, but for spmilcity we unconfigure here
            RCLCPP_ERROR(this->get_logger(), "For safety reasons requesting cleanup() motors into OFF (can process %i)", can_interface_idx);
            cleanup_requested_.store(true); // supervisor (executor thread) runs cleanup(); a comm thread must not join itself
            return;
        }

        return;
    }
    else
    {
        num_can_errors_per_interfaces_[can_interface_idx] += can_errors_in_this_cycle;
    }

    // Snapshot of motor-space pos/vel before any filtering (median, MA, AB); published post-transmission below.
    std::vector<float> unfiltered_pos_motor_space(joint_states.size());
    std::vector<float> unfiltered_vel_motor_space(joint_states.size());
    for (unsigned int i = 0; i < joint_states.size(); i++)
    {
        unfiltered_pos_motor_space[i] = joint_states[i].pos;
        unfiltered_vel_motor_space[i] = joint_states[i].vel;
    }

    // Pre-filter: median filter on position (outlier rejection before AB sees it).
    // Skip on comm failure so we don't feed stale or zero values into the median window.
    for (unsigned int i = 0; i < joint_states.size(); i++)
    {
        if (joint_params[i].pos_median_filter_size > 1 &&
            joint_states[i].communication_status == cubemars::ComStatus::SUCCESS)
        {
            joint_states[i].pos = joint_pos_median_filters_per_can_interface_[can_interface_idx][i].update(joint_states[i].pos);
        }
    }

    for(unsigned int i = 0; i < joint_states.size(); i++)
    {
        switch (joint_params[i].vel_filter_type)
        {
        case VelFilterType::MOVING_AVERAGE:
            if (joint_params[i].vel_filter_size > 1)
            {
                joint_states[i].vel = joint_vel_filters_per_can_interface_[can_interface_idx][i].update(joint_states[i].vel);
            }
            break;
        case VelFilterType::ALPHA_BETA:
        {
            // Only update on a fresh reply; otherwise preserve filter state
            if (joint_states[i].communication_status == cubemars::ComStatus::SUCCESS)
            {
                int64_t rx_ns = joint_states[i].rx_timestamp_ns;
                int64_t last_ns = last_joint_rx_ns_per_can_interface_[can_interface_idx][i];
                double joint_dt = (last_ns == 0) ? 0.0 : (rx_ns - last_ns) / 1e9;
                auto &ab = joint_ab_filters_per_can_interface_[can_interface_idx][i];
                ab.update(joint_states[i].pos, joint_dt);
                joint_states[i].pos = ab.position();
                joint_states[i].vel = ab.velocity();
                last_joint_rx_ns_per_can_interface_[can_interface_idx][i] = rx_ns;
            }
            break;
        }
        case VelFilterType::NONE:
            break;
        }
    }

    // Publish unfiltered pos/vel in joint-space units (same units as ~/joint_states.{position,velocity})
    for (unsigned int i = 0; i < joint_states.size(); i++)
    {
        if (joint_states[i].communication_status == cubemars::ComStatus::SUCCESS)
        {
            unfiltered_velocity_msg_.data[joint_params[i].msg_idx] =
                unfiltered_vel_motor_space[i] / joint_params[i].transmission_ratio;
            unfiltered_position_msg_.data[joint_params[i].msg_idx] =
                unfiltered_pos_motor_space[i] / joint_params[i].transmission_ratio + joint_params[i].zero_position;
        }
        else
        {
            unfiltered_velocity_msg_.data[joint_params[i].msg_idx] = std::nanf("");
            unfiltered_position_msg_.data[joint_params[i].msg_idx] = std::nanf("");
        }
    }

    // Add transmission ratios to joint states
    for (unsigned int i = 0; i < joint_states.size(); i++)
    {
        joint_states[i].pos *= 1.0 / joint_params[i].transmission_ratio;
        joint_states[i].vel *= 1.0 / joint_params[i].transmission_ratio;
        joint_states[i].torque *= joint_params[i].transmission_ratio;
    }

    // Write back state
    joint_state_msg_mutex_.lock_shared();
    if (!comm_threads_[can_interface_idx]->running.load(std::memory_order_relaxed))
    {
        joint_state_msg_mutex_.unlock_shared();
        return; // Stop was requested mid-cycle
    }
    int64_t cmd_rx_ns = cmd_rx_ns_.load(std::memory_order_relaxed);
    int64_t tx_fill_end_ns = can_interfaces_[can_interface_idx]->get_tx_fill_end_ns();
    for (unsigned int i = 0; i < joint_states.size(); i++)
    {
        joint_state_msg_.position[joint_params[i].msg_idx] = joint_states[i].pos + joint_params[i].zero_position;
        joint_state_msg_.velocity[joint_params[i].msg_idx] = joint_states[i].vel;
        joint_state_msg_.effort[joint_params[i].msg_idx] = joint_states[i].torque;
        joint_temp_msg_.data[joint_params[i].msg_idx] = joint_states[i].temp;
        // Only refresh the RX timestamp on a fresh reply; on comm failure the pos/vel/effort
        // values above are stale, so we keep the joint's last successful sample time.
        if (joint_states[i].communication_status == cubemars::ComStatus::SUCCESS)
        {
            joint_rx_ns_[joint_params[i].msg_idx] = joint_states[i].rx_timestamp_ns;
        }
        // Motor turnaround: time from this cycle's command frame on the wire (TX) to its reply (RX).
        // Needs a fresh reply this cycle and a valid TX timestamp for the same frame.
        if (joint_states[i].communication_status == cubemars::ComStatus::SUCCESS &&
            joint_states[i].send_timestamp_ns > 0 &&
            joint_states[i].rx_timestamp_ns >= joint_states[i].send_timestamp_ns)
        {
            joint_motor_reply_latency_msg_.data[joint_params[i].msg_idx] =
                static_cast<float>((joint_states[i].rx_timestamp_ns - joint_states[i].send_timestamp_ns) / 1e3);
        }
        else
        {
            joint_motor_reply_latency_msg_.data[joint_params[i].msg_idx] = std::nanf("");
        }
        // Latency from ROS command receipt to this joint's frame going on the wire (software TX timestamp).
        if (joint_states[i].send_timestamp_ns > 0 && cmd_rx_ns > 0)
        {
            joint_cmd_to_bus_latency_msg_.data[joint_params[i].msg_idx] =
                static_cast<float>((joint_states[i].send_timestamp_ns - cmd_rx_ns) / 1e3);
        }
        else
        {
            joint_cmd_to_bus_latency_msg_.data[joint_params[i].msg_idx] = std::nanf("");
        }
        // Time from the TX buffer being filled to this motor's reply arriving in node space (userspace dequeue).
        if (joint_states[i].dequeue_timestamp_ns > 0 && tx_fill_end_ns > 0)
        {
            joint_reply_after_tx_msg_.data[joint_params[i].msg_idx] =
                static_cast<float>((joint_states[i].dequeue_timestamp_ns - tx_fill_end_ns) / 1e3);
        }
        else
        {
            joint_reply_after_tx_msg_.data[joint_params[i].msg_idx] = std::nanf("");
        }
    }
    // Check limits
    if (this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
        for (unsigned int i = 0; i < joint_states.size(); i++)
        {
            if (joint_state_msg_.position[joint_params[i].msg_idx] < joint_params[i].pos_limit_min)
            {
                RCLCPP_ERROR(this->get_logger(), "Joint %s on can_interface %s (msg idx %i) violated min position limit (%f < %f) - deactivating (damping)", joint_params[i].name.c_str(), can_interfaces_[can_interface_idx]->GetName().c_str(), joint_params[i].msg_idx, joint_state_msg_.position[joint_params[i].msg_idx], joint_params[i].pos_limit_min);
                joint_state_msg_mutex_.unlock_shared();
                deactivate();
                return;
            }
            if (joint_state_msg_.position[joint_params[i].msg_idx] > joint_params[i].pos_limit_max)
            {
                RCLCPP_ERROR(this->get_logger(), "Joint %s on can_interface %s (msg idx %i) violated max position limit (%f > %f) - deactivating (damping)", joint_params[i].name.c_str(), can_interfaces_[can_interface_idx]->GetName().c_str(), joint_params[i].msg_idx, joint_state_msg_.position[joint_params[i].msg_idx], joint_params[i].pos_limit_max);
                joint_state_msg_mutex_.unlock_shared();
                deactivate();
                return;
            }
        }
    }

    

    can_interface_frequency_msg_.data[can_interface_idx] = can_cyle_frequency;
    can_tx_fill_duration_msg_.data[can_interface_idx] = static_cast<float>(can_interfaces_[can_interface_idx]->get_tx_fill_duration_ns() / 1e3);
    can_rx_duration_msg_.data[can_interface_idx] = static_cast<float>(can_interfaces_[can_interface_idx]->get_rx_duration_ns() / 1e3);
    joint_state_msg_mutex_.unlock_shared();
}

void CubeMarsHardwareNode::set_all_motors_origin_here_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                                               std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;
    if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
    {
        RCLCPP_ERROR(this->get_logger(), "Cannot set all motors origin here, node not in CONFIGURED state");

        response->success = false;
        response->message = "Node not in CONFIGURED state";
        return;
    }
    // Pause all comm threads (join) before grabbing the bus exclusively for calibration.
    stop_all_comm_threads();
    can_communication_mutex_.lock();

    for (unsigned int i = 0; i < can_interfaces_.size(); i++)
    {
        try
        {
            for (unsigned int j = 0; j < joint_parameters_per_can_interface_[i].size(); j++)
            {
                RCLCPP_INFO(this->get_logger(), "Deactivate joint %s (can_interface %s, can id %i)", joint_parameters_per_can_interface_[i][j].name.c_str(), can_interfaces_[i]->GetName().c_str(), can_interfaces_[i]->get_can_id(j));
                can_interfaces_[i]->end_motor_control_mode(j);
                RCLCPP_INFO(this->get_logger(), "Set origin here on joint %s (can_interface %s, can id %i)", joint_parameters_per_can_interface_[i][j].name.c_str(), can_interfaces_[i]->GetName().c_str(), can_interfaces_[i]->get_can_id(j));
                can_interfaces_[i]->start_motor_control_mode(j, true);
                RCLCPP_INFO(this->get_logger(), "Succesfully set orgin and re-activated joint %s (can_interface %s, can id %i)", joint_parameters_per_can_interface_[i][j].name.c_str(), can_interfaces_[i]->GetName().c_str(), can_interfaces_[i]->get_can_id(j));
            }
        }
        catch (const std::exception &e)
        {
            // Notidy users
            RCLCPP_ERROR(this->get_logger(), "Device error on CAN interface %s occured while setting origin: %s", can_interfaces_[i]->GetName().c_str(), e.what());
            // This can only happen when actual motors are enabled, hence try to disable motors
            try
            {
                for (unsigned int i_o = 0; i_o <= can_interfaces_.size(); i_o++)
                {
                    // Enable all motors
                    can_interfaces_[i_o]->end_motor_control_mode();
                }
                can_interfaces_.clear();
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Device error on CAN interface %s during deactivation occured, be carefull with still active motors: %s", can_interfaces_[i]->GetName().c_str(), e.what());
            }
            can_communication_mutex_.unlock();
            response->success = false;
            response->message = "Error in CAN communication, see log";
            cleanup();
        }
    }
    // Resume the comm threads after calibration (skipped if cleanup() emptied can_interfaces_).
    for (unsigned int i = 0; i < can_interfaces_.size(); i++)
    {
        start_comm_thread(i);
    }
    response->success = true;
    response->message = "All motors set to origin";
    can_communication_mutex_.unlock();
}

void CubeMarsHardwareNode::set_motor_origin_here_callback(
    const std::shared_ptr<robot_control_msgs::srv::SetMotorOriginHere::Request> request,
    std::shared_ptr<robot_control_msgs::srv::SetMotorOriginHere::Response> response)
{
    if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
    {
        RCLCPP_ERROR(this->get_logger(), "Cannot set motor origin here, node not in CONFIGURED state");
        response->success = false;
        response->message = "Node not in CONFIGURED state";
        return;
    }

    // Find which can_interface and joint index corresponds to the given motor_id (msg_idx)
    int target_can_interface_id = -1;
    unsigned int target_joint_idx = 0;
    for (unsigned int i = 0; i < joint_parameters_per_can_interface_.size(); i++)
    {
        for (unsigned int j = 0; j < joint_parameters_per_can_interface_[i].size(); j++)
        {
            if (joint_parameters_per_can_interface_[i][j].msg_idx == static_cast<unsigned int>(request->motor_id))
            {
                target_can_interface_id = static_cast<int>(i);
                target_joint_idx = j;
                break;
            }
        }
        if (target_can_interface_id != -1)
        {
            break;
        }
    }

    if (target_can_interface_id == -1)
    {
        RCLCPP_ERROR(this->get_logger(), "Cannot set motor origin: motor_id %i not found", request->motor_id);
        response->success = false;
        response->message = "Motor ID " + std::to_string(request->motor_id) + " not found";
        return;
    }

    unsigned int iface = static_cast<unsigned int>(target_can_interface_id);

    // Pause the comm thread for the target interface (join) before grabbing the bus.
    stop_comm_thread(iface);

    can_communication_mutex_.lock();

    try
    {
        // Turn off all motors on this CAN interface
        for (unsigned int j = 0; j < joint_parameters_per_can_interface_[iface].size(); j++)
        {
            RCLCPP_INFO(this->get_logger(), "Deactivate joint %s (can_interface %s, can id %i)", joint_parameters_per_can_interface_[iface][j].name.c_str(), can_interfaces_[iface]->GetName().c_str(), can_interfaces_[iface]->get_can_id(j));
            can_interfaces_[iface]->end_motor_control_mode(j);
        }
        // Re-enable all motors; set zero position only for the target joint
        for (unsigned int j = 0; j < joint_parameters_per_can_interface_[iface].size(); j++)
        {
            bool set_zero = (j == target_joint_idx);
            if (set_zero)
            {
                RCLCPP_INFO(this->get_logger(), "Set origin here on joint %s (can_interface %s, can id %i)", joint_parameters_per_can_interface_[iface][j].name.c_str(), can_interfaces_[iface]->GetName().c_str(), can_interfaces_[iface]->get_can_id(j));
            }
            can_interfaces_[iface]->start_motor_control_mode(j, set_zero);
            RCLCPP_INFO(this->get_logger(), "Succesfully %s joint %s (can_interface %s, can id %i)",
                        set_zero ? "set origin and re-activated" : "re-activated",
                        joint_parameters_per_can_interface_[iface][j].name.c_str(),
                        can_interfaces_[iface]->GetName().c_str(),
                        can_interfaces_[iface]->get_can_id(j));
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Device error on CAN interface %s occured while setting origin: %s", can_interfaces_[iface]->GetName().c_str(), e.what());
        try
        {
            for (unsigned int i_o = 0; i_o < can_interfaces_.size(); i_o++)
            {
                can_interfaces_[i_o]->end_motor_control_mode();
            }
            can_interfaces_.clear();
        }
        catch (const std::exception &e_inner)
        {
            RCLCPP_ERROR(this->get_logger(), "Device error during emergency deactivation, be carefull with still active motors: %s", e_inner.what());
        }
        can_communication_mutex_.unlock();
        response->success = false;
        response->message = "Error in CAN communication, see log";
        cleanup();
        return;
    }

    // Resume the comm thread for the target interface.
    start_comm_thread(iface);
    response->success = true;
    response->message = "Motor " + std::to_string(request->motor_id) + " (" + joint_parameters_per_can_interface_[iface][target_joint_idx].name + ") set to origin";
    can_communication_mutex_.unlock();
}

bool CubeMarsHardwareNode::parse_per_joint_param(const std::string &name, std::string &joint_name_out, std::string &field_out) const
{
    static const std::string prefix = "joint_defintions.";
    if (name.rfind(prefix, 0) != 0)
    {
        return false;
    }
    auto dot = name.find('.', prefix.size());
    if (dot == std::string::npos)
    {
        return false;
    }
    joint_name_out = name.substr(prefix.size(), dot - prefix.size());
    field_out = name.substr(dot + 1);
    return !joint_name_out.empty() && !field_out.empty();
}

rcl_interfaces::msg::SetParametersResult CubeMarsHardwareNode::on_set_parameters_callback(
    const std::vector<rclcpp::Parameter> &params)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    // Per-joint pending updates, indexed by joint name
    struct PerJointUpdate
    {
        std::optional<VelFilterType> vel_filter_type;
        std::optional<int64_t> vel_filter_size;
        std::optional<int64_t> pos_median_filter_size;
        std::optional<double> alpha;
        std::optional<double> beta;
        std::optional<double> tau_c, tau_s, v_s, k, k_a, b;
        std::optional<double> pos_limit_min, pos_limit_max;
    };
    std::unordered_map<std::string, PerJointUpdate> updates;

    // Global pending updates
    std::optional<double> new_default_damping_KD;
    std::optional<bool> new_damping_on_motor_error;
    std::optional<int64_t> new_max_can_errors;

    auto fail = [&](const std::string &reason)
    {
        result.successful = false;
        result.reason = reason;
        return result;
    };

    // Pass 1: parse + per-param validation
    for (const auto &p : params)
    {
        const std::string &name = p.get_name();
        if (name == "default_damping_KD")
        {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
                return fail(name + ": must be double");
            new_default_damping_KD = p.as_double();
        }
        else if (name == "damping_on_motor_error")
        {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_BOOL)
                return fail(name + ": must be bool");
            new_damping_on_motor_error = p.as_bool();
        }
        else if (name == "max_can_errors_before_motor_shutdown")
        {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER)
                return fail(name + ": must be int");
            auto v = p.as_int();
            if (v < 1)
                return fail(name + ": must be >= 1");
            new_max_can_errors = v;
        }
        else
        {
            std::string joint_name, field;
            if (!parse_per_joint_param(name, joint_name, field))
                return fail(name + ": parameter is not dynamically reconfigurable");
            auto it_joint = joint_name_to_can_iface_and_idx_.find(joint_name);
            if (it_joint == joint_name_to_can_iface_and_idx_.end())
                return fail(name + ": unknown joint '" + joint_name + "'");

            auto &u = updates[joint_name];
            if (field == "vel_filter_type")
            {
                if (p.get_type() != rclcpp::ParameterType::PARAMETER_STRING)
                    return fail(name + ": must be string");
                const auto s = p.as_string();
                if (s == "none") u.vel_filter_type = VelFilterType::NONE;
                else if (s == "moving_average") u.vel_filter_type = VelFilterType::MOVING_AVERAGE;
                else if (s == "alpha_beta") u.vel_filter_type = VelFilterType::ALPHA_BETA;
                else return fail(name + ": must be 'none', 'moving_average' or 'alpha_beta'");
            }
            else if (field == "vel_filter_size")
            {
                if (p.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER)
                    return fail(name + ": must be int");
                auto v = p.as_int();
                if (v < 0)
                    return fail(name + ": must be >= 0");
                u.vel_filter_size = v;
            }
            else if (field == "pos_median_filter_size")
            {
                if (p.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER)
                    return fail(name + ": must be int");
                auto v = p.as_int();
                if (v < 0)
                    return fail(name + ": must be >= 0");
                u.pos_median_filter_size = v;
            }
            else if (field == "alpha")
            {
                if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
                    return fail(name + ": must be double");
                auto v = p.as_double();
                if (!(v > 0.0 && v <= 1.0))
                    return fail(name + ": must be in (0, 1]");
                u.alpha = v;
            }
            else if (field == "beta")
            {
                if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
                    return fail(name + ": must be double");
                auto v = p.as_double();
                if (v <= 0.0)
                    return fail(name + ": must be > 0");
                u.beta = v; // upper bound checked in pass 2 against effective alpha
            }
            else if (field == "tau_c" || field == "tau_s" || field == "v_s" || field == "k" || field == "k_a" || field == "b")
            {
                if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
                    return fail(name + ": must be double");
                const double v = p.as_double();
                if (field == "tau_c") u.tau_c = v;
                else if (field == "tau_s") u.tau_s = v;
                else if (field == "v_s") u.v_s = v;
                else if (field == "k") u.k = v;
                else if (field == "k_a") u.k_a = v;
                else u.b = v;
            }
            else if (field == "pos_limit_min")
            {
                if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
                    return fail(name + ": must be double");
                u.pos_limit_min = p.as_double();
            }
            else if (field == "pos_limit_max")
            {
                if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
                    return fail(name + ": must be double");
                u.pos_limit_max = p.as_double();
            }
            else
            {
                return fail(name + ": field '" + field + "' is not dynamically reconfigurable");
            }
        }
    }

    // Pass 2: pairwise validation against effective post-batch values (need current values)
    {
        std::shared_lock<WritePreferringSharedMutex> lock(joint_params_mutex_);
        for (const auto &[joint_name, u] : updates)
        {
            auto [iface, idx] = joint_name_to_can_iface_and_idx_.at(joint_name);
            const auto &jp = joint_parameters_per_can_interface_[iface][idx];
            const auto &ab = joint_ab_filters_per_can_interface_[iface][idx];

            if (u.beta.has_value())
            {
                double eff_alpha = u.alpha.value_or(ab.get_alpha());
                if (!(*u.beta < 2.0 * eff_alpha))
                {
                    return fail("joint_defintions." + joint_name + ".beta: must satisfy beta < 2 * alpha (effective alpha = " + std::to_string(eff_alpha) + ")");
                }
            }

            double eff_min = u.pos_limit_min.value_or(jp.pos_limit_min);
            double eff_max = u.pos_limit_max.value_or(jp.pos_limit_max);
            if (eff_min > eff_max)
            {
                return fail("joint_defintions." + joint_name + ": pos_limit_min must be <= pos_limit_max");
            }
        }
    }

    // Pass 3: apply under unique lock
    {
        std::unique_lock<WritePreferringSharedMutex> lock(joint_params_mutex_);

        if (new_default_damping_KD.has_value()) default_damping_KD_ = *new_default_damping_KD;
        if (new_damping_on_motor_error.has_value()) damping_on_motor_error_ = *new_damping_on_motor_error;
        if (new_max_can_errors.has_value()) max_can_errors_before_motor_shutdown_ = static_cast<unsigned int>(*new_max_can_errors);

        for (const auto &[joint_name, u] : updates)
        {
            auto [iface, idx] = joint_name_to_can_iface_and_idx_.at(joint_name);
            auto &jp = joint_parameters_per_can_interface_[iface][idx];
            auto &ab = joint_ab_filters_per_can_interface_[iface][idx];
            auto &ma = joint_vel_filters_per_can_interface_[iface][idx];
            auto &med = joint_pos_median_filters_per_can_interface_[iface][idx];

            if (u.vel_filter_type.has_value()) jp.vel_filter_type = *u.vel_filter_type;
            if (u.vel_filter_size.has_value())
            {
                jp.vel_filter_size = static_cast<unsigned int>(*u.vel_filter_size);
                ma.resize(jp.vel_filter_size);
            }
            if (u.pos_median_filter_size.has_value())
            {
                jp.pos_median_filter_size = static_cast<unsigned int>(*u.pos_median_filter_size);
                med.resize(jp.pos_median_filter_size);
            }
            if (u.alpha.has_value()) ab.set_alpha(*u.alpha);
            if (u.beta.has_value()) ab.set_beta(*u.beta);

            if (u.tau_c.has_value()) jp.friction_parameters.tau_c = *u.tau_c;
            if (u.tau_s.has_value()) jp.friction_parameters.tau_s = *u.tau_s;
            if (u.v_s.has_value())   jp.friction_parameters.v_s   = *u.v_s;
            if (u.k.has_value())     jp.friction_parameters.k     = *u.k;
            if (u.k_a.has_value())   jp.friction_parameters.k_a   = *u.k_a;
            if (u.b.has_value())     jp.friction_parameters.b     = *u.b;

            if (u.pos_limit_min.has_value()) jp.pos_limit_min = *u.pos_limit_min;
            if (u.pos_limit_max.has_value()) jp.pos_limit_max = *u.pos_limit_max;
        }
    }

    return result;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    sched_param sch;
    sch.sched_priority = 80;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sch) != 0)
    {
        RCLCPP_WARN(rclcpp::get_logger("PrioritySetter"), "Failed to set thread priority");
    }
    auto node = std::make_shared<CubeMarsHardwareNode>();
    // Each CAN interface's cycle callback blocks in send_and_receive while holding an executor
    // thread, so we need at least one thread per bus plus a few for publishing/watchdog/services.
    // Threads parked in recv cost no CPU, so over-provisioning is safe. Override via the
    // 'executor_threads' parameter (default 12; raise if you add buses).
    int num_threads = node->declare_parameter<int>("executor_threads", 12);
    if (num_threads < 2)
    {
        num_threads = 2;
    }
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), static_cast<size_t>(num_threads));
    RCLCPP_INFO(node->get_logger(), "Node is running on %li threads", executor.get_number_of_threads());
    executor.add_node(node->get_node_base_interface());
    executor.spin(); // Call back because of CTRL_C brings us over this
    node->on_shutdown(node->get_current_state());
    rclcpp::shutdown();
    return 0;
}
