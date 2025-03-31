#include "cubemars_hardware_interface/cubemars_hardware_node.hpp"

CubeMarsHardwareNode::CubeMarsHardwareNode() : rclcpp_lifecycle::LifecycleNode("cubermars_hardware_node")
{
}

LifecycleNodeInterface::CallbackReturn CubeMarsHardwareNode::on_configure([[maybe_unused]] const rclcpp_lifecycle::State &previous_state)
{
    /**Declare and read parameters */
    this->declare_parameter_if_undeclared("joints", rclcpp::PARAMETER_STRING_ARRAY);
    this->declare_parameter_if_undeclared("default_damping_KD", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter_if_undeclared("enable_loopback", true);
    this->declare_parameter_if_undeclared("can_socket_timeout_usec", 1000);
    this->declare_parameter_if_undeclared("frequency", 1000);
    this->declare_parameter_if_undeclared("watchdog_frequency", 100);
    this->declare_parameter_if_undeclared("friction_compensation_sign_steepness", 100.);

    std::set<std::string> can_interfaces_names_;
    std::unordered_map<std::string, std::set<int>> can_id_per_interface;
    std::set<int> msg_idxs;
    std::set<std::string> motor_types;
    std::vector<std::vector<cubemars::joint_config_t>> joint_configs_per_can_interface;
    try
    {
        auto joint_names = this->get_parameter("joints").as_string_array();
        default_damping_KD_ = this->get_parameter("default_damping_KD").as_double();
        friction_compensation_sign_steepness_ = this->get_parameter("friction_compensation_sign_steepness").as_double();
        frequency_ = std::chrono::duration<double>(1.0 / this->get_parameter("frequency").as_int());
        watchdog_frequency_ = std::chrono::duration<double>(1.0 / this->get_parameter("watchdog_frequency").as_int());

        // Publishers //TODO: add option to disabke 'debug topics'
        joint_state_pub_ = this->create_publisher<robot_control_msgs::msg::JointState>("joint_states", QOS_BEST_EFFORT_NO_DEPTH);
        joint_temp_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("joint_temperatures", QOS_BEST_EFFORT_NO_DEPTH);
        can_interface_frequency_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("can_cycle_frequencies", QOS_BEST_EFFORT_NO_DEPTH);

        // For each joint create default parameters and  validat them them
        for (unsigned int i = 0; i < joint_names.size(); i++)
        {
            // Declare joint definitions
            this->declare_parameter_if_undeclared("joint_defintions." + joint_names[i] + ".msg_idx", static_cast<int>(i));
            this->declare_parameter_if_undeclared("joint_defintions." + joint_names[i] + ".can_interface", rclcpp::PARAMETER_STRING);
            this->declare_parameter_if_undeclared("joint_defintions." + joint_names[i] + ".can_id", rclcpp::PARAMETER_INTEGER);
            this->declare_parameter_if_undeclared("joint_defintions." + joint_names[i] + ".motor_type", rclcpp::PARAMETER_STRING);
            this->declare_parameter_if_undeclared("joint_defintions." + joint_names[i] + ".invert", false);
            this->declare_parameter_if_undeclared("joint_defintions." + joint_names[i] + ".b", 0.0);
            this->declare_parameter_if_undeclared("joint_defintions." + joint_names[i] + ".cf", 0.0);
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
            auto can_id = this->get_parameter("joint_defintions." + joint_names[i] + ".can_id").as_int();
            if (!can_id_per_interface[can_interface_name].insert(can_id).second)
            {
                RCLCPP_ERROR(this->get_logger(), "Joint %s has can_id %li which is already used can interface %s", joint_names[i].c_str(), can_id, can_interface_name.c_str());
                return LifecycleNodeInterface::CallbackReturn::FAILURE;
            }
            auto motor_type = this->get_parameter("joint_defintions." + joint_names[i] + ".motor_type").as_string();
            motor_types.insert(motor_type);
        }
        num_joints_ = joint_names.size();
        // Create joint configs per can interfaces and create state and message mapping
        unsigned int num_can_interfaces = can_interfaces_names_.size();
        joint_configs_per_can_interface.resize(num_can_interfaces);
        joint_commands_per_can_interface_.resize(num_can_interfaces);
        joint_states_per_can_interface_.resize(num_can_interfaces);
        msg_idxs_per_can_interface_.resize(num_can_interfaces);
        joint_commands_per_can_interface_.resize(num_can_interfaces);
        friction_parameters_per_can_interface_.resize(num_can_interfaces);
        can_cycle_timers_per_can_interface_.resize(num_can_interfaces);
        joint_names_per_can_interface_.resize(num_can_interfaces);
        can_interfaces_.resize(num_can_interfaces);
        last_can_cycle_times_.resize(num_can_interfaces, this->get_clock()->now());
        for (unsigned int i = 0; i < num_can_interfaces; i++)
        {
            can_cycle_callback_groups_.push_back(this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive));
        }
        can_interface_frequency_msg_.data.resize(num_can_interfaces, 0.0);

        // Prepare messages
        joint_cmd_msg_.effort.resize(num_joints_, 0.0);
        joint_cmd_msg_.velocity.resize(num_joints_, 0.0);
        joint_cmd_msg_.position.resize(num_joints_, 0.0);
        joint_cmd_msg_.kd.resize(num_joints_, 0.0);
        joint_cmd_msg_.kp.resize(num_joints_, 0.0);
        joint_state_msg_.position.resize(num_joints_, 0.0);
        joint_state_msg_.velocity.resize(num_joints_, 0.0);
        joint_state_msg_.effort.resize(num_joints_, 0.0);
        joint_temp_msg_.data.resize(num_joints_, 0.0);

        for (unsigned int i = 0; i < joint_names.size(); i++)
        {
            // Declare joint definitions
            auto joint_config = cubemars::joint_config_per_motor_type.at(this->get_parameter("joint_defintions." + joint_names[i] + ".motor_type").as_string());
            joint_config.can_id = this->get_parameter("joint_defintions." + joint_names[i] + ".can_id").as_int();
            joint_config.invert = this->get_parameter("joint_defintions." + joint_names[i] + ".invert").as_bool();
            auto msg_idx = this->get_parameter("joint_defintions." + joint_names[i] + ".msg_idx").as_int();
            auto can_interface = this->get_parameter("joint_defintions." + joint_names[i] + ".can_interface").as_string();
            auto can_interface_id = std::distance(can_interfaces_names_.begin(), can_interfaces_names_.find(can_interface));
            joint_configs_per_can_interface[can_interface_id].push_back(joint_config);
            joint_commands_per_can_interface_[can_interface_id].push_back({0, 0, 0, 0, 0});
            joint_states_per_can_interface_[can_interface_id].push_back({0, 0, 0, 0, cubemars::ErrorCode::FAULT_CODE_NONE});
            msg_idxs_per_can_interface_[can_interface_id].push_back(msg_idx);
            friction_parameters_per_can_interface_[can_interface_id].push_back({this->get_parameter("joint_defintions." + joint_names[i] + ".b").as_double(),
                                                                                this->get_parameter("joint_defintions." + joint_names[i] + ".cf").as_double()});
            joint_names_per_can_interface_[can_interface_id].push_back(joint_names[i]);
        }

        // Now create can devices and callback
        for (auto can_interface_name : can_interfaces_names_)
        {
            auto can_interface_id = std::distance(can_interfaces_names_.begin(), can_interfaces_names_.find(can_interface_name));
            can_interfaces_[can_interface_id] = std::make_shared<cubemars::CubemarsCan>(
                can_interface_name,
                this->get_parameter("enable_loopback").as_bool(),
                joint_configs_per_can_interface[can_interface_id],
                this->get_parameter("can_socket_timeout_usec").as_int(),
                true);
        }

        // Goes to default callback group
        publish_timer_ = this->create_timer(frequency_, std::bind(&CubeMarsHardwareNode::joint_state_publish_callback, this));
        // Create subscriber
        joint_cmd_sub_ = this->create_subscription<robot_control_msgs::msg::JointCommand>("joint_commands", QOS_BEST_EFFORT_NO_DEPTH, std::bind(&CubeMarsHardwareNode::joint_cmd_msg_callback, this, std::placeholders::_1));

        // Create timers that will enable all control cycles:
        // Now create can devices and callback
        for (unsigned int i = 0; i < can_interfaces_.size(); i++)
        {
            // Enable all motors
            try
            {
                can_interfaces_[i]->start_motor_control_mode();
            }
            catch (const std::exception &e)
            {
                // Notidy users
                RCLCPP_ERROR(this->get_logger(), "Device error on CAN interface %s occured: %s", can_interfaces_[i]->GetName().c_str(), e.what());
                // This can only happen when actual motors are enabled, hence try to disable motors
                try
                {
                    for (unsigned int i_o = 0; i_o <= i; i_o++)
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
                return LifecycleNodeInterface::CallbackReturn::ERROR;
            }
        }
        for (unsigned int i = 0; i < can_interfaces_.size(); i++)
        {
            // Create all Can cyle timer
            can_cycle_timers_per_can_interface_[i] = this->create_timer(frequency_, [i, this]()
                                                                        { this->can_cycle_callback(i); }, can_cycle_callback_groups_[i]);
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
    // Stop all times
    publish_timer_.reset();
    can_cycle_timers_per_can_interface_.clear();
    can_cycle_callback_groups_.clear();

    joint_cmd_msg_mutex_.lock(); // Nobody can enter
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
                RCLCPP_ERROR(this->get_logger(), "Error when stopping motor %s on can interface %s (joint with msg_idx %i): %s", joint_names_per_can_interface_[i][j].c_str(), can_interfaces_[i]->GetName().c_str(), msg_idxs_per_can_interface_[i][j], e.what());
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

    friction_parameters_per_can_interface_.clear();
    msg_idxs_per_can_interface_.clear();
    joint_states_per_can_interface_.clear();
    joint_commands_per_can_interface_.clear();
    can_interfaces_names_.clear();
    joint_cmd_sub_.reset();
    joint_state_pub_.reset();
    joint_temp_pub_.reset();
    can_interface_frequency_pub_.reset();
    last_can_cycle_times_.clear();
    joint_cmd_msg_mutex_.unlock();
    joint_state_msg_mutex_.unlock();
    joint_cmd_msg_.effort.clear();
    joint_cmd_msg_.velocity.clear();
    joint_cmd_msg_.position.clear();
    joint_cmd_msg_.kd.clear();
    joint_cmd_msg_.kp.clear();
    joint_state_msg_.position.clear();
    joint_state_msg_.velocity.clear();
    joint_state_msg_.effort.clear();
    joint_temp_msg_.data.clear();
    joint_names_per_can_interface_.clear();
    if (success)
    {
        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
    else
    {
        return LifecycleNodeInterface::CallbackReturn::ERROR;
    }
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
    // The motors have to be send into damping
    joint_cmd_msg_mutex_.lock();
    for (unsigned int i = 0; i < joint_cmd_msg_.position.size(); i++)
    {
        joint_cmd_msg_.effort[i] = 0.0;
        joint_cmd_msg_.velocity[i] = 0.0;
        joint_cmd_msg_.position[i] = 0.0;
        joint_cmd_msg_.kp[i] = 0.0;
        joint_cmd_msg_.kd[i] = default_damping_KD_;
    }
    joint_cmd_msg_mutex_.unlock();
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
        joint_cmd_sub_.reset();
        publish_timer_.reset();
        joint_state_pub_.reset();
        joint_temp_pub_.reset();
        can_interface_frequency_pub_.reset();
        can_interfaces_names_.clear();
        can_cycle_callback_groups_.clear();
        joint_cmd_msg_.effort.clear();
        joint_cmd_msg_.velocity.clear();
        joint_cmd_msg_.position.clear();
        joint_cmd_msg_.kd.clear();
        joint_cmd_msg_.kp.clear();
        joint_state_msg_.position.clear();
        joint_state_msg_.velocity.clear();
        joint_state_msg_.effort.clear();
        joint_temp_msg_.data.clear();
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
        break;
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

void CubeMarsHardwareNode::joint_cmd_msg_callback(const robot_control_msgs::msg::JointCommand &joint_cmd_msg)
{

    if (joint_cmd_msg.position.size() != num_joints_ ||
        joint_cmd_msg.velocity.size() != num_joints_ ||
        joint_cmd_msg.effort.size() != num_joints_ ||
        joint_cmd_msg.kp.size() != num_joints_ ||
        joint_cmd_msg.kd.size() != num_joints_)
    {
        RCLCPP_ERROR(this->get_logger(), "Received joint_cmd_msg with array sizes unequal to %i, skipping this message", num_joints_);
        return;
    }

    msg_received_ = true;
    // Only accept message if active
    if (this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
        joint_cmd_msg_mutex_.lock();
        joint_cmd_msg_ = joint_cmd_msg;
        joint_cmd_msg_mutex_.unlock();
    }
}

void CubeMarsHardwareNode::joint_state_publish_callback()
{
    joint_state_msg_mutex_.lock();
    joint_state_msg_to_pub_ = joint_state_msg_;
    joint_temp_msg_to_pub_ = joint_temp_msg_;
    can_interface_frequency_msg_to_pub_ = can_interface_frequency_msg_;
    joint_state_msg_mutex_.unlock();
    joint_state_pub_->publish(joint_state_msg_to_pub_); // TODO: check if this blocks and maybe avoid block during mutex
    joint_temp_pub_->publish(joint_temp_msg_to_pub_);
    can_interface_frequency_pub_->publish(can_interface_frequency_msg_to_pub_);
}

void CubeMarsHardwareNode::can_cycle_callback(unsigned int can_interface_idx)
{

    auto &joint_cmds = joint_commands_per_can_interface_[can_interface_idx];
    auto &joint_states = joint_states_per_can_interface_[can_interface_idx];
    auto &msg_idxs = msg_idxs_per_can_interface_[can_interface_idx];
    auto &friction_parameters = friction_parameters_per_can_interface_[can_interface_idx];

    // Calculate timings
    auto current_time = this->get_clock()->now();
    double can_cyle_frequency = (1000000000. / (current_time - last_can_cycle_times_[can_interface_idx]).nanoseconds());
    last_can_cycle_times_[can_interface_idx] = current_time;

    // Collect data
    joint_cmd_msg_mutex_.lock_shared();
    if (can_cycle_timers_per_can_interface_.size() == 0)
    {
        joint_cmd_msg_mutex_.unlock_shared();
        return; // If unconfigured in the meantime
    }
    for (unsigned int i = 0; i < joint_cmds.size(); i++)
    {
        joint_cmds[i].kd = joint_cmd_msg_.kd[msg_idxs[i]];
        joint_cmds[i].kp = joint_cmd_msg_.kp[msg_idxs[i]];
        joint_cmds[i].pos = joint_cmd_msg_.position[msg_idxs[i]];
        joint_cmds[i].vel = joint_cmd_msg_.velocity[msg_idxs[i]];
        joint_cmds[i].torque = joint_cmd_msg_.effort[msg_idxs[i]];
    }
    joint_cmd_msg_mutex_.unlock_shared();

    // Calculate torque compensation
    if (this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
        for (unsigned int i = 0; i < joint_cmds.size(); i++)
        {
            joint_cmds[i].torque += friction_parameters[i].b * joint_states[i].vel + friction_parameters[i].cf * atan(friction_compensation_sign_steepness_ * joint_states[i].vel);
        }
    }
    // Send an receive
    try
    {
        can_interfaces_[can_interface_idx]->send_and_receive(joint_cmds, joint_states);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "%s", e.what());
        RCLCPP_ERROR(this->get_logger(), "For safety reasons deactivating into damping");
        deactivate();
        RCLCPP_ERROR(this->get_logger(), "For safety reasons disable motors into uncofnigured");
        cleanup();
    }
    // Check status
    for (unsigned int i = 0; i < joint_cmds.size(); i++)
    {
        if (joint_states[i].status != cubemars::ErrorCode::FAULT_CODE_NONE)
        {
            RCLCPP_ERROR(this->get_logger(), "Joint %s on can_interface %s (msg idx %i) has error %s - deactivating joints", joint_names_per_can_interface_[can_interface_idx][i].c_str(), can_interfaces_[can_interface_idx]->GetName().c_str(), msg_idxs[can_interface_idx], cubemars::errorFlagToString(joint_states[i].status));
            deactivate();
        }
    }

    // Write back state
    joint_state_msg_mutex_.lock_shared();
    if (can_cycle_timers_per_can_interface_.size() == 0)
    {
        joint_state_msg_mutex_.unlock_shared();
        return; // If unconfigured in the meantime
    }
    for (unsigned int i = 0; i < joint_cmds.size(); i++)
    {
        joint_state_msg_.position[msg_idxs[i]] = joint_states[i].pos;
        joint_state_msg_.velocity[msg_idxs[i]] = joint_states[i].vel;
        joint_state_msg_.effort[msg_idxs[i]] = joint_states[i].torque;
        joint_temp_msg_.data[msg_idxs[i]] = joint_states[i].temp;
    }
    can_interface_frequency_msg_.data[can_interface_idx] = can_cyle_frequency;
    joint_state_msg_mutex_.unlock_shared();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    sched_param sch;
    sch.sched_priority = 80;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sch) != 0) {
        RCLCPP_WARN(rclcpp::get_logger("PrioritySetter"), "Failed to set thread priority");
    }
    auto node = std::make_shared<CubeMarsHardwareNode>();
    rclcpp::executors::MultiThreadedExecutor executor; // TODO: specify more threads
    RCLCPP_INFO(node->get_logger(), "Node is running on %li threads", executor.get_number_of_threads());
    executor.add_node(node->get_node_base_interface());
    executor.spin(); // Call back because of CTRL_C brings us over this
    node->on_shutdown(node->get_current_state());
    rclcpp::shutdown();
    return 0;
}
