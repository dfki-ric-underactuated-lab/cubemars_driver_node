#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "robot_control_msgs/msg/joint_command.hpp"
#include "robot_control_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "cubemars_hardware_interface/cubemars_can.hpp"
#include "cubemars_hardware_interface/custom_qos.hpp"
#include <mutex>
#include <semaphore>
#include <shared_mutex>
#include <sensor_msgs/msg/joint_state.hpp>

using namespace rclcpp_lifecycle::node_interfaces;

class CubeMarsHardwareNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    struct friction_parameters
    {
        double b;
        double cf;
    };

private:
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr joint_temp_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr can_interface_frequency_pub_;
    rclcpp::Publisher<robot_control_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Subscription<robot_control_msgs::msg::JointCommand>::SharedPtr joint_cmd_sub_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    std::chrono::duration<double> frequency_;
    std::chrono::duration<double> watchdog_frequency_;

    bool msg_received_;
    robot_control_msgs::msg::JointCommand joint_cmd_msg_;
    robot_control_msgs::msg::JointState joint_state_msg_;
    std_msgs::msg::Float32MultiArray joint_temp_msg_;
    std_msgs::msg::Float32MultiArray can_interface_frequency_msg_;
    robot_control_msgs::msg::JointState joint_state_msg_to_pub_;
    std_msgs::msg::Float32MultiArray joint_temp_msg_to_pub_;
    std_msgs::msg::Float32MultiArray can_interface_frequency_msg_to_pub_;
    std::shared_mutex joint_cmd_msg_mutex_;
    std::shared_mutex joint_state_msg_mutex_;
    std::shared_mutex can_communication_mutex_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr ros2_joint_state_pub_;
    sensor_msgs::msg::JointState ros2_joint_state_msg_;
    bool publish_ros2_joint_state_;
    std::vector<double> joint_zero_positions_;
    

    double default_damping_KD_;
    double friction_compensation_sign_steepness_;
    unsigned int num_joints_;

    std::set<std::string> can_interfaces_names_; // This determines the order

    std::vector<std::vector<cubemars::joint_cmd_t>> joint_commands_per_can_interface_;
    std::vector<std::vector<cubemars::joint_state_t>> joint_states_per_can_interface_;
    std::vector<std::vector<unsigned int>> msg_idxs_per_can_interface_;
    std::vector<std::vector<friction_parameters>> friction_parameters_per_can_interface_;
    std::vector<std::vector<double>> transmission_ratios_per_can_interface_;
    std::vector<rclcpp::TimerBase::SharedPtr> can_cycle_timers_per_can_interface_;
    std::vector<std::shared_ptr<cubemars::CubemarsCan>> can_interfaces_;
    std::vector<rclcpp::Time> last_can_cycle_times_;
    std::vector<std::vector<std::string>> joint_names_per_can_interface_;
    std::vector<rclcpp::CallbackGroup::SharedPtr> can_cycle_callback_groups_;

    template <typename T>
    T declare_and_get_parameter(const std::string &name)
    {
        if (!this->has_parameter(name)) // To prevent exceptions due to double declaration
        {
            this->declare_parameter<T>(name);
        }   
        this->get_parameter(name).get_value<T>();
    }

    void declare_parameter_if_undeclared(const std::string &name, const rclcpp::ParameterType & type){
        if(!this->has_parameter(name)){
            this->declare_parameter(name, type);
        }
    }

    template <typename T>
    void declare_parameter_if_undeclared(const std::string &name, const T & value){
        if(!this->has_parameter(name)){
            this->declare_parameter(name, value);
        }
    }

public:
    CubeMarsHardwareNode();

    LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
    LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
    LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
    LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;
    LifecycleNodeInterface::CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state) override;
    LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override;

    void watchdog_timer_callback();
    void joint_cmd_msg_callback(const robot_control_msgs::msg::JointCommand &joint_cmd_msg);
    void joint_state_publish_callback();
    void can_cycle_callback(unsigned int can_interface_idx);
};