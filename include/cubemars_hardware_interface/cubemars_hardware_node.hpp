#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "robot_control_msgs/msg/joint_command.hpp"
#include "robot_control_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "cubemars_hardware_interface/cubemars_can.hpp"
#include "cubemars_hardware_interface/custom_qos.hpp"
#include <mutex>
#include <semaphore>
#include <shared_mutex>
#include <optional>
#include <unordered_map>
#include <utility>
#include <sensor_msgs/msg/joint_state.hpp>
#include "std_srvs/srv/trigger.hpp"
#include "robot_control_msgs/srv/set_motor_origin_here.hpp"
#include "cubemars_hardware_interface/filters.hpp"

using namespace rclcpp_lifecycle::node_interfaces;

class CubeMarsHardwareNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    /**
     * Stribeck friction model with 6 parameters (tau_c, tau_s, v_s, k, k_a, b):
     * tau_f =  tau_c*tanh(k_a*v) + (tau_s - tau_c)*exp(-|v|/v_s)^k*tanh(k_a*v) + b*v
     */
    struct FrictionParameters
    {
        double tau_c; // Coulomb friction
        double tau_s; // Static friction
        double v_s;   // Stribeck Velocity
        double k;     // Exponential term
        double k_a;   // Steepness factor for tanh (instead of sign)
        double b;     // Viscous friction coefficient
    };

    enum class VelFilterType { NONE, MOVING_AVERAGE, ALPHA_BETA };

    struct JointParameters
    {
        double pos_limit_min;
        double pos_limit_max;
        double transmission_ratio;
        FrictionParameters friction_parameters;
        unsigned int vel_filter_size;
        VelFilterType vel_filter_type;
        unsigned int pos_median_filter_size;
        double zero_position;
        bool set_zero_position_on_startup;
        unsigned int msg_idx;
        std::string name;
    };

private:
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr joint_temp_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr can_interface_frequency_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr joint_rx_latency_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr unfiltered_velocity_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr unfiltered_position_pub_;
    rclcpp::Publisher<robot_control_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Subscription<robot_control_msgs::msg::JointCommand>::SharedPtr joint_cmd_sub_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    std::chrono::duration<double> frequency_;
    std::chrono::duration<double> watchdog_frequency_;

    bool damping_on_motor_error_;
    unsigned int max_can_errors_before_motor_shutdown_;

    bool msg_received_;


    robot_control_msgs::msg::JointCommand joint_cmd_msg_;
    robot_control_msgs::msg::JointState joint_state_msg_;
    std_msgs::msg::Float32MultiArray joint_temp_msg_;
    std_msgs::msg::Float32MultiArray can_interface_frequency_msg_;
    std_msgs::msg::Float32MultiArray joint_rx_latency_msg_;
    std_msgs::msg::Float32MultiArray unfiltered_velocity_msg_;
    std_msgs::msg::Float32MultiArray unfiltered_position_msg_;
    robot_control_msgs::msg::JointState joint_state_msg_to_pub_;
    std_msgs::msg::Float32MultiArray joint_temp_msg_to_pub_;
    std_msgs::msg::Float32MultiArray can_interface_frequency_msg_to_pub_;
    std_msgs::msg::Float32MultiArray joint_rx_latency_msg_to_pub_;
    std_msgs::msg::Float32MultiArray unfiltered_velocity_msg_to_pub_;
    std_msgs::msg::Float32MultiArray unfiltered_position_msg_to_pub_;
    std::shared_mutex joint_cmd_msg_mutex_;
    std::shared_mutex joint_state_msg_mutex_;
    std::shared_mutex can_communication_mutex_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr ros2_joint_state_pub_;
    sensor_msgs::msg::JointState ros2_joint_state_msg_;
    bool publish_ros2_joint_state_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr set_all_motors_origin_here_srv_;
    rclcpp::Service<robot_control_msgs::srv::SetMotorOriginHere>::SharedPtr set_motor_origin_here_srv_;

    double default_damping_KD_;
    double friction_compensation_sign_steepness_;
    unsigned int num_joints_;
    unsigned int joint_msg_length_;

    std::set<std::string> can_interfaces_names_; // This determines the order
    std::vector<std::vector<cubemars::joint_cmd_t>> joint_commands_per_can_interface_;
    std::vector<std::vector<cubemars::joint_state_t>> joint_states_per_can_interface_;
    std::vector<std::vector<JointParameters>> joint_parameters_per_can_interface_;
    std::vector<rclcpp::TimerBase::SharedPtr> can_cycle_timers_per_can_interface_;
    std::vector<unsigned int> num_can_errors_per_interfaces_;
    std::vector<std::shared_ptr<cubemars::CubemarsCan>> can_interfaces_;
    std::vector<rclcpp::Time> last_can_cycle_times_;
    std::vector<rclcpp::CallbackGroup::SharedPtr> can_cycle_callback_groups_;
    std::vector<std::vector<MovingAverage<double>>> joint_vel_filters_per_can_interface_;
    std::vector<std::vector<AlphaBetaFilter<double>>> joint_ab_filters_per_can_interface_;
    std::vector<std::vector<MedianFilter<double>>> joint_pos_median_filters_per_can_interface_;
    std::vector<std::vector<int64_t>> last_joint_rx_ns_per_can_interface_;

    // Guards joint_parameters_per_can_interface_ and the filter vectors against
    // concurrent reads by can_cycle_callback while the parameter callback applies updates.
    std::shared_mutex joint_params_mutex_;
    std::unordered_map<std::string, std::pair<unsigned int, unsigned int>> joint_name_to_can_iface_and_idx_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_handle_;

    rcl_interfaces::msg::SetParametersResult on_set_parameters_callback(const std::vector<rclcpp::Parameter> &params);
    bool parse_per_joint_param(const std::string &name, std::string &joint_name_out, std::string &field_out) const;

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
    void set_all_motors_origin_here_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                             std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void set_motor_origin_here_callback(const std::shared_ptr<robot_control_msgs::srv::SetMotorOriginHere::Request> request,
                                        std::shared_ptr<robot_control_msgs::srv::SetMotorOriginHere::Response> response);
};