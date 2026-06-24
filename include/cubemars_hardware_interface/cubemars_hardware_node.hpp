#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "robot_control_msgs/msg/joint_command.hpp"
#include "robot_control_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"
#include "cubemars_hardware_interface/cubemars_can.hpp"
#include "cubemars_hardware_interface/custom_qos.hpp"
#include <atomic>
#include <mutex>
#include <pthread.h>
#include <semaphore>
#include <shared_mutex>
#include <thread>
#include <optional>
#include <unordered_map>
#include <utility>
#include <sensor_msgs/msg/joint_state.hpp>
#include "std_srvs/srv/trigger.hpp"
#include "robot_control_msgs/srv/set_motor_origin_here.hpp"
#include "cubemars_hardware_interface/filters.hpp"

using namespace rclcpp_lifecycle::node_interfaces;

// std::shared_mutex on Linux/glibc defaults to reader-preference; with constant
// 1 kHz x N readers in can_cycle_callback, a writer (parameter callback) can
// starve indefinitely. This wrapper exposes the SharedMutex concept backed by a
// pthread_rwlock_t configured with writer preference, so std::shared_lock /
// std::unique_lock still work as drop-in.
class WritePreferringSharedMutex
{
public:
    WritePreferringSharedMutex()
    {
        pthread_rwlockattr_t attr;
        pthread_rwlockattr_init(&attr);
        pthread_rwlockattr_setkind_np(&attr, PTHREAD_RWLOCK_PREFER_WRITER_NONRECURSIVE_NP);
        pthread_rwlock_init(&lock_, &attr);
        pthread_rwlockattr_destroy(&attr);
    }
    ~WritePreferringSharedMutex() { pthread_rwlock_destroy(&lock_); }
    WritePreferringSharedMutex(const WritePreferringSharedMutex &) = delete;
    WritePreferringSharedMutex &operator=(const WritePreferringSharedMutex &) = delete;

    void lock() { pthread_rwlock_wrlock(&lock_); }
    bool try_lock() { return pthread_rwlock_trywrlock(&lock_) == 0; }
    void unlock() { pthread_rwlock_unlock(&lock_); }
    void lock_shared() { pthread_rwlock_rdlock(&lock_); }
    bool try_lock_shared() { return pthread_rwlock_tryrdlock(&lock_) == 0; }
    void unlock_shared() { pthread_rwlock_unlock(&lock_); }

private:
    pthread_rwlock_t lock_;
};

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
    // Per-CAN-interface: time to write all command frames into the TX buffer, in us.
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr can_tx_fill_duration_pub_;
    // Per-CAN-interface: time to receive all replies after the TX buffer was filled, in us.
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr can_rx_duration_pub_;
    // Per-joint: time from the TX buffer being filled to that motor's reply arriving in node space, in us.
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr joint_reply_after_tx_pub_;
    // Per-joint freshness of ~/joint_states at publish time: now - last successful reply RX timestamp, in us.
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr joint_state_age_pub_;
    // Per-joint latency from ROS command receipt to the command frame going on the wire (software TX timestamp), in us.
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr joint_cmd_to_bus_latency_pub_;
    // Per-joint motor turnaround: reply RX timestamp - command TX timestamp (time from frame on the wire to reply), in us.
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr joint_motor_reply_latency_pub_;
    // Per-joint kernel RX-delivery offset: software RX timestamp - hardware (card) RX timestamp, in us. The
    // absolute value is a constant card-clock-vs-CLOCK_REALTIME offset; its VARIATION/spikes are the time the
    // kernel took to deliver a reply the card already had (softirq/IRQ delay) vs. a genuinely late reply.
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr joint_rx_delivery_pub_;
    // Per-joint card hardware-clock RX timestamp, baseline-subtracted (us since this joint's first reply,
    // so it fits float32). Diff consecutive values for the card's RX inter-arrival timing.
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr joint_rx_hw_timestamp_pub_;
    // Per-joint TX-path latency: command TX timestamp - write() enqueue timestamp (time from buffer to wire), in us.
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr joint_enqueue_to_wire_pub_;
    // Per-CAN-interface: cycle wall time minus send and receive (the prologue + filters + write-back + diagnostics), in us.
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr can_processing_pub_;
    // Per-CAN-interface: gap from the end of one can_cycle_callback to the start of the next (loop/scheduling overhead), in us.
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr can_intercycle_gap_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr unfiltered_velocity_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr unfiltered_position_pub_;
    rclcpp::Publisher<robot_control_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    // Round-trip controller latency: now - stamp of the incoming joint_cmd, in milliseconds.
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr controller_latency_pub_;
    rclcpp::Subscription<robot_control_msgs::msg::JointCommand>::SharedPtr joint_cmd_sub_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    std::chrono::duration<double> frequency_;
    std::chrono::duration<double> watchdog_frequency_;

    bool damping_on_motor_error_;
    unsigned int max_can_errors_before_motor_shutdown_;
    bool enable_tx_timestamping_; // software TX timestamps + the cmd_to_bus / motor_reply latency topics
    bool enable_can_error_frames_; // deliver + log CAN bus-error frames (bus-off, ACK errors, ...)

    bool msg_received_;


    // Latest joint command, published lock-free by the subscriber and read by the comm threads.
    // The message is never mutated in place; each update stores a fresh immutable copy and the
    // atomic pointer swap is the only synchronization (no mutex, no blocking, no priority inversion).
    std::atomic<std::shared_ptr<const robot_control_msgs::msg::JointCommand>> joint_cmd_ptr_;
    robot_control_msgs::msg::JointState joint_state_msg_;
    std_msgs::msg::Float32MultiArray joint_temp_msg_;
    std_msgs::msg::Float32MultiArray can_interface_frequency_msg_;
    std_msgs::msg::Float32MultiArray can_tx_fill_duration_msg_;
    std_msgs::msg::Float32MultiArray can_rx_duration_msg_;
    std_msgs::msg::Float32MultiArray can_processing_msg_;
    std_msgs::msg::Float32MultiArray can_intercycle_gap_msg_;
    std_msgs::msg::Float32MultiArray joint_reply_after_tx_msg_;
    std_msgs::msg::Float32MultiArray joint_enqueue_to_wire_msg_;
    std_msgs::msg::Float32MultiArray joint_state_age_msg_;
    std_msgs::msg::Float32MultiArray joint_cmd_to_bus_latency_msg_;
    std_msgs::msg::Float32MultiArray joint_motor_reply_latency_msg_;
    std_msgs::msg::Float32MultiArray joint_rx_delivery_msg_;
    std_msgs::msg::Float32MultiArray joint_rx_hw_timestamp_msg_;
    std_msgs::msg::Float32MultiArray unfiltered_velocity_msg_;
    std_msgs::msg::Float32MultiArray unfiltered_position_msg_;
    robot_control_msgs::msg::JointState joint_state_msg_to_pub_;
    std_msgs::msg::Float32MultiArray joint_temp_msg_to_pub_;
    std_msgs::msg::Float32MultiArray can_interface_frequency_msg_to_pub_;
    std_msgs::msg::Float32MultiArray can_tx_fill_duration_msg_to_pub_;
    std_msgs::msg::Float32MultiArray can_rx_duration_msg_to_pub_;
    std_msgs::msg::Float32MultiArray can_processing_msg_to_pub_;
    std_msgs::msg::Float32MultiArray can_intercycle_gap_msg_to_pub_;
    std_msgs::msg::Float32MultiArray joint_reply_after_tx_msg_to_pub_;
    std_msgs::msg::Float32MultiArray joint_enqueue_to_wire_msg_to_pub_;
    std_msgs::msg::Float32MultiArray joint_cmd_to_bus_latency_msg_to_pub_;
    std_msgs::msg::Float32MultiArray joint_motor_reply_latency_msg_to_pub_;
    std_msgs::msg::Float32MultiArray joint_rx_delivery_msg_to_pub_;
    std_msgs::msg::Float32MultiArray joint_rx_hw_timestamp_msg_to_pub_;
    std_msgs::msg::Float32MultiArray unfiltered_velocity_msg_to_pub_;
    std_msgs::msg::Float32MultiArray unfiltered_position_msg_to_pub_;
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
    std::vector<unsigned int> num_can_errors_per_interfaces_;
    std::vector<std::shared_ptr<cubemars::CubemarsCan>> can_interfaces_;
    std::vector<rclcpp::Time> last_can_cycle_times_;
    std::vector<int64_t> last_cycle_end_ns_per_can_interface_; // CLOCK_MONOTONIC end of the previous can_cycle_callback, for the inter-cycle gap

    // One dedicated thread per CAN interface running the send/receive cycle back-to-back,
    // instead of a ROS timer dispatched by the executor (removes per-cycle dispatch latency).
    struct CommThread
    {
        std::thread thread;
        std::atomic<bool> running{false};
    };
    std::vector<std::unique_ptr<CommThread>> comm_threads_; // unique_ptr: std::atomic is not movable
    // Set by a comm thread on a fatal error in INACTIVE state; the supervisor timer (executor
    // thread) performs the actual cleanup() so the comm thread never joins itself.
    std::atomic<bool> cleanup_requested_{false};
    rclcpp::TimerBase::SharedPtr supervisor_timer_;
    std::vector<std::vector<MovingAverage<double>>> joint_vel_filters_per_can_interface_;
    std::vector<std::vector<AlphaBetaFilter<double>>> joint_ab_filters_per_can_interface_;
    std::vector<std::vector<MedianFilter<double>>> joint_pos_median_filters_per_can_interface_;
    std::vector<std::vector<int64_t>> last_joint_rx_ns_per_can_interface_;
    // Latest successful kernel RX timestamp (CLOCK_REALTIME ns) per msg_idx, used to
    // stamp the aggregated joint_state_msg_. Guarded by joint_state_msg_mutex_.
    std::vector<int64_t> joint_rx_ns_;
    // Per-msg_idx running minimum of (software_rx - hardware_rx) ns. This is the "no kernel delivery
    // delay" floor of the (otherwise huge, constant) card-clock-vs-CLOCK_REALTIME offset; subtracting
    // it before the float cast keeps joint_rx_delivery_offsets_us near zero so its spikes are visible.
    std::vector<int64_t> joint_rx_delivery_floor_ns_;
    // Per-msg_idx first hardware-clock RX timestamp (ns), used as the baseline for joint_rx_hw_timestamps_us
    // (0 = not yet set).
    std::vector<int64_t> joint_rx_hw_base_ns_;
    // CLOCK_REALTIME ns the most recently stored joint command was received on the ROS subscriber.
    // Written by joint_cmd_msg_callback, read by can_cycle_callback (different callback groups).
    std::atomic<int64_t> cmd_rx_ns_{0};

    // Guards joint_parameters_per_can_interface_ and the filter vectors against
    // concurrent reads by can_cycle_callback while the parameter callback applies updates.
    // Writer-preferring to prevent the param callback from starving against the
    // constant high-frequency readers in can_cycle_callback.
    WritePreferringSharedMutex joint_params_mutex_;
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
    void joint_cmd_msg_callback(const robot_control_msgs::msg::JointCommand::ConstSharedPtr &joint_cmd_msg);
    void joint_state_publish_callback();
    void can_cycle_callback(unsigned int can_interface_idx);
    void comm_loop(unsigned int can_interface_idx);
    void start_comm_thread(unsigned int can_interface_idx);
    void stop_comm_thread(unsigned int can_interface_idx);
    void stop_all_comm_threads();
    void supervisor_callback();
    void set_all_motors_origin_here_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                             std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void set_motor_origin_here_callback(const std::shared_ptr<robot_control_msgs::srv::SetMotorOriginHere::Request> request,
                                        std::shared_ptr<robot_control_msgs::srv::SetMotorOriginHere::Response> response);
};