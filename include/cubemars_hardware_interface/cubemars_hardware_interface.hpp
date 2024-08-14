#ifndef CUBEMARS_HARDWARE_INTERFACE_HPP_
#define CUBEMARS_HARDWARE_INTERFACE_HPP_

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/rclcpp.hpp"

#include "cubemars_com.hpp"

#define CHECK_SC(call, error_msg)                                \
    do                                                           \
    {                                                            \
        ret_val = (call);                                        \
        if (ret_val < 0)                                         \
        {                                                        \
            RCLCPP_FATAL(                                        \
                rclcpp::get_logger("CubemarsHardwareInterface"), \
                "'%s': '%s'", error_msg, strerror(errno));       \
            return hardware_interface::CallbackReturn::ERROR;    \
        }                                                        \
    } while (0)

namespace cubemars_hardware_interface
{
    class CubemarsHardwareInterface : public hardware_interface::SystemInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(CubemarsHardwareInterface)

        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareInfo &info) override;

        hardware_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_cleanup(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::return_type read(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        hardware_interface::return_type write(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        hardware_interface::return_type prepare_command_mode_switch(
            const std::vector<std::string> &start_interfaces,
            const std::vector<std::string> &stop_interfaces) override;

    private:
        std::string can_interface_;
        int can_socket_fd_;
        int enable_loopback_;

        hardware_interface::return_type write_to_can(can_frame frame);
        hardware_interface::return_type start_motor_control_mode(int can_id);
        hardware_interface::return_type exit_motor_control_mode(int can_id);
        hardware_interface::return_type set_zero_position(int can_id);

        std::vector<cubemars::joint_config_t> hw_joint_configs_;

        std::vector<double> hw_commands_position_;
        std::vector<double> hw_commands_velocity_;
        std::vector<double> hw_commands_effort_;

        std::vector<double> hw_states_position_;
        std::vector<double> hw_states_velocity_;
        std::vector<double> hw_states_effort_;

        std::vector<double> hw_states_temperature_;
        std::vector<cubemars::JointMode> hw_control_level_;

        void pack_cmd(
            can_frame *frame, 
            float p_des, 
            float v_des, 
            float t_ff, 
            cubemars::joint_config_t joint_config, 
            cubemars::JointMode control_mode);

        int float_to_uint(float x, float x_min, float x_max, unsigned int bits);
        cubemars::ErrorCode unpack_reply(can_frame msg);
        float uint_to_float(int x_int, float x_min, float x_max, int bits);
    };
} // namespace

#endif // CUBEMARS_HARDWARE_INTERFACE_HPP_