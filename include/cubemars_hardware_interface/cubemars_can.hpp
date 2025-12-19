#pragma once

#include <cmath>
#include <memory>
#include <cstring>
#include <vector>
#include <chrono>
#include <functional>
#include <stdexcept>
#include <format>

#include <unistd.h>
#include <net/if.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "cubemars_com.hpp"

namespace cubemars
{
    class can_interface_error : public std::runtime_error
    {
    public:
        explicit can_interface_error(const std::string &__arg) : std::runtime_error(__arg) {};
        explicit can_interface_error(const char *__arg) : std::runtime_error(__arg) {};
    };

    class can_device_error : public std::runtime_error
    {
    public:
        explicit can_device_error(const std::string &__arg) : std::runtime_error(__arg) {};
        explicit can_device_error(const char *__arg) : std::runtime_error(__arg) {};
    };

    class motor_error : public std::runtime_error
    {
    public:
        explicit motor_error(const std::string &__arg) : std::runtime_error(__arg) {};
        explicit motor_error(const char *__arg) : std::runtime_error(__arg) {};
    };

    class CubemarsCan
    {
    public:
        CubemarsCan(const std::string &can_interface, const int &enable_loopback, const std::vector<joint_config_t> &joint_configs, const long &socket_timeout_sec, const long &socket_timeout_usec);
        void start_motor_control_mode(unsigned int joint_id, bool set_zero_postion_on_enable);
        void end_motor_control_mode(unsigned int joint_id);
        void start_motor_control_mode(bool set_zero_postion_on_enable);
        void end_motor_control_mode();

        void send_and_receive(const std::vector<joint_cmd_t> &cmds, std::vector<joint_state_t> &states);

        const std::string &GetName()
        {
            return can_interface_;
        }

        canid_t get_can_id(unsigned int joint_index)
        {
            return joint_configs_.at(joint_index).can_id;
        }

        virtual ~CubemarsCan();

    private:
        std::string can_interface_;
        int enable_loopback_;
        std::vector<joint_config_t> joint_configs_;
        long socket_timeout_sec_;
        long socket_timeout_usec_;
        std::vector<bool> send_ok_;
        std::vector<bool> recv_ok_;
        int can_socket_fd_;
        can_frame send_frame_;
        can_frame recv_frame_;

        /**Can helper functions**/
        void send_control_frameV2(const canid_t &can_id, const std::array<uint8_t, CAN_MAX_DLEN> &control_sequence);

        template <uint8_t mesage_len>
        void send_control_frameV3(const canid_t &can_id, const CAN_PACKET_ID &mode_id, const std::array<uint8_t, mesage_len> &control_sequence)
        {
            static_assert(mesage_len <= CAN_MAX_DLEN, "Message length exceeds CAN max data length");
            send_frame_.can_id = ((can_id | ((uint32_t)mode_id << 8)) & CAN_EFF_MASK) | CAN_EFF_FLAG;
            send_frame_.len = mesage_len;
            if (control_sequence.data() != send_frame_.data)
            { // Only copy if nececarry
                std::copy(control_sequence.begin(), control_sequence.end(), send_frame_.data);
            }
            if (::write(can_socket_fd_, &send_frame_, sizeof(struct can_frame)) < 0)
            {
                send_frame_.len = CAN_MAX_DLEN;
                throw cubemars::can_device_error(std::format("Failed to write can frame to can_id {} - {}", std::to_string(can_id), std::string(strerror(errno))));
            }
            send_frame_.len = CAN_MAX_DLEN;
            // Receive answer
            memset(&recv_frame_, 0, sizeof(recv_frame_));
            int nbytes = ::read(can_socket_fd_, &recv_frame_, CAN_MTU);
            if (nbytes <= 0)
            {
                throw cubemars::can_device_error(std::format("Did not receive reply from can_id {} - {} ", std::to_string(can_id), std::string(strerror(errno))));
            }
            auto id = recv_frame_.can_id & 0xFF;
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


        /**
         * Converts a float to an unsigned int, given range and number of bits
         * @param x float to convert
         * @param x_min lower bound of range
         * @param x_max upper bound of range
         * @param bits number of bits
         */
        static inline unsigned int float_to_uint(float x, float x_min, float x_max, unsigned int bits)
        {
            float span = x_max - x_min;
            if (x < x_min)
                x = x_min;
            else if (x > x_max)
                x = x_max;

            return (unsigned int)((x - x_min) * ((float)((1 << bits) -1)) / span);
        }

        /**
         * Converts unsigned int to float, given range and number of bits
         */
        static inline float uint_to_float(int x_int, float x_min, float x_max, int bits)
        {
            float span = x_max - x_min;
            float offset = x_min;
            return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
        }
    };
}
