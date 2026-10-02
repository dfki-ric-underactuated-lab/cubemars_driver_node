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
#include "can_comm_base.hpp"

namespace cubemars
{
    // CAN exception types (can_interface_error / can_device_error / motor_error) are declared in
    // can_comm_base.hpp so every backend shares them.
    class CubemarsCan : public CanCommBase
    {
    public:
        CubemarsCan(const std::string &can_interface, const int &enable_loopback, const std::vector<joint_config_t> &joint_configs, const long &socket_timeout_sec, const long &socket_timeout_usec, unsigned int max_init_connect_trials, bool enable_tx_timestamping = true, bool enable_can_error_frames = false);
        void start_motor_control_mode(unsigned int joint_id, bool set_zero_postion_on_enable) override;
        void end_motor_control_mode(unsigned int joint_id) override;
        void start_motor_control_mode(bool set_zero_postion_on_enable) override;
        void end_motor_control_mode() override;
        void set_zero_position(unsigned int joint_id) override;

        void send_and_receive(const std::vector<joint_cmd_t> &cmds, std::vector<joint_state_t> &states, bool is_active) override;

        const std::string &GetName() override
        {
            return can_interface_;
        }

        canid_t get_can_id(unsigned int joint_index) override
        {
            return joint_configs_.at(joint_index).can_id;
        }

        // Diagnostics from the last send_and_receive() (CLOCK_REALTIME ns):
        int64_t get_tx_fill_duration_ns() const override { return tx_fill_duration_ns_; } // time to write all command frames to the TX buffer
        int64_t get_rx_duration_ns() const override { return rx_duration_ns_; }           // time to receive all replies after the TX buffer was filled
        int64_t get_tx_fill_end_ns() const override { return tx_fill_end_ns_; }           // timestamp the TX buffer finished being filled (reference for per-motor reply timing)
        unsigned int get_last_tx_errq_count() const override { return last_tx_errq_count_; } // error-queue entries drained in the last collect_tx_timestamps()
        unsigned int get_last_error_count() const override { return last_error_count_; }      // CAN bus-error frames seen in the last send_and_receive()
        canid_t get_last_error_canid() const override { return last_error_canid_; }           // can_id (error class bits) of the most recent error frame

        ~CubemarsCan() override;

    private:
        std::string can_interface_;
        int enable_loopback_;
        bool enable_tx_timestamping_;
        bool enable_can_error_frames_;
        std::vector<joint_config_t> joint_configs_;
        long socket_timeout_sec_;
        long socket_timeout_usec_;
        unsigned int max_initial_connection_trials_;
        std::vector<bool> send_ok_;
        std::vector<bool> recv_ok_;
        int64_t tx_fill_duration_ns_ = 0;
        int64_t rx_duration_ns_ = 0;
        int64_t tx_fill_end_ns_ = 0;
        unsigned int last_tx_errq_count_ = 0;
        unsigned int last_error_count_ = 0;
        canid_t last_error_canid_ = 0;
        int can_socket_fd_;
        can_frame send_frame_;
        can_frame recv_frame_;

        /**Can helper functions**/
        // recvmsg-based receive that pulls the RX timestamp ancillary data.
        // Returns the same value as recvmsg() (bytes received, or -1 on error).
        // rx_timestamp_ns is set to the kernel software RX timestamp (SO_TIMESTAMPNS) in CLOCK_REALTIME ns,
        // or 0 if unavailable. rx_hw_timestamp_ns is set to the card's hardware RX timestamp
        // (SCM_TIMESTAMPING ts[2], raw free-running card clock), or 0 if unavailable. Comparing the two
        // tells a reply that was late on the wire from one the kernel delivered late.
        int recv_frame_with_timestamp(struct can_frame &frame, int64_t &rx_timestamp_ns, int64_t &rx_hw_timestamp_ns);

        // Drains the socket error queue (MSG_ERRQUEUE) of software TX timestamps and assigns each
        // to the matching joint's send_timestamp_ns (matched by the echoed frame's can_id). Called
        // after the receive loop, when the command frames have completed transmission.
        void collect_tx_timestamps(std::vector<joint_state_t> &states);

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
