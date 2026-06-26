#include "cubemars_hardware_interface/mab_fd_can.hpp"

#include <cerrno>
#include <cstring>
#include <format>

#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can/raw.h>

namespace cubemars
{

MabFdCan::MabFdCan(const std::string &can_interface, const int &enable_loopback,
                   const std::vector<joint_config_t> &joint_configs,
                   const long &socket_timeout_sec, const long &socket_timeout_usec,
                   unsigned int max_init_connect_trials,
                   bool enable_tx_timestamping, bool enable_can_error_frames)
    : can_interface_(can_interface),
      enable_loopback_(enable_loopback),
      joint_configs_(joint_configs),
      socket_timeout_sec_(socket_timeout_sec),
      socket_timeout_usec_(socket_timeout_usec),
      max_initial_connection_trials_(max_init_connect_trials),
      enable_tx_timestamping_(enable_tx_timestamping),
      enable_can_error_frames_(enable_can_error_frames)
{
    // Configuring CAN socket
    struct sockaddr_can addr;
    struct ifreq ifr;
    can_socket_fd_ = socket(AF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket_fd_ < 0)
    {
        throw can_interface_error(std::format("Failed to create CAN socket - {}", std::string(strerror(errno))));
    }

    // Enable CAN FD frames on the socket. This is the defining difference from the CubeMars backend:
    // MAB electronics use CAN FD, so frames are canfd_frame (up to 64 data bytes) rather than can_frame.
    int enable_canfd = 1;
    if (setsockopt(can_socket_fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd)) != 0)
    {
        close(can_socket_fd_);
        throw can_interface_error(std::format("Failed to enable CAN FD frames - {} ", std::string(strerror(errno))));
    }

    // Configure loopback
    if (setsockopt(can_socket_fd_, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &enable_loopback, sizeof(enable_loopback)) < 0)
    {
        close(can_socket_fd_);
        throw can_interface_error(std::format("Failed to set loopback to {} - {} ", enable_loopback, std::string(strerror(errno))));
    }

    // Set CAN RX filters: deliver only the configured motors' reply frames.
    std::vector<can_filter> rfilter;
    bool zero_added = false;
    for (unsigned int i = 0; i < joint_configs_.size(); i++)
    {
        switch (joint_configs_[i].series_type)
        {
        case SERIES_TYPE::V2:
            if (joint_configs_[i].reply_on_own_id)
            {
                rfilter.push_back({joint_configs_[i].can_id, CAN_SFF_MASK | CAN_EFF_FLAG});
            }
            else if (!zero_added)
            {
                rfilter.push_back({0, CAN_SFF_MASK | CAN_EFF_FLAG});
            }
            break;
        case SERIES_TYPE::V3:
            rfilter.push_back({joint_configs_[i].can_id | ((uint32_t)CAN_PACKET_RESPONSE << 8) | CAN_EFF_FLAG, CAN_EFF_MASK | CAN_EFF_FLAG});
            break;
        }
    }
    if (setsockopt(can_socket_fd_, SOL_CAN_RAW, CAN_RAW_FILTER, rfilter.data(), rfilter.size() * sizeof(can_filter)) < 0)
    {
        close(can_socket_fd_);
        throw can_interface_error(std::format("Failed to setup CAN filter - {} ", std::string(strerror(errno))));
    }

    // Find CAN interface index
    memset(&ifr, 0, sizeof(ifr));
    strcpy(ifr.ifr_name, can_interface_.c_str());
    if (ioctl(can_socket_fd_, SIOCGIFINDEX, &ifr))
    {
        close(can_socket_fd_);
        throw can_interface_error(std::format("Failed to find CAN interface {} - {} ", can_interface, std::string(strerror(errno))));
    }

    // Bind the socket to the interface
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(can_socket_fd_, (struct sockaddr *)&addr, sizeof(struct sockaddr)) < 0)
    {
        close(can_socket_fd_);
        throw can_interface_error(std::format("Failed to bind CAN socket - {} ", std::string(strerror(errno))));
    }

    // Set receive timeout so a missing reply surfaces as a timeout instead of blocking forever.
    struct timeval tv;
    tv.tv_sec = socket_timeout_sec_;
    tv.tv_usec = socket_timeout_usec_;
    if (setsockopt(can_socket_fd_, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof(struct timeval)) < 0)
    {
        close(can_socket_fd_);
        throw can_interface_error(std::format("Failed to set socket option for timeout - {} ", std::string(strerror(errno))));
    }

    // setup vars
    memset(&send_frame_, 0, sizeof(send_frame_));
    send_frame_.len = CAN_MAX_DLEN;
}

MabFdCan::~MabFdCan()
{
    close(can_socket_fd_); // If this goes wrong, we cant do anything
}

// ---------------------------------------------------------------------------------------------------
// Register protocol: not implemented yet. The next merge step ports the MAB_FDCAN send_and_receive,
// motor enable/disable (MotorMode/MotorState/RunZero register writes) and timestamping. Until then the
// "doing" methods throw a clear error; the "undo" methods are no-ops (nothing can have been enabled).
// ---------------------------------------------------------------------------------------------------

void MabFdCan::start_motor_control_mode(unsigned int, bool)
{
    throw can_device_error("MabFdCan: motor control mode is not implemented yet "
                           "(only the CAN FD socket is set up; the MAB register protocol port is pending).");
}

void MabFdCan::start_motor_control_mode(bool)
{
    throw can_device_error("MabFdCan: motor control mode is not implemented yet "
                           "(only the CAN FD socket is set up; the MAB register protocol port is pending).");
}

void MabFdCan::end_motor_control_mode(unsigned int)
{
    // No-op: nothing can have been enabled yet (start_motor_control_mode is not implemented).
}

void MabFdCan::end_motor_control_mode()
{
    // No-op: nothing can have been enabled yet (start_motor_control_mode is not implemented).
}

void MabFdCan::send_and_receive(const std::vector<joint_cmd_t> &, std::vector<joint_state_t> &)
{
    throw can_device_error("MabFdCan: send_and_receive is not implemented yet "
                           "(the MAB register protocol port is pending).");
}

} // namespace cubemars
