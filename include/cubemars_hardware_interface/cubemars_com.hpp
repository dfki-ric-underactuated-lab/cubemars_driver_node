#ifndef CUBEMARS_COM_HPP_
#define CUBEMARS_COM_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

#include <linux/can.h>
#include <limits>
#include <map>

namespace cubemars
{
    typedef enum
    {
        UNDEFINED = 0,
        POSITION,
        VELOCITY,
        EFFORT,
        ERROR
    } JointMode;

    typedef enum
    {
        NO_FAULT = 0,
        MOTOR_OVER_TEMP,
        OVER_CURRENT,
        OVER_VOLTAGE,
        UNDER_VOLTAGE,
        ENCODER_FAULT,
        MOSFET_OVER_TEMP,
        MOTOR_STALL
    } ErrorCode;

    constexpr const char *errorFlagToString(ErrorCode error_flag) throw()
    {
        switch (error_flag)
        {
        case NO_FAULT:
            return "NO_FAULT";
        case MOTOR_OVER_TEMP:
            return "MOTOR_OVER_TEMP";
        case OVER_CURRENT:
            return "OVER_CURRENT";
        case OVER_VOLTAGE:
            return "OVER_VOLTAGE";
        case UNDER_VOLTAGE:
            return "UNDER_VOLTAGE";
        case ENCODER_FAULT:
            return "ENCODER_FAULT";
        case MOSFET_OVER_TEMP:
            return "MOSFET_OVER_TEMP";
        case MOTOR_STALL:
            return "MOTOR_STALL";
        }
        return "UNKNOWN_ERROR_CODE";
    }

    typedef enum
    {
        SUCCESS = 0,
        CAN_WRITE_FAILED = 1,
        CAN_READ_FAILED = 2,
        CAN_NO_RESPONSE = 3,
        CAN_WRITE_FAILED_BUT_RESPONSE_RECEIVED = 4,
    } ComStatus;

    constexpr const char *comStatusToString(ComStatus com_status) throw()
    {
        switch (com_status)
        {
        case SUCCESS:
            return "SUCCESS";
        case CAN_WRITE_FAILED:
            return "CAN_WRITE_FAILED";
        case CAN_READ_FAILED:
            return "CAN_READ_FAILED";
        case CAN_NO_RESPONSE:
            return "CAN_NO_RESPONSE";
        case CAN_WRITE_FAILED_BUT_RESPONSE_RECEIVED:
            return "CAN_WRITE_FAILED_BUT_RESPONSE_RECEIVED";
        }
        return "UNKNOWN_COM_STATUS";
    }

    typedef enum
    {
        CAN_PACKET_SET_DUTY = 0,      // Duty Cycle Mode
        CAN_PACKET_SET_CURRENT,       // Current Loop Mode
        CAN_PACKET_SET_CURRENT_BRAKE, // Current Brake Mode
        CAN_PACKET_SET_RPM,           // Speed Mode
        CAN_PACKET_SET_POS,           // Position Mode
        CAN_PACKET_SET_ORIGIN_HERE,   // Set Origin Mode
        CAN_PACKET_SET_POS_SPD,       // Position-Speed Loop Mode
        CAN_PACKET_SET_MIT = 8, // MIT MODE (for v3 motors)
        CAN_PACKET_RESPONSE = 0x29 // Response Packet (for v3 motors)
    } CAN_PACKET_ID;

    typedef enum
    {
        V2,
        V3
    } SERIES_TYPE;

    struct joint_config_t
    {
        canid_t can_id = 0;
        std::string name = "";
        double P_MIN = std::numeric_limits<double>::quiet_NaN();
        double P_MAX = std::numeric_limits<double>::quiet_NaN();
        double V_MIN = std::numeric_limits<double>::quiet_NaN();
        double V_MAX = std::numeric_limits<double>::quiet_NaN();
        double I_MIN = std::numeric_limits<double>::quiet_NaN();
        double I_MAX = std::numeric_limits<double>::quiet_NaN();
        double KP_MIN = std::numeric_limits<double>::quiet_NaN();
        double KP_MAX = std::numeric_limits<double>::quiet_NaN();
        double KD_MIN = std::numeric_limits<double>::quiet_NaN();
        double KD_MAX = std::numeric_limits<double>::quiet_NaN();
        bool invert = false;
        bool reply_on_own_id = false; //Needed to differentiate v2 with and without plastics screw
        SERIES_TYPE series_type = SERIES_TYPE::V2;
        uint8_t numer_of_pole_pairs = 21; //Needed for v3 to convert back the ERPM to RPM
        uint8_t gear_ratio = 9; //Needed for v3 to convert back the ERPM to RPM
        double torque_constant = 1; //Needed for v3 to convert back the current measurement to torque (has to be experperemntally determined!)
        bool current_mesurement_has_sign = true; //Some v3 motors measure the current signless
    };

    // For MIT mode with v2 motors
    static const std::array<uint8_t, 8> START_MOTOR_CONTROL_MODE = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    static const std::array<uint8_t, 8> EXIT_MOTOR_CONTROL_MODE = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    static const std::array<uint8_t, 8> SET_ZERO_POSITION = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};


    struct joint_cmd_t
    {
        float pos;
        float vel;
        float torque;
        float kp;
        float kd;
    };

    struct joint_state_t
    {
        float pos;
        float vel;
        float torque;
        float temp;
        float output_encoder_pos;       // output-side encoder position [rad], NaN where unsupported (e.g. non-MAB drivers)
        float output_encoder_vel;       // output-side encoder velocity [rad/s], NaN where unsupported (e.g. non-MAB drivers)
        ErrorCode device_status;
        ComStatus communication_status;
        int com_errno;
        int64_t rx_timestamp_ns;       // CLOCK_REALTIME nanoseconds of reply frame arrival (kernel RX timestamp)
        int64_t send_timestamp_ns;     // CLOCK_REALTIME nanoseconds of command frame TX completion (kernel software TX timestamp), 0 if unavailable
        int64_t dequeue_timestamp_ns;  // CLOCK_REALTIME nanoseconds the reply was read into node space (userspace, after recvmsg), 0 if no reply
        int64_t enqueue_timestamp_ns;  // CLOCK_REALTIME nanoseconds the command frame was written into the TX buffer (userspace, after ::write), 0 if write failed
        int64_t rx_hw_timestamp_ns;    // raw NIC/card hardware-clock nanoseconds of reply frame arrival (free-running clock, NOT CLOCK_REALTIME), 0 if unavailable
    };

    static const std::map<std::string, joint_config_t> joint_config_per_motor_type ={
        {"AK10-9",      {0,"", -12.5, 12.5, -50.0, 50.0, -65.0, 65.0, 0.0, 500.0, 0.0, 5.0, false, true, SERIES_TYPE::V2}},
        {"AK10-9_plastic_screw",      {0,"", -12.5, 12.5, -50.0, 50.0, -65.0, 65.0, 0.0, 500.0, 0.0, 5.0, false, false, SERIES_TYPE::V2}},
        {"AK80-6",      {0,"", -12.5, 12.5, -76.0, 76.0, -12.0, 12.0, 0.0, 500.0, 0.0, 5.0, false, true, SERIES_TYPE::V2}},
        {"AK80-6_V1p1", {0,"", -12.5, 12.5, -22.5, 22.5, -12.0, 12.0, 0.0, 500.0, 0.0, 5.0, true, true, SERIES_TYPE::V2}},
        {"AK10-9v3",      {0,"", -12.56, 12.56, -28.0, 28.0, -54.0, 54.0, 0.0, 500.0, 0.0, 5.0, false, false, SERIES_TYPE::V3,21,9,1.1314,false}},
        {"AK80-8v3",      {0,"", -12.5, 12.5, -37.5, 37.5, -32.0, 32.0, 0.0, 500.0, 0.0, 5.0, false, false, SERIES_TYPE::V3,21,8,1.0566,true}},
        {"MAB",      {0,"", -12.5, 12.5, -50.0, 50.0, -65.0, 65.0, 0.0, 500.0, 0.0, 5.0, false, true, SERIES_TYPE::V2}},
    };

} // namespace cubemars

#endif // CUBEMARS_COM_HPP_