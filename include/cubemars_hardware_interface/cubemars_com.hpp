#ifndef CUBEMARS_COM_HPP_
#define CUBEMARS_COM_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

#include <linux/can.h>
#include <limits>

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
        FAULT_CODE_NONE = 0,
        FAULT_CODE_OVER_VOLTAGE,                       // Overvoltage
        FAULT_CODE_UNDER_VOLTAGE,                      // Undervoltage
        FAULT_CODE_DRV,                                // Driver fault
        FAULT_CODE_ABS_OVER_CURRENT,                   // Motor overcurrent
        FAULT_CODE_OVER_TEMP_FET,                      // MOS overtemperature
        FAULT_CODE_OVER_TEMP_MOTOR,                    // Motor overtemperature
        FAULT_CODE_GATE_DRIVER_OVER_VOLTAGE,           // Driver overvoltage
        FAULT_CODE_GATE_DRIVER_UNDER_VOLTAGE,          // Driver undervoltage
        FAULT_CODE_MCU_UNDER_VOLTAGE,                  // MCU undervoltage
        FAULT_CODE_BOOTING_FROM_WATCHDOG_RESET,        // Undervoltage
        FAULT_CODE_ENCODER_SPI,                        // SPI encoder fault
        FAULT_CODE_ENCODER_SINCOS_BELOW_MIN_AMPLITUDE, // Encoder below minimum amplitude
        FAULT_CODE_ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE, // Encoder above maximum amplitude
        FAULT_CODE_FLASH_CORRUPTION,                   // Flash fault
        FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_1,       // Current sampling channel 1 fault
        FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_2,       // Current sampling channel 2 fault
        FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_3,       // Current sampling channel 3 fault
        FAULT_CODE_UNBALANCED_CURRENTS,                // Unbalanced currents
        FA
    } ErrorCode;

    constexpr const char *errorFlagToString(ErrorCode error_flag) throw()
    {
        switch (error_flag)
        {
        case ErrorCode::FAULT_CODE_NONE:
            return "No Error";
        case ErrorCode::FAULT_CODE_OVER_VOLTAGE:
            return "Overvoltage";
        case ErrorCode::FAULT_CODE_UNDER_VOLTAGE:
            return "Undervoltage";
        case ErrorCode::FAULT_CODE_DRV:
            return "Driver fault";
        case ErrorCode::FAULT_CODE_ABS_OVER_CURRENT:
            return "Motor overcurrent";
        case ErrorCode::FAULT_CODE_OVER_TEMP_FET:
            return "MOS overtemperature";
        case ErrorCode::FAULT_CODE_OVER_TEMP_MOTOR:
            return "Motor overtemperature";
        case ErrorCode::FAULT_CODE_GATE_DRIVER_OVER_VOLTAGE:
            return "Driver overvoltage";
        case ErrorCode::FAULT_CODE_GATE_DRIVER_UNDER_VOLTAGE:
            return "Driver undervoltage";
        case ErrorCode::FAULT_CODE_MCU_UNDER_VOLTAGE:
            return "MCU undervoltage";
        case ErrorCode::FAULT_CODE_BOOTING_FROM_WATCHDOG_RESET:
            return "Undervoltage";
        case ErrorCode::FAULT_CODE_ENCODER_SPI:
            return "SPI encoder fault";
        case ErrorCode::FAULT_CODE_ENCODER_SINCOS_BELOW_MIN_AMPLITUDE:
            return "Encoder below minimum amplitude";
        case ErrorCode::FAULT_CODE_ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE:
            return "Encoder above maximum amplitude";
        case ErrorCode::FAULT_CODE_FLASH_CORRUPTION:
            return "Flash fault";
        case ErrorCode::FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_1:
            return "Current sampling channel 1 fault";
        case ErrorCode::FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_2:
            return "Current sampling channel 2 fault";
        case ErrorCode::FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_3:
            return "Current sampling channel 3 fault";
        case ErrorCode::FAULT_CODE_UNBALANCED_CURRENTS:
            return "Unbalanced currents";
        default:
            return "Unknown Error";
        }
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
    } CAN_PACKET_ID;

    struct joint_config_t
    {
        uint32_t can_id = 0;
        std::string name = "";
        double Kp = std::numeric_limits<double>::quiet_NaN();
        double Kd = std::numeric_limits<double>::quiet_NaN();
        double P_MIN = std::numeric_limits<double>::quiet_NaN();
        double P_MAX = std::numeric_limits<double>::quiet_NaN();
        double V_MIN = std::numeric_limits<double>::quiet_NaN();
        double V_MAX = std::numeric_limits<double>::quiet_NaN();
        double I_MIN = std::numeric_limits<double>::quiet_NaN();
        double I_MAX = std::numeric_limits<double>::quiet_NaN();
        double T_MIN = std::numeric_limits<double>::quiet_NaN();
        double T_MAX = std::numeric_limits<double>::quiet_NaN();
        double Kp_MIN = std::numeric_limits<double>::quiet_NaN();
        double Kp_MAX = std::numeric_limits<double>::quiet_NaN();
        double Kd_MIN = std::numeric_limits<double>::quiet_NaN();
        double Kd_MAX = std::numeric_limits<double>::quiet_NaN();
    };

    static const std::array<uint8_t, 8> START_MOTOR_CONTROL_MODE = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    static const std::array<uint8_t, 8> EXIT_MOTOR_CONTROL_MODE = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    static const std::array<uint8_t, 8> SET_ZERO_POSITION = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};

} // namespace cubemars

#endif // CUBEMARS_COM_HPP_