#pragma once

#include <Arduino.h>
#include "Secrets.hpp"

#if !defined(AGV_RAISED_WHEEL_BUILD) \
    || !defined(AGV_MOTOR_OUTPUTS_ENABLED) \
    || !defined(AGV_PHYSICAL_FLEET_ENABLED) \
    || !defined(AGV_PHYSICAL_FLEET_MOTOR_ENABLED) \
    || !defined(AGV_STRAIGHT_CALIBRATION_ENABLED) \
    || !defined(AGV_STRAIGHT_CALIBRATION_MOTOR_ENABLED) \
    || !defined(AGV_TURN_CALIBRATION_ENABLED) \
    || !defined(AGV_TURN_CALIBRATION_MOTOR_ENABLED) \
    || !defined(AGV_TURN_CALIBRATION_DIRECTION)
#error "Select an explicit PlatformIO motor-safety build profile"
#endif

#if (AGV_RAISED_WHEEL_BUILD != 0 && AGV_RAISED_WHEEL_BUILD != 1) \
    || (AGV_MOTOR_OUTPUTS_ENABLED != 0 \
        && AGV_MOTOR_OUTPUTS_ENABLED != 1) \
    || (AGV_PHYSICAL_FLEET_ENABLED != 0 \
        && AGV_PHYSICAL_FLEET_ENABLED != 1) \
    || (AGV_PHYSICAL_FLEET_MOTOR_ENABLED != 0 \
        && AGV_PHYSICAL_FLEET_MOTOR_ENABLED != 1) \
    || (AGV_STRAIGHT_CALIBRATION_ENABLED != 0 \
        && AGV_STRAIGHT_CALIBRATION_ENABLED != 1) \
    || (AGV_STRAIGHT_CALIBRATION_MOTOR_ENABLED != 0 \
        && AGV_STRAIGHT_CALIBRATION_MOTOR_ENABLED != 1) \
    || (AGV_TURN_CALIBRATION_ENABLED != 0 \
        && AGV_TURN_CALIBRATION_ENABLED != 1) \
    || (AGV_TURN_CALIBRATION_MOTOR_ENABLED != 0 \
        && AGV_TURN_CALIBRATION_MOTOR_ENABLED != 1) \
    || (AGV_TURN_CALIBRATION_DIRECTION < 0 \
        || AGV_TURN_CALIBRATION_DIRECTION > 2)
#error "Motor-safety flags must be binary; turn direction must be 0, 1, or 2"
#endif

#if AGV_RAISED_WHEEL_BUILD && AGV_PHYSICAL_FLEET_ENABLED
#error "Raised-wheel demo and physical-fleet modes are mutually exclusive"
#endif

#if AGV_PHYSICAL_FLEET_MOTOR_ENABLED && !AGV_PHYSICAL_FLEET_ENABLED
#error "Physical-fleet motor output requires the physical-fleet executor"
#endif

#if AGV_STRAIGHT_CALIBRATION_MOTOR_ENABLED \
    && !AGV_STRAIGHT_CALIBRATION_ENABLED
#error "Calibration motor output requires the straight-calibration mode"
#endif

#if AGV_STRAIGHT_CALIBRATION_ENABLED \
    && (AGV_RAISED_WHEEL_BUILD || AGV_PHYSICAL_FLEET_ENABLED \
        || AGV_TURN_CALIBRATION_ENABLED)
#error "Straight calibration must remain isolated from other live modes"
#endif

#if AGV_TURN_CALIBRATION_MOTOR_ENABLED \
    && !AGV_TURN_CALIBRATION_ENABLED
#error "Turn calibration motor output requires the turn-calibration mode"
#endif

#if AGV_TURN_CALIBRATION_ENABLED \
    && (AGV_RAISED_WHEEL_BUILD || AGV_PHYSICAL_FLEET_ENABLED \
        || AGV_STRAIGHT_CALIBRATION_ENABLED)
#error "Turn calibration must remain isolated from other live modes"
#endif

#if AGV_TURN_CALIBRATION_ENABLED \
    && (AGV_TURN_CALIBRATION_DIRECTION != 1 \
        && AGV_TURN_CALIBRATION_DIRECTION != 2)
#error "Turn calibration must select CW=1 or CCW=2"
#endif

#if !AGV_TURN_CALIBRATION_ENABLED && AGV_TURN_CALIBRATION_DIRECTION != 0
#error "Turn direction must be disabled outside turn-calibration mode"
#endif

#if AGV_MOTOR_OUTPUTS_ENABLED \
    != (AGV_RAISED_WHEEL_BUILD \
        || AGV_PHYSICAL_FLEET_MOTOR_ENABLED \
        || AGV_STRAIGHT_CALIBRATION_MOTOR_ENABLED \
        || AGV_TURN_CALIBRATION_MOTOR_ENABLED)
#error "Motor-output selection does not match the explicit live profile"
#endif

namespace AppConfig
{
    static constexpr const char* kWifiSsid = LocalSecrets::kWifiSsid;
    static constexpr const char* kWifiPassword = LocalSecrets::kWifiPassword;

    // Recheck this with ipconfig if the PC reconnects to Wi-Fi.
    static constexpr char kServerHost[] = "192.168.45.203";
    static constexpr uint16_t kServerPort = 16666;
    static constexpr uint32_t kRequestedAgvID = 1;

    static constexpr uint32_t kSerialBaud = 115200;
    static constexpr uint32_t kStatusIntervalMs = 100;
    static constexpr uint32_t kStatusLogIntervalMs = 1000;
    static constexpr uint32_t kReconnectIntervalMs = 2000;
    static constexpr uint32_t kHelloAckTimeoutMs = 3000;

    // Only explicit live profiles may enable motor output. The default
    // esp32dev environment keeps every motor-output lock off.
    static constexpr bool kRaisedWheelBuild = AGV_RAISED_WHEEL_BUILD != 0;
    static constexpr bool kEnableMotorOutputs =
        AGV_MOTOR_OUTPUTS_ENABLED != 0;
    static constexpr bool kPhysicalFleetEnabled =
        AGV_PHYSICAL_FLEET_ENABLED != 0;
    static constexpr bool kPhysicalFleetMotorBuild =
        AGV_PHYSICAL_FLEET_MOTOR_ENABLED != 0;
    static constexpr bool kStraightCalibrationEnabled =
        AGV_STRAIGHT_CALIBRATION_ENABLED != 0;
    static constexpr bool kStraightCalibrationMotorBuild =
        AGV_STRAIGHT_CALIBRATION_MOTOR_ENABLED != 0;
    static constexpr bool kTurnCalibrationEnabled =
        AGV_TURN_CALIBRATION_ENABLED != 0;
    static constexpr bool kTurnCalibrationMotorBuild =
        AGV_TURN_CALIBRATION_MOTOR_ENABLED != 0;
    static constexpr int kTurnCalibrationDirection =
        AGV_TURN_CALIBRATION_DIRECTION;

    // Only this exact two-node route is accepted in the first physical demo.
    // The current server protocol carries no metric distance, so [1 -> 2]
    // temporarily means the already verified local 30 cm motion.
    static constexpr uint32_t kDemoStartNodeID = 1;
    static constexpr uint32_t kDemoTargetNodeID = 2;
    static constexpr int32_t kForward30CmCount = 520;
    static constexpr uint32_t kEncoderSettleStableMs = 150;
    static constexpr uint32_t kEncoderSettleTimeoutMs = 2000;
    static constexpr uint32_t kApprovalCountdownMs = 5000;
    static constexpr uint32_t kPrimitiveSafePauseMs = 500;
    static constexpr uint32_t kPhysicalFleetStartNodeID = 1;
    static constexpr float kPhysicalFleetStartHeadingRad = 0.0f;
    static constexpr float kPhysicalFleetScaleMmPerMapUnit = 50.0f;
    static constexpr float kPhysicalFleetCruiseSpeedMmPerSecond = 80.0f;
    static constexpr float kForwardCountsPerMm = 520.0f / 300.0f;
    static constexpr float kTurnCountsPerRadian =
        176.0f / 1.57079632679489661923f;
    static constexpr int32_t kTurn90Count = 176;
    static constexpr uint32_t kCalibrationSampleIntervalMs = 50;
    static constexpr int kBootButtonPin = 0;
    static constexpr uint32_t kButtonDebounceMs = 50;

    // Physically verified TB6612 and encoder wiring.
    static constexpr int kMotorStandbyPin = 13;
    static constexpr int kLeftMotorIn1Pin = 25;
    static constexpr int kLeftMotorIn2Pin = 26;
    static constexpr int kLeftMotorPwmPin = 27;
    static constexpr int kRightMotorIn1Pin = 33;
    static constexpr int kRightMotorIn2Pin = 32;
    static constexpr int kRightMotorPwmPin = 14;

    static constexpr int kLeftEncoderAPin = 19;
    static constexpr int kLeftEncoderBPin = 18;
    static constexpr int kRightEncoderAPin = 17;
    static constexpr int kRightEncoderBPin = 16;

    static constexpr int kLeftPwmChannel = 0;
    static constexpr int kRightPwmChannel = 1;
    static constexpr int kPwmFrequency = 20000;
    static constexpr int kPwmResolutionBits = 8;
}
