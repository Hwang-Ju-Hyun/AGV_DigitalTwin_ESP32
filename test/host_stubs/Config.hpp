#pragma once

#include <cstdint>

namespace AppConfig
{
    static constexpr uint32_t kPrimitiveSafePauseMs = 500;
    static constexpr float kPhysicalFleetScaleMmPerMapUnit = 50.0f;
    static constexpr float kForwardCountsPerMm = 520.0f / 300.0f;
    static constexpr float kForwardLeftTargetScale = 1.0f;
    static constexpr float kForwardRightTargetScale = 1.0f;
    static constexpr int32_t kTurn90CwCount = 163;
    static constexpr int32_t kTurn90CcwCount = 159;
    static constexpr float kTurnCwCountsPerRadian =
        static_cast<float>(kTurn90CwCount)
        / 1.57079632679489661923f;
    static constexpr float kTurnCcwCountsPerRadian =
        static_cast<float>(kTurn90CcwCount)
        / 1.57079632679489661923f;
    static constexpr float kCorrectionMinimumDriveMm = 20.0f;
    static constexpr float kCorrectionMaximumDriveMm = 120.0f;
    static constexpr float kCorrectionMinimumTurnRad =
        5.0f * 3.14159265358979323846f / 180.0f;
    static constexpr float kCorrectionMaximumTurnRad =
        3.14159265358979323846f / 2.0f;
    static constexpr uint8_t kMaximumCorrectionPrimitivesPerNode = 8;
    static constexpr uint32_t kCorrectionPrimitiveTimeoutMs = 10000;
    static constexpr uint32_t kEncoderSettleStableMs = 150;
    static constexpr uint32_t kEncoderSettleTimeoutMs = 2000;
    static constexpr bool kEnableMotorOutputs = true;
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
