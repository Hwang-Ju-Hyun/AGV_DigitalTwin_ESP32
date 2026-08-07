#pragma once

#include <Arduino.h>
#include "Secrets.hpp"

namespace AppConfig
{
    static constexpr const char* kWifiSsid = LocalSecrets::kWifiSsid;
    static constexpr const char* kWifiPassword = LocalSecrets::kWifiPassword;

    // Recheck this with ipconfig if the PC reconnects to Wi-Fi.
    static constexpr char kServerHost[] = "192.168.45.126";
    static constexpr uint16_t kServerPort = 6666;
    static constexpr uint32_t kRequestedAgvID = 1;

    static constexpr uint32_t kSerialBaud = 115200;
    static constexpr uint32_t kStatusIntervalMs = 100;
    static constexpr uint32_t kStatusLogIntervalMs = 1000;
    static constexpr uint32_t kReconnectIntervalMs = 2000;
    static constexpr uint32_t kHelloAckTimeoutMs = 3000;

    // Phase 2B integration is build-only. Physical output activation requires
    // a separate, explicitly approved safety review and powered test.
    static constexpr bool kEnableMotorOutputs = false;

    // Only this exact two-node route is accepted in the first physical demo.
    // The current server protocol carries no metric distance, so [1 -> 2]
    // temporarily means the already verified local 30 cm motion.
    static constexpr uint32_t kDemoStartNodeID = 1;
    static constexpr uint32_t kDemoTargetNodeID = 2;
    static constexpr int32_t kForward30CmCount = 520;
    static constexpr uint32_t kEncoderSettleStableMs = 150;
    static constexpr uint32_t kEncoderSettleTimeoutMs = 2000;
    static constexpr uint32_t kApprovalCountdownMs = 5000;
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
