#pragma once

#include <cstdint>

namespace AppConfig
{
    static constexpr uint32_t kPrimitiveSafePauseMs = 500;
    static constexpr float kPhysicalFleetScaleMmPerMapUnit = 50.0f;
    static constexpr float kForwardCountsPerMm = 520.0f / 300.0f;
    static constexpr float kTurnCountsPerRadian =
        176.0f / 1.57079632679489661923f;
    static constexpr float kCorrectionMinimumDriveMm = 20.0f;
    static constexpr float kCorrectionMaximumDriveMm = 120.0f;
    static constexpr float kCorrectionMinimumTurnRad =
        5.0f * 3.14159265358979323846f / 180.0f;
    static constexpr float kCorrectionMaximumTurnRad =
        3.14159265358979323846f / 2.0f;
    static constexpr uint8_t kMaximumCorrectionPrimitivesPerNode = 8;
    static constexpr uint32_t kCorrectionPrimitiveTimeoutMs = 10000;
}
