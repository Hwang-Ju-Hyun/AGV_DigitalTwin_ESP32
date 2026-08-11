#pragma once

#include <cstdint>

#if !defined(AGV_TRAJECTORY_PREVIEW_ENABLED)
#error "Select an explicit trajectory-preview build profile"
#endif

#if AGV_TRAJECTORY_PREVIEW_ENABLED != 0 \
    && AGV_TRAJECTORY_PREVIEW_ENABLED != 1
#error "AGV_TRAJECTORY_PREVIEW_ENABLED must be 0 or 1"
#endif

namespace TrajectoryConfig
{
    // Preview means parse, validate and retain only. It deliberately exposes
    // no API that can start MotionController or RouteExecutor.
    static constexpr bool kPreviewEnabled =
        AGV_TRAJECTORY_PREVIEW_ENABLED != 0;

    // Physically measured chassis values. The track width remains a nominal
    // value until a later odometry calibration task.
    static constexpr double kWheelDiameterMm = 48.0;
    static constexpr double kTrackWidthMm = 130.0;
    static constexpr int32_t kCountsPerWheelRevolution = 260;

    static constexpr float kTrajectoryOriginToleranceMm = 1.0f;
}
