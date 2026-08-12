#pragma once

#include <cstdint>

#if !defined(AGV_TRAJECTORY_PREVIEW_ENABLED)
#error "Select an explicit trajectory-preview build profile"
#endif

#if AGV_TRAJECTORY_PREVIEW_ENABLED != 0 \
    && AGV_TRAJECTORY_PREVIEW_ENABLED != 1
#error "AGV_TRAJECTORY_PREVIEW_ENABLED must be 0 or 1"
#endif

#if !defined(AGV_TRAJECTORY_TRACE_ENABLED)
#error "Select an explicit trajectory-trace build profile"
#endif

#if AGV_TRAJECTORY_TRACE_ENABLED != 0 \
    && AGV_TRAJECTORY_TRACE_ENABLED != 1
#error "AGV_TRAJECTORY_TRACE_ENABLED must be 0 or 1"
#endif

#if AGV_TRAJECTORY_TRACE_ENABLED && !AGV_TRAJECTORY_PREVIEW_ENABLED
#error "Trajectory trace requires the motor-locked preview parser"
#endif

#if !defined(AGV_PHYSICAL_FLEET_ENABLED)
#error "Select an explicit physical-fleet build profile"
#endif

#if AGV_PHYSICAL_FLEET_ENABLED != 0 \
    && AGV_PHYSICAL_FLEET_ENABLED != 1
#error "AGV_PHYSICAL_FLEET_ENABLED must be 0 or 1"
#endif

namespace TrajectoryConfig
{
    // Preview means parse, validate and retain only. It deliberately exposes
    // no API that can start MotionController or RouteExecutor.
    static constexpr bool kPreviewEnabled =
        AGV_TRAJECTORY_PREVIEW_ENABLED != 0;
    static constexpr bool kTraceEnabled =
        AGV_TRAJECTORY_TRACE_ENABLED != 0;
    static constexpr bool kPhysicalFleetEnabled =
        AGV_PHYSICAL_FLEET_ENABLED != 0;

    // Physically measured chassis values. The track width remains a nominal
    // value until a later odometry calibration task.
    static constexpr double kWheelDiameterMm = 48.0;
    static constexpr double kTrackWidthMm = 130.0;
    static constexpr int32_t kCountsPerWheelRevolution = 260;

    static constexpr float kTrajectoryOriginToleranceMm = 1.0f;
}
