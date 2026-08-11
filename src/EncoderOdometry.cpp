#include "EncoderOdometry.hpp"

#include "TrajectoryConfig.hpp"

#include <cmath>

namespace
{
    constexpr double kPi = 3.14159265358979323846;
    constexpr double kTwoPi = 2.0 * kPi;
    constexpr double kMillimetersPerCount =
        kPi * TrajectoryConfig::kWheelDiameterMm /
        static_cast<double>(TrajectoryConfig::kCountsPerWheelRevolution);
}

void EncoderOdometry::resetLocal(int32_t leftCount,
                                 int32_t rightCount,
                                 uint32_t encoderResetEpoch,
                                 uint32_t nowMs)
{
    m_ForwardMm = 0.0;
    m_LeftMm = 0.0;
    m_HeadingRad = 0.0;
    m_LinearVelocityMmPerSecond = 0.0;
    m_AngularVelocityRadPerSecond = 0.0;
    m_LastLeftCount = leftCount;
    m_LastRightCount = rightCount;
    m_LastEncoderResetEpoch = encoderResetEpoch;
    m_LastUpdateMs = nowMs;
    m_Initialized = true;
    m_Finite = true;
}

void EncoderOdometry::update(int32_t leftCount,
                             int32_t rightCount,
                             uint32_t encoderResetEpoch,
                             uint32_t nowMs)
{
    if (!m_Initialized)
    {
        resetLocal(leftCount, rightCount, encoderResetEpoch, nowMs);
        return;
    }

    const uint32_t elapsedMs = nowMs - m_LastUpdateMs;
    int64_t leftDelta = 0;
    int64_t rightDelta = 0;

    if (encoderResetEpoch != m_LastEncoderResetEpoch)
    {
        // MotionController resets both counters to zero atomically. Preserve
        // counts accumulated after that reset, but do not derive a velocity
        // from an interval that crossed two count coordinate systems.
        leftDelta = static_cast<int64_t>(leftCount);
        rightDelta = static_cast<int64_t>(rightCount);
        m_LastEncoderResetEpoch = encoderResetEpoch;
        integrate(leftDelta, rightDelta, 0);
    }
    else
    {
        leftDelta = static_cast<int64_t>(leftCount) - m_LastLeftCount;
        rightDelta = static_cast<int64_t>(rightCount) - m_LastRightCount;
        integrate(leftDelta, rightDelta, elapsedMs);
    }

    m_LastLeftCount = leftCount;
    m_LastRightCount = rightCount;
    m_LastUpdateMs = nowMs;
}

EncoderOdometry::Snapshot EncoderOdometry::snapshot() const
{
    Snapshot result;
    result.forwardMm = static_cast<float>(m_ForwardMm);
    result.leftMm = static_cast<float>(m_LeftMm);
    result.headingRad = static_cast<float>(m_HeadingRad);
    result.linearVelocityMmPerSecond =
        static_cast<float>(m_LinearVelocityMmPerSecond);
    result.angularVelocityRadPerSecond =
        static_cast<float>(m_AngularVelocityRadPerSecond);
    result.leftCount = m_LastLeftCount;
    result.rightCount = m_LastRightCount;
    result.encoderResetEpoch = m_LastEncoderResetEpoch;
    result.initialized = m_Initialized;
    result.finite = m_Finite;
    return result;
}

double EncoderOdometry::normalizeRadians(double radians)
{
    if (!std::isfinite(radians))
        return NAN;
    return std::remainder(radians, kTwoPi);
}

void EncoderOdometry::integrate(int64_t leftDeltaCount,
                                int64_t rightDeltaCount,
                                uint32_t elapsedMs)
{
    const double leftDistanceMm =
        static_cast<double>(leftDeltaCount) * kMillimetersPerCount;
    const double rightDistanceMm =
        static_cast<double>(rightDeltaCount) * kMillimetersPerCount;
    const double centerDistanceMm =
        (leftDistanceMm + rightDistanceMm) * 0.5;
    const double headingDeltaRad =
        (rightDistanceMm - leftDistanceMm) /
        TrajectoryConfig::kTrackWidthMm;
    const double midpointHeading =
        normalizeRadians(m_HeadingRad + headingDeltaRad * 0.5);

    if (!std::isfinite(leftDistanceMm)
        || !std::isfinite(rightDistanceMm)
        || !std::isfinite(centerDistanceMm)
        || !std::isfinite(headingDeltaRad)
        || !std::isfinite(midpointHeading))
    {
        m_LinearVelocityMmPerSecond = 0.0;
        m_AngularVelocityRadPerSecond = 0.0;
        m_Finite = false;
        return;
    }

    m_ForwardMm += centerDistanceMm * std::cos(midpointHeading);
    m_LeftMm += centerDistanceMm * std::sin(midpointHeading);
    m_HeadingRad = normalizeRadians(m_HeadingRad + headingDeltaRad);

    if (elapsedMs != 0)
    {
        const double inverseSeconds = 1000.0 / static_cast<double>(elapsedMs);
        m_LinearVelocityMmPerSecond = centerDistanceMm * inverseSeconds;
        m_AngularVelocityRadPerSecond = headingDeltaRad * inverseSeconds;
    }
    else
    {
        m_LinearVelocityMmPerSecond = 0.0;
        m_AngularVelocityRadPerSecond = 0.0;
    }

    m_Finite = std::isfinite(m_ForwardMm)
        && std::isfinite(m_LeftMm)
        && std::isfinite(m_HeadingRad)
        && std::isfinite(m_LinearVelocityMmPerSecond)
        && std::isfinite(m_AngularVelocityRadPerSecond);
}
