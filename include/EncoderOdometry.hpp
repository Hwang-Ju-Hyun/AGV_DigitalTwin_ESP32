#pragma once

#include <cstdint>

class EncoderOdometry
{
public:
    struct Snapshot
    {
        float forwardMm = 0.0f;
        float leftMm = 0.0f;
        float headingRad = 0.0f;
        float linearVelocityMmPerSecond = 0.0f;
        float angularVelocityRadPerSecond = 0.0f;
        int32_t leftCount = 0;
        int32_t rightCount = 0;
        uint32_t encoderResetEpoch = 0;
        bool initialized = false;
        bool finite = true;
    };

    // Establishes a local (0, 0, 0) pose without changing encoder hardware.
    void resetLocal(int32_t leftCount,
                    int32_t rightCount,
                    uint32_t encoderResetEpoch,
                    uint32_t nowMs);

    // Consumes one atomic MotionController encoder snapshot. Encoder resets
    // are distinguished by epoch and do not erase the accumulated pose.
    void update(int32_t leftCount,
                int32_t rightCount,
                uint32_t encoderResetEpoch,
                uint32_t nowMs);

    Snapshot snapshot() const;

private:
    static double normalizeRadians(double radians);
    void integrate(int64_t leftDeltaCount,
                   int64_t rightDeltaCount,
                   uint32_t elapsedMs);

    double m_ForwardMm = 0.0;
    double m_LeftMm = 0.0;
    double m_HeadingRad = 0.0;
    double m_LinearVelocityMmPerSecond = 0.0;
    double m_AngularVelocityRadPerSecond = 0.0;
    int32_t m_LastLeftCount = 0;
    int32_t m_LastRightCount = 0;
    uint32_t m_LastEncoderResetEpoch = 0;
    uint32_t m_LastUpdateMs = 0;
    bool m_Initialized = false;
    bool m_Finite = true;
};
