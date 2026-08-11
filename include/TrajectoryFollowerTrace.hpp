#pragma once

#include "RobotProtocol.hpp"

#include <cstdint>

// Geometry-only, non-driving Pure Pursuit trace. This class never owns a
// trajectory, never retains a reference to TrajectoryCommandStore, and has no
// access to motor, PWM, GPIO, Arduino, or timing APIs.
class TrajectoryFollowerTrace
{
public:
    static constexpr float kLookaheadDistanceMm = 45.0f;
    static constexpr float kHalfTrackWidthMm = 65.0f;

    enum class State : uint8_t
    {
        EMPTY,
        ANALYZING,
        COMPLETE,
        UNSUPPORTED_STOP,
        UNSUPPORTED_ROTATE,
        INVALID,
        COMMAND_CHANGED
    };

    struct Snapshot
    {
        State state = State::EMPTY;
        uint32_t routeID = 0;
        uint16_t waypointCount = 0;
        uint16_t nextSourceIndex = 0;
        uint16_t analyzedSampleCount = 0;
        uint16_t lastSourceIndex = 0;
        uint16_t lastTargetIndex = 0;

        float lastLookaheadDistanceMm = 0.0f;
        float lastCurvaturePerMm = 0.0f;
        float lastTurnRadiusMm = 0.0f;
        float lastLeftWheelRatio = 0.0f;
        float lastRightWheelRatio = 0.0f;

        float minimumTurnRadiusMm = 0.0f;
        float minimumRawInnerWheelRatio = 1.0f;
        float maximumAbsoluteCurvaturePerMm = 0.0f;

        bool hasCurrentWheelRatio = false;
        bool hasCurvedSample = false;
        bool reverseRequired = false;
    };

    State begin(const RobotProtocol::TrajectoryCommandPayload& command);

    // Processes no more than one source waypoint. The bounded lookahead search
    // may inspect at most the protocol maximum of 64 waypoint records.
    State update(const RobotProtocol::TrajectoryCommandPayload& command);

    void reset();
    Snapshot snapshot() const;

    static const char* stateName(State state);

private:
    bool commandIdentityMatches(
        const RobotProtocol::TrajectoryCommandPayload& command) const;
    void enterTerminalState(State state);

    State m_State = State::EMPTY;
    uint32_t m_RouteID = 0;
    uint8_t m_FormatVersion = 0;
    uint16_t m_WaypointCount = 0;
    uint32_t m_StartNodeID = 0;
    uint32_t m_FinalNodeID = 0;
    uint16_t m_NextSourceIndex = 0;
    uint16_t m_AnalyzedSampleCount = 0;
    uint16_t m_LastSourceIndex = 0;
    uint16_t m_LastTargetIndex = 0;

    float m_LastLookaheadDistanceMm = 0.0f;
    float m_LastCurvaturePerMm = 0.0f;
    float m_LastTurnRadiusMm = 0.0f;
    float m_LastLeftWheelRatio = 0.0f;
    float m_LastRightWheelRatio = 0.0f;
    float m_MinimumTurnRadiusMm = 0.0f;
    float m_MinimumRawInnerWheelRatio = 1.0f;
    float m_MaximumAbsoluteCurvaturePerMm = 0.0f;
    bool m_HasCurrentWheelRatio = false;
    bool m_HasCurvedSample = false;
    bool m_ReverseRequired = false;
};
