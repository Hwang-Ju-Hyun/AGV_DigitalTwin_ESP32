#include "TrajectoryFollowerTrace.hpp"

#include <algorithm>
#include <cmath>

namespace
{
    constexpr double kLookaheadSquaredMm =
        static_cast<double>(TrajectoryFollowerTrace::kLookaheadDistanceMm)
        * TrajectoryFollowerTrace::kLookaheadDistanceMm;
    constexpr double kDistanceSquaredEpsilon = 1.0e-6;
    constexpr double kCurvatureEpsilon = 1.0e-9;

    constexpr uint8_t kKnownFlags =
        RobotProtocol::TRAJECTORY_FLAG_NODE_BOUNDARY
        | RobotProtocol::TRAJECTORY_FLAG_STOP
        | RobotProtocol::TRAJECTORY_FLAG_ROTATE_IN_PLACE
        | RobotProtocol::TRAJECTORY_FLAG_FINAL;

    bool hasFlag(const RobotProtocol::TrajectoryWaypoint& waypoint,
                 uint8_t flag)
    {
        return (waypoint.flags & flag) != 0;
    }

    bool waypointIsFinite(const RobotProtocol::TrajectoryWaypoint& waypoint)
    {
        return std::isfinite(waypoint.forwardMm)
            && std::isfinite(waypoint.leftMm)
            && std::isfinite(waypoint.headingRad)
            && std::isfinite(waypoint.targetSpeedMmPerSecond)
            && waypoint.targetSpeedMmPerSecond >= 0.0f
            && (waypoint.flags & static_cast<uint8_t>(~kKnownFlags)) == 0;
    }
}

TrajectoryFollowerTrace::State TrajectoryFollowerTrace::begin(
    const RobotProtocol::TrajectoryCommandPayload& command)
{
    reset();

    if (command.routeID == 0
        || command.formatVersion != RobotProtocol::kTrajectoryFormatVersion
        || command.waypointCount < 2
        || command.waypointCount > RobotProtocol::kMaxTrajectoryWaypoints
        || command.startNodeID == 0
        || command.finalNodeID == 0
        || !std::isfinite(command.millimetersPerMapUnit)
        || command.millimetersPerMapUnit <= 0.0f)
    {
        m_State = State::INVALID;
        return m_State;
    }

    const RobotProtocol::TrajectoryWaypoint& last =
        command.waypoints[command.waypointCount - 1];
    if (!waypointIsFinite(last)
        || !hasFlag(last, RobotProtocol::TRAJECTORY_FLAG_STOP)
        || !hasFlag(last, RobotProtocol::TRAJECTORY_FLAG_FINAL))
    {
        m_State = State::INVALID;
        return m_State;
    }

    m_RouteID = command.routeID;
    m_FormatVersion = command.formatVersion;
    m_WaypointCount = command.waypointCount;
    m_StartNodeID = command.startNodeID;
    m_FinalNodeID = command.finalNodeID;
    m_State = State::ANALYZING;
    return m_State;
}

TrajectoryFollowerTrace::State TrajectoryFollowerTrace::update(
    const RobotProtocol::TrajectoryCommandPayload& command)
{
    if (m_State != State::ANALYZING)
        return m_State;

    if (!commandIdentityMatches(command))
    {
        enterTerminalState(State::COMMAND_CHANGED);
        return m_State;
    }

    if (!std::isfinite(command.millimetersPerMapUnit)
        || command.millimetersPerMapUnit <= 0.0f)
    {
        enterTerminalState(State::INVALID);
        return m_State;
    }

    if (m_NextSourceIndex >= m_WaypointCount)
    {
        enterTerminalState(State::COMPLETE);
        return m_State;
    }

    const uint16_t sourceIndex = m_NextSourceIndex;
    const RobotProtocol::TrajectoryWaypoint& source =
        command.waypoints[sourceIndex];
    if (!waypointIsFinite(source))
    {
        enterTerminalState(State::INVALID);
        return m_State;
    }

    const bool isFinalSource = sourceIndex + 1 == m_WaypointCount;
    if (hasFlag(source, RobotProtocol::TRAJECTORY_FLAG_FINAL)
        != isFinalSource)
    {
        enterTerminalState(State::INVALID);
        return m_State;
    }
    if (hasFlag(source, RobotProtocol::TRAJECTORY_FLAG_ROTATE_IN_PLACE))
    {
        enterTerminalState(State::UNSUPPORTED_ROTATE);
        return m_State;
    }
    if (!isFinalSource
        && hasFlag(source, RobotProtocol::TRAJECTORY_FLAG_STOP))
    {
        enterTerminalState(State::UNSUPPORTED_STOP);
        return m_State;
    }

    // The final waypoint is a terminal marker, not a new Pure Pursuit source
    // sample. It was already checked for FINAL|STOP in begin().
    if (isFinalSource)
    {
        ++m_NextSourceIndex;
        enterTerminalState(State::COMPLETE);
        return m_State;
    }

    uint16_t targetIndex = 0;
    double targetDistanceSquared = 0.0;
    bool targetFound = false;

    // This loop is bounded by kMaxTrajectoryWaypoints (64). It also refuses to
    // look through a stop or rotate marker that Pure Pursuit cannot represent.
    for (uint16_t candidateIndex = sourceIndex + 1;
         candidateIndex < m_WaypointCount;
         ++candidateIndex)
    {
        const RobotProtocol::TrajectoryWaypoint& candidate =
            command.waypoints[candidateIndex];
        if (!waypointIsFinite(candidate))
        {
            enterTerminalState(State::INVALID);
            return m_State;
        }

        const bool candidateIsFinal =
            candidateIndex + 1 == m_WaypointCount;
        if (hasFlag(candidate, RobotProtocol::TRAJECTORY_FLAG_FINAL)
            != candidateIsFinal)
        {
            enterTerminalState(State::INVALID);
            return m_State;
        }
        if (!candidateIsFinal
            && hasFlag(candidate,
                       RobotProtocol::TRAJECTORY_FLAG_ROTATE_IN_PLACE))
        {
            enterTerminalState(State::UNSUPPORTED_ROTATE);
            return m_State;
        }
        if (!candidateIsFinal
            && hasFlag(candidate, RobotProtocol::TRAJECTORY_FLAG_STOP))
        {
            enterTerminalState(State::UNSUPPORTED_STOP);
            return m_State;
        }

        const double deltaForward =
            static_cast<double>(candidate.forwardMm) - source.forwardMm;
        const double deltaLeft =
            static_cast<double>(candidate.leftMm) - source.leftMm;
        const double distanceSquared =
            deltaForward * deltaForward + deltaLeft * deltaLeft;
        if (!std::isfinite(distanceSquared))
        {
            enterTerminalState(State::INVALID);
            return m_State;
        }

        if (distanceSquared >= kLookaheadSquaredMm || candidateIsFinal)
        {
            targetIndex = candidateIndex;
            targetDistanceSquared = distanceSquared;
            targetFound = true;
            break;
        }
    }

    if (!targetFound || targetDistanceSquared <= kDistanceSquaredEpsilon)
    {
        enterTerminalState(State::INVALID);
        return m_State;
    }

    const RobotProtocol::TrajectoryWaypoint& target =
        command.waypoints[targetIndex];
    const double deltaForward =
        static_cast<double>(target.forwardMm) - source.forwardMm;
    const double deltaLeft =
        static_cast<double>(target.leftMm) - source.leftMm;
    const double heading = static_cast<double>(source.headingRad);
    const double cosine = std::cos(heading);
    const double sine = std::sin(heading);
    const double targetLeftInSourceFrame =
        -sine * deltaForward + cosine * deltaLeft;
    const double curvature =
        2.0 * targetLeftInSourceFrame / targetDistanceSquared;
    const double absoluteCurvature = std::abs(curvature);
    const double rawLeftRatio =
        1.0 - curvature * TrajectoryFollowerTrace::kHalfTrackWidthMm;
    const double rawRightRatio =
        1.0 + curvature * TrajectoryFollowerTrace::kHalfTrackWidthMm;
    const double rawInnerRatio = std::min(rawLeftRatio, rawRightRatio);
    const double normalization =
        std::max(1.0, std::max(std::abs(rawLeftRatio),
                               std::abs(rawRightRatio)));
    const double normalizedLeftRatio = rawLeftRatio / normalization;
    const double normalizedRightRatio = rawRightRatio / normalization;
    const double lookaheadDistance = std::sqrt(targetDistanceSquared);
    const bool curved = absoluteCurvature > kCurvatureEpsilon;
    const double turnRadius = curved ? 1.0 / absoluteCurvature : 0.0;

    if (!std::isfinite(cosine)
        || !std::isfinite(sine)
        || !std::isfinite(targetLeftInSourceFrame)
        || !std::isfinite(curvature)
        || !std::isfinite(rawLeftRatio)
        || !std::isfinite(rawRightRatio)
        || !std::isfinite(rawInnerRatio)
        || !std::isfinite(normalization)
        || normalization <= 0.0
        || !std::isfinite(normalizedLeftRatio)
        || !std::isfinite(normalizedRightRatio)
        || !std::isfinite(lookaheadDistance)
        || !std::isfinite(turnRadius))
    {
        enterTerminalState(State::INVALID);
        return m_State;
    }

    m_LastSourceIndex = sourceIndex;
    m_LastTargetIndex = targetIndex;
    m_LastLookaheadDistanceMm = static_cast<float>(lookaheadDistance);
    m_LastCurvaturePerMm = static_cast<float>(curvature);
    m_LastTurnRadiusMm = static_cast<float>(turnRadius);
    m_LastLeftWheelRatio = static_cast<float>(normalizedLeftRatio);
    m_LastRightWheelRatio = static_cast<float>(normalizedRightRatio);
    m_HasCurrentWheelRatio = true;

    m_MinimumRawInnerWheelRatio = static_cast<float>(
        std::min(static_cast<double>(m_MinimumRawInnerWheelRatio),
                 rawInnerRatio));
    m_MaximumAbsoluteCurvaturePerMm = static_cast<float>(
        std::max(static_cast<double>(m_MaximumAbsoluteCurvaturePerMm),
                 absoluteCurvature));
    m_ReverseRequired = m_ReverseRequired || rawInnerRatio < 0.0;

    if (curved)
    {
        if (!m_HasCurvedSample || turnRadius < m_MinimumTurnRadiusMm)
            m_MinimumTurnRadiusMm = static_cast<float>(turnRadius);
        m_HasCurvedSample = true;
    }

    ++m_AnalyzedSampleCount;
    ++m_NextSourceIndex;
    return m_State;
}

void TrajectoryFollowerTrace::reset()
{
    m_State = State::EMPTY;
    m_RouteID = 0;
    m_FormatVersion = 0;
    m_WaypointCount = 0;
    m_StartNodeID = 0;
    m_FinalNodeID = 0;
    m_NextSourceIndex = 0;
    m_AnalyzedSampleCount = 0;
    m_LastSourceIndex = 0;
    m_LastTargetIndex = 0;
    m_LastLookaheadDistanceMm = 0.0f;
    m_LastCurvaturePerMm = 0.0f;
    m_LastTurnRadiusMm = 0.0f;
    m_LastLeftWheelRatio = 0.0f;
    m_LastRightWheelRatio = 0.0f;
    m_MinimumTurnRadiusMm = 0.0f;
    m_MinimumRawInnerWheelRatio = 1.0f;
    m_MaximumAbsoluteCurvaturePerMm = 0.0f;
    m_HasCurrentWheelRatio = false;
    m_HasCurvedSample = false;
    m_ReverseRequired = false;
}

TrajectoryFollowerTrace::Snapshot TrajectoryFollowerTrace::snapshot() const
{
    Snapshot result;
    result.state = m_State;
    result.routeID = m_RouteID;
    result.waypointCount = m_WaypointCount;
    result.nextSourceIndex = m_NextSourceIndex;
    result.analyzedSampleCount = m_AnalyzedSampleCount;
    result.lastSourceIndex = m_LastSourceIndex;
    result.lastTargetIndex = m_LastTargetIndex;
    result.lastLookaheadDistanceMm = m_LastLookaheadDistanceMm;
    result.lastCurvaturePerMm = m_LastCurvaturePerMm;
    result.lastTurnRadiusMm = m_LastTurnRadiusMm;
    result.lastLeftWheelRatio = m_LastLeftWheelRatio;
    result.lastRightWheelRatio = m_LastRightWheelRatio;
    result.minimumTurnRadiusMm = m_MinimumTurnRadiusMm;
    result.minimumRawInnerWheelRatio = m_MinimumRawInnerWheelRatio;
    result.maximumAbsoluteCurvaturePerMm = m_MaximumAbsoluteCurvaturePerMm;
    result.hasCurrentWheelRatio = m_HasCurrentWheelRatio;
    result.hasCurvedSample = m_HasCurvedSample;
    result.reverseRequired = m_ReverseRequired;
    return result;
}

const char* TrajectoryFollowerTrace::stateName(State state)
{
    switch (state)
    {
    case State::EMPTY:              return "EMPTY";
    case State::ANALYZING:          return "ANALYZING";
    case State::COMPLETE:           return "COMPLETE";
    case State::UNSUPPORTED_STOP:   return "UNSUPPORTED_STOP";
    case State::UNSUPPORTED_ROTATE: return "UNSUPPORTED_ROTATE";
    case State::INVALID:            return "INVALID";
    case State::COMMAND_CHANGED:    return "COMMAND_CHANGED";
    default:                        return "UNKNOWN";
    }
}

bool TrajectoryFollowerTrace::commandIdentityMatches(
    const RobotProtocol::TrajectoryCommandPayload& command) const
{
    return command.routeID == m_RouteID
        && command.formatVersion == m_FormatVersion
        && command.waypointCount == m_WaypointCount
        && command.startNodeID == m_StartNodeID
        && command.finalNodeID == m_FinalNodeID;
}

void TrajectoryFollowerTrace::enterTerminalState(State state)
{
    m_State = state;
    m_LastLookaheadDistanceMm = 0.0f;
    m_LastCurvaturePerMm = 0.0f;
    m_LastTurnRadiusMm = 0.0f;
    m_LastLeftWheelRatio = 0.0f;
    m_LastRightWheelRatio = 0.0f;
    m_HasCurrentWheelRatio = false;
}
