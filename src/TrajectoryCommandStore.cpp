#include "TrajectoryCommandStore.hpp"

#include "TrajectoryConfig.hpp"

#include <cmath>

namespace
{
    bool hasFlag(const RobotProtocol::TrajectoryWaypoint& waypoint,
                 uint8_t flag)
    {
        return (waypoint.flags & flag) != 0;
    }
}

TrajectoryCommandStore::StoreResult TrajectoryCommandStore::store(
    const RobotProtocol::TrajectoryCommandPayload& command)
{
    if (!isSemanticallyValid(command))
        return StoreResult::REJECTED_INVALID;

    if (m_HasCommand)
    {
        return isSameCommand(m_Command, command)
            ? StoreResult::DUPLICATE_IGNORED
            : StoreResult::REJECTED_BUSY;
    }

    m_Command = command;
    m_HasCommand = true;
    return StoreResult::STORED;
}

void TrajectoryCommandStore::clear()
{
    m_Command = {};
    m_HasCommand = false;
}

const char* TrajectoryCommandStore::resultName(StoreResult result)
{
    switch (result)
    {
    case StoreResult::STORED:            return "STORED_PREVIEW_ONLY";
    case StoreResult::DUPLICATE_IGNORED: return "DUPLICATE_IGNORED";
    case StoreResult::REJECTED_INVALID:  return "REJECTED_INVALID";
    case StoreResult::REJECTED_BUSY:     return "REJECTED_BUSY";
    default:                             return "UNKNOWN";
    }
}

bool TrajectoryCommandStore::isSemanticallyValid(
    const RobotProtocol::TrajectoryCommandPayload& command)
{
    if (command.routeID == 0
        || command.startNodeID == 0
        || command.finalNodeID == 0
        || command.formatVersion != RobotProtocol::kTrajectoryFormatVersion
        || command.waypointCount == 0
        || command.waypointCount > RobotProtocol::kMaxTrajectoryWaypoints
        || !std::isfinite(command.millimetersPerMapUnit)
        || command.millimetersPerMapUnit <= 0.0f)
    {
        return false;
    }

    const RobotProtocol::TrajectoryWaypoint& first = command.waypoints[0];
    const RobotProtocol::TrajectoryWaypoint& last =
        command.waypoints[command.waypointCount - 1];
    if (std::hypot(first.forwardMm, first.leftMm)
            > TrajectoryConfig::kTrajectoryOriginToleranceMm
        || first.nodeID != command.startNodeID
        || !hasFlag(first, RobotProtocol::TRAJECTORY_FLAG_NODE_BOUNDARY)
        || last.nodeID != command.finalNodeID
        || !hasFlag(last, RobotProtocol::TRAJECTORY_FLAG_NODE_BOUNDARY)
        || !hasFlag(last, RobotProtocol::TRAJECTORY_FLAG_STOP)
        || !hasFlag(last, RobotProtocol::TRAJECTORY_FLAG_FINAL))
    {
        return false;
    }

    for (uint16_t i = 0; i < command.waypointCount; ++i)
    {
        const RobotProtocol::TrajectoryWaypoint& waypoint = command.waypoints[i];
        const bool nodeBoundary =
            hasFlag(waypoint, RobotProtocol::TRAJECTORY_FLAG_NODE_BOUNDARY);
        const bool rotateInPlace =
            hasFlag(waypoint, RobotProtocol::TRAJECTORY_FLAG_ROTATE_IN_PLACE);
        const bool final = hasFlag(waypoint, RobotProtocol::TRAJECTORY_FLAG_FINAL);

        if (nodeBoundary != (waypoint.nodeID != 0))
            return false;
        if (rotateInPlace
            && (nodeBoundary
                || waypoint.nodeID != 0
                || waypoint.targetSpeedMmPerSecond != 0.0f))
        {
            return false;
        }
        if (final != (i + 1 == command.waypointCount))
            return false;
    }

    return true;
}

bool TrajectoryCommandStore::isSameCommand(
    const RobotProtocol::TrajectoryCommandPayload& lhs,
    const RobotProtocol::TrajectoryCommandPayload& rhs)
{
    if (lhs.routeID != rhs.routeID
        || lhs.formatVersion != rhs.formatVersion
        || lhs.waypointCount != rhs.waypointCount
        || lhs.startNodeID != rhs.startNodeID
        || lhs.finalNodeID != rhs.finalNodeID
        || lhs.millimetersPerMapUnit != rhs.millimetersPerMapUnit)
    {
        return false;
    }

    for (uint16_t i = 0; i < lhs.waypointCount; ++i)
    {
        const RobotProtocol::TrajectoryWaypoint& left = lhs.waypoints[i];
        const RobotProtocol::TrajectoryWaypoint& right = rhs.waypoints[i];
        if (left.forwardMm != right.forwardMm
            || left.leftMm != right.leftMm
            || left.headingRad != right.headingRad
            || left.targetSpeedMmPerSecond != right.targetSpeedMmPerSecond
            || left.nodeID != right.nodeID
            || left.flags != right.flags)
        {
            return false;
        }
    }
    return true;
}
