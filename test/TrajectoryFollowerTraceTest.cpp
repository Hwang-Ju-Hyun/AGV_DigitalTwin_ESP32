#include "TrajectoryFollowerTrace.hpp"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <limits>

namespace
{
    using Command = RobotProtocol::TrajectoryCommandPayload;
    using Follower = TrajectoryFollowerTrace;
    using State = Follower::State;
    using Waypoint = RobotProtocol::TrajectoryWaypoint;

    constexpr float kTolerance = 1.0e-5f;

    uint8_t flags(uint8_t first, uint8_t second = 0, uint8_t third = 0)
    {
        return static_cast<uint8_t>(first | second | third);
    }

    Waypoint waypoint(float forwardMm,
                      float leftMm,
                      float headingRad,
                      float speedMmPerSecond,
                      uint32_t nodeID,
                      uint8_t waypointFlags)
    {
        Waypoint result;
        result.forwardMm = forwardMm;
        result.leftMm = leftMm;
        result.headingRad = headingRad;
        result.targetSpeedMmPerSecond = speedMmPerSecond;
        result.nodeID = nodeID;
        result.flags = waypointFlags;
        return result;
    }

    Waypoint finalWaypoint(float forwardMm, float leftMm, uint32_t nodeID)
    {
        return waypoint(
            forwardMm,
            leftMm,
            0.0f,
            0.0f,
            nodeID,
            flags(RobotProtocol::TRAJECTORY_FLAG_NODE_BOUNDARY,
                  RobotProtocol::TRAJECTORY_FLAG_STOP,
                  RobotProtocol::TRAJECTORY_FLAG_FINAL));
    }

    Command straightCommand()
    {
        Command command{};
        command.routeID = 7;
        command.formatVersion = RobotProtocol::kTrajectoryFormatVersion;
        command.waypointCount = 3;
        command.startNodeID = 1;
        command.finalNodeID = 4;
        command.millimetersPerMapUnit = 60.0f;
        command.waypoints[0] = waypoint(
            0.0f,
            0.0f,
            0.0f,
            120.0f,
            1,
            RobotProtocol::TRAJECTORY_FLAG_NODE_BOUNDARY);
        command.waypoints[1] = waypoint(
            60.0f,
            0.0f,
            0.0f,
            120.0f,
            0,
            RobotProtocol::TRAJECTORY_FLAG_NONE);
        command.waypoints[2] = finalWaypoint(120.0f, 0.0f, 4);
        return command;
    }

    Command twoPointCurve(float targetForwardMm, float targetLeftMm)
    {
        Command command = straightCommand();
        command.waypointCount = 2;
        command.waypoints[1] = finalWaypoint(
            targetForwardMm,
            targetLeftMm,
            command.finalNodeID);
        return command;
    }

    bool near(float left, float right)
    {
        return std::fabs(left - right) <= kTolerance;
    }

    State runUntilTerminal(Follower& follower, const Command& command)
    {
        State state = follower.snapshot().state;
        for (uint16_t step = 0;
             step <= RobotProtocol::kMaxTrajectoryWaypoints
             && state == State::ANALYZING;
             ++step)
        {
            state = follower.update(command);
        }
        assert(state != State::ANALYZING);
        return state;
    }

    void testStraightCompletesWithEqualWheelRatios()
    {
        const Command command = straightCommand();
        Follower follower;

        assert(follower.begin(command) == State::ANALYZING);
        assert(follower.update(command) == State::ANALYZING);

        const Follower::Snapshot first = follower.snapshot();
        assert(first.hasCurrentWheelRatio);
        assert(near(first.lastCurvaturePerMm, 0.0f));
        assert(near(first.lastLeftWheelRatio, first.lastRightWheelRatio));
        assert(near(first.lastLeftWheelRatio, 1.0f));
        assert(!first.reverseRequired);

        assert(runUntilTerminal(follower, command) == State::COMPLETE);
        const Follower::Snapshot complete = follower.snapshot();
        assert(complete.analyzedSampleCount == 2);
        assert(!complete.reverseRequired);
    }

    void testCurvatureSignsAndWheelRatios()
    {
        {
            const Command command = twoPointCurve(50.0f, 50.0f);
            Follower follower;
            assert(follower.begin(command) == State::ANALYZING);
            assert(follower.update(command) == State::ANALYZING);
            const Follower::Snapshot leftTurn = follower.snapshot();
            assert(leftTurn.lastCurvaturePerMm > 0.0f);
            assert(leftTurn.lastLeftWheelRatio < leftTurn.lastRightWheelRatio);
        }

        {
            const Command command = twoPointCurve(50.0f, -50.0f);
            Follower follower;
            assert(follower.begin(command) == State::ANALYZING);
            assert(follower.update(command) == State::ANALYZING);
            const Follower::Snapshot rightTurn = follower.snapshot();
            assert(rightTurn.lastCurvaturePerMm < 0.0f);
            assert(rightTurn.lastLeftWheelRatio > rightTurn.lastRightWheelRatio);
        }
    }

    void testTightCurveRequiresReverseWheel()
    {
        const Command command = twoPointCurve(0.0f, 50.0f);
        Follower follower;

        assert(follower.begin(command) == State::ANALYZING);
        assert(follower.update(command) == State::ANALYZING);
        const Follower::Snapshot snapshot = follower.snapshot();
        assert(snapshot.reverseRequired);
        assert(snapshot.minimumRawInnerWheelRatio < 0.0f);
        assert(snapshot.minimumTurnRadiusMm < 65.0f);
    }

    void testInteriorStopIsUnsupported()
    {
        Command command = straightCommand();
        command.waypoints[1].targetSpeedMmPerSecond = 0.0f;
        command.waypoints[1].nodeID = 2;
        command.waypoints[1].flags = flags(
            RobotProtocol::TRAJECTORY_FLAG_NODE_BOUNDARY,
            RobotProtocol::TRAJECTORY_FLAG_STOP);

        Follower follower;
        assert(follower.begin(command) == State::ANALYZING);
        assert(runUntilTerminal(follower, command) == State::UNSUPPORTED_STOP);
    }

    void testRotateInPlaceIsUnsupported()
    {
        Command command = straightCommand();
        command.waypoints[1].targetSpeedMmPerSecond = 0.0f;
        command.waypoints[1].nodeID = 0;
        command.waypoints[1].flags =
            RobotProtocol::TRAJECTORY_FLAG_ROTATE_IN_PLACE;

        Follower follower;
        assert(follower.begin(command) == State::ANALYZING);
        assert(runUntilTerminal(follower, command) == State::UNSUPPORTED_ROTATE);
    }

    void testCommandIdentityChangeTerminates()
    {
        const Command original = straightCommand();
        Command changed = original;
        ++changed.routeID;

        Follower follower;
        assert(follower.begin(original) == State::ANALYZING);
        assert(follower.update(changed) == State::COMMAND_CHANGED);
        assert(!follower.snapshot().hasCurrentWheelRatio);
    }

    void testDuplicateWaypointFailsWithinBound()
    {
        Command command = twoPointCurve(0.0f, 0.0f);
        Follower follower;

        assert(follower.begin(command) == State::ANALYZING);
        assert(runUntilTerminal(follower, command) == State::INVALID);
        assert(follower.snapshot().analyzedSampleCount == 0);
    }

    void testNanWaypointFailsWithinBound()
    {
        Command command = straightCommand();
        command.waypoints[1].leftMm =
            std::numeric_limits<float>::quiet_NaN();

        Follower follower;
        assert(follower.begin(command) == State::ANALYZING);
        assert(runUntilTerminal(follower, command) == State::INVALID);
        assert(follower.snapshot().analyzedSampleCount == 0);
    }

    void testZeroSpeedPreviewRemainsGeometryOnly()
    {
        Command command = straightCommand();
        for (uint16_t index = 0; index < command.waypointCount; ++index)
            command.waypoints[index].targetSpeedMmPerSecond = 0.0f;

        Follower follower;
        assert(follower.begin(command) == State::ANALYZING);
        assert(runUntilTerminal(follower, command) == State::COMPLETE);
    }

    void testScaleMetadataDoesNotRescaleMillimetres()
    {
        Command scale25 = twoPointCurve(100.0f, -50.0f);
        scale25.millimetersPerMapUnit = 25.0f;
        Command scale60 = scale25;
        scale60.millimetersPerMapUnit = 60.0f;

        Follower follower25;
        Follower follower60;
        assert(follower25.begin(scale25) == State::ANALYZING);
        assert(follower60.begin(scale60) == State::ANALYZING);
        assert(follower25.update(scale25) == State::ANALYZING);
        assert(follower60.update(scale60) == State::ANALYZING);

        const Follower::Snapshot left = follower25.snapshot();
        const Follower::Snapshot right = follower60.snapshot();
        assert(near(left.lastCurvaturePerMm, right.lastCurvaturePerMm));
        assert(near(left.lastLeftWheelRatio, right.lastLeftWheelRatio));
        assert(near(left.lastRightWheelRatio, right.lastRightWheelRatio));
    }

    void testNonzeroSourceHeadingTransformsTarget()
    {
        Command command = twoPointCurve(0.0f, 50.0f);
        command.waypoints[0].headingRad = 1.57079632679f;

        Follower follower;
        assert(follower.begin(command) == State::ANALYZING);
        assert(follower.update(command) == State::ANALYZING);
        const Follower::Snapshot snapshot = follower.snapshot();
        assert(std::fabs(snapshot.lastCurvaturePerMm) < 1.0e-5f);
        assert(near(snapshot.lastLeftWheelRatio, snapshot.lastRightWheelRatio));
    }

    void testMaximumWaypointCountTerminatesWithinBound()
    {
        Command command{};
        command.routeID = 88;
        command.formatVersion = RobotProtocol::kTrajectoryFormatVersion;
        command.waypointCount = RobotProtocol::kMaxTrajectoryWaypoints;
        command.startNodeID = 1;
        command.finalNodeID = 4;
        command.millimetersPerMapUnit = 60.0f;

        for (uint16_t index = 0; index < command.waypointCount; ++index)
        {
            command.waypoints[index] = waypoint(
                static_cast<float>(index) * 10.0f,
                0.0f,
                0.0f,
                0.0f,
                0,
                RobotProtocol::TRAJECTORY_FLAG_NONE);
        }
        command.waypoints[0].nodeID = command.startNodeID;
        command.waypoints[0].flags =
            RobotProtocol::TRAJECTORY_FLAG_NODE_BOUNDARY;
        command.waypoints[command.waypointCount - 1] = finalWaypoint(
            static_cast<float>(command.waypointCount - 1) * 10.0f,
            0.0f,
            command.finalNodeID);

        Follower follower;
        assert(follower.begin(command) == State::ANALYZING);
        assert(runUntilTerminal(follower, command) == State::COMPLETE);
        assert(follower.snapshot().analyzedSampleCount
               == command.waypointCount - 1);
    }
}

int main()
{
    testStraightCompletesWithEqualWheelRatios();
    testCurvatureSignsAndWheelRatios();
    testTightCurveRequiresReverseWheel();
    testInteriorStopIsUnsupported();
    testRotateInPlaceIsUnsupported();
    testCommandIdentityChangeTerminates();
    testDuplicateWaypointFailsWithinBound();
    testNanWaypointFailsWithinBound();
    testZeroSpeedPreviewRemainsGeometryOnly();
    testScaleMetadataDoesNotRescaleMillimetres();
    testNonzeroSourceHeadingTransformsTarget();
    testMaximumWaypointCountTerminatesWithinBound();
    return 0;
}
