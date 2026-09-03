#include "PhysicalFleetExecutor.hpp"
#include "Config.hpp"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <vector>

namespace
{
    constexpr float kPi = 3.14159265358979323846f;
    constexpr float kHalfPi = 1.57079632679489661923f;
    constexpr float kFloatTolerance = 1.0e-5f;

    constexpr float radians(float degrees)
    {
        return degrees * kPi / 180.0f;
    }

    struct StartCall
    {
        MotionController::Mode mode = MotionController::Mode::NONE;
        int32_t targetCount = 0;
        uint32_t nowMs = 0;
        MotionController::Profile profile = MotionController::Profile::NORMAL;
    };

    struct FakeMotionControl
    {
        MotionController::StartResult startResult =
            MotionController::StartResult::STARTED;
        MotionController::UpdateResult updateResult =
            MotionController::UpdateResult::RUNNING;
        MotionController::Fault injectedFault =
            MotionController::Fault::STALL;
        bool forceUnsafe = false;
        uint32_t stopCount = 0;
        int32_t leftProgress = 0;
        int32_t rightProgress = 0;
        int32_t leftTarget = 0;
        int32_t rightTarget = 0;
        std::vector<StartCall> starts;
    };

    FakeMotionControl g_Fake;

    void resetFake()
    {
        g_Fake = {};
    }

    uint8_t flags(uint8_t first, uint8_t second = 0, uint8_t third = 0)
    {
        return static_cast<uint8_t>(first | second | third);
    }

    RobotProtocol::TrajectoryWaypoint waypoint(
        float forwardMm,
        float leftMm,
        float headingRad,
        float speedMmPerSecond,
        uint32_t nodeID,
        uint8_t waypointFlags)
    {
        RobotProtocol::TrajectoryWaypoint result;
        result.forwardMm = forwardMm;
        result.leftMm = leftMm;
        result.headingRad = headingRad;
        result.targetSpeedMmPerSecond = speedMmPerSecond;
        result.nodeID = nodeID;
        result.flags = waypointFlags;
        return result;
    }

    RobotProtocol::TrajectoryCommandPayload oneEdgeCommand(
        uint32_t routeID,
        uint32_t startNodeID,
        uint32_t finalNodeID,
        float distanceMm = 100.0f)
    {
        RobotProtocol::TrajectoryCommandPayload command{};
        command.routeID = routeID;
        command.formatVersion = RobotProtocol::kTrajectoryFormatVersion;
        command.waypointCount = 2;
        command.startNodeID = startNodeID;
        command.finalNodeID = finalNodeID;
        command.millimetersPerMapUnit = 50.0f;
        command.waypoints[0] = waypoint(
            0.0f,
            0.0f,
            0.0f,
            80.0f,
            startNodeID,
            RobotProtocol::TRAJECTORY_FLAG_NODE_BOUNDARY);
        command.waypoints[1] = waypoint(
            distanceMm,
            0.0f,
            0.0f,
            0.0f,
            finalNodeID,
            flags(RobotProtocol::TRAJECTORY_FLAG_NODE_BOUNDARY,
                  RobotProtocol::TRAJECTORY_FLAG_STOP,
                  RobotProtocol::TRAJECTORY_FLAG_FINAL));
        return command;
    }

    RobotProtocol::NodeCorrectionCommandPayload correction(
        uint32_t routeID,
        uint32_t nodeID,
        uint32_t commandID,
        RobotProtocol::NodeCorrectionAction action,
        float magnitude)
    {
        RobotProtocol::NodeCorrectionCommandPayload command;
        command.routeID = routeID;
        command.nodeID = nodeID;
        command.commandID = commandID;
        command.action = action;
        command.magnitude = magnitude;
        return command;
    }

    void completeCurrentMotion(PhysicalFleetExecutor& executor,
                               uint32_t nowMs)
    {
        g_Fake.updateResult = MotionController::UpdateResult::COMPLETE;
        executor.update(nowMs, true);
        g_Fake.updateResult = MotionController::UpdateResult::RUNNING;
    }

    void enterNodeWait(PhysicalFleetExecutor& executor,
                       uint32_t routeID,
                       uint32_t startNodeID,
                       uint32_t finalNodeID)
    {
        const auto trajectory = oneEdgeCommand(
            routeID, startNodeID, finalNodeID);
        assert(executor.acceptTrajectory(trajectory, true, 10)
               == PhysicalFleetExecutor::AcceptResult::STORED);
        executor.update(10, true);
        assert(executor.state() == PhysicalFleetExecutor::State::RUNNING);
        assert(g_Fake.starts.back().profile
               == MotionController::Profile::PHYSICAL_FLEET);
        completeCurrentMotion(executor, 20);

        uint32_t arrivalNodeID = 0;
        assert(executor.arrivalPending(arrivalNodeID));
        assert(arrivalNodeID == finalNodeID);
        executor.markArrivedSendResult(true, 21);
        assert(executor.state() == PhysicalFleetExecutor::State::NODE_WAIT);
        assert(!executor.hasActiveCommand());
    }

    RobotProtocol::NodeCorrectionReportPayload completeCorrection(
        PhysicalFleetExecutor& executor,
        uint32_t nowMs)
    {
        completeCurrentMotion(executor, nowMs);
        RobotProtocol::NodeCorrectionReportPayload report;
        assert(executor.correctionReportPending(report));
        assert(report.result
               == RobotProtocol::NodeCorrectionResult::COMPLETED);
        assert(report.detail == 0);
        executor.markCorrectionReportSendResult(true);
        assert(executor.state() == PhysicalFleetExecutor::State::NODE_WAIT);
        assert(!executor.correctionReportPending(report));
        return report;
    }

    void testOneEdgeArrivalCannotAutoResume()
    {
        resetFake();
        MotionController motion;
        PhysicalFleetExecutor executor(motion);
        executor.begin(1, 0.0f);
        enterNodeWait(executor, 7, 1, 2);

        const size_t startCount = g_Fake.starts.size();
        executor.update(60000, true);
        assert(executor.state() == PhysicalFleetExecutor::State::NODE_WAIT);
        assert(g_Fake.starts.size() == startCount);
        assert(motion.outputsSafe());
    }

    void testMultiEdgeTrajectoryIsRejected()
    {
        resetFake();
        MotionController motion;
        PhysicalFleetExecutor executor(motion);
        executor.begin(1, 0.0f);

        auto command = oneEdgeCommand(8, 1, 3, 200.0f);
        command.waypointCount = 3;
        command.waypoints[1] = waypoint(
            100.0f,
            0.0f,
            0.0f,
            80.0f,
            2,
            flags(RobotProtocol::TRAJECTORY_FLAG_NODE_BOUNDARY,
                  RobotProtocol::TRAJECTORY_FLAG_STOP));
        command.waypoints[2] = waypoint(
            200.0f,
            0.0f,
            0.0f,
            0.0f,
            3,
            flags(RobotProtocol::TRAJECTORY_FLAG_NODE_BOUNDARY,
                  RobotProtocol::TRAJECTORY_FLAG_STOP,
                  RobotProtocol::TRAJECTORY_FLAG_FINAL));

        assert(executor.acceptTrajectory(command, true, 0)
               == PhysicalFleetExecutor::AcceptResult::REJECTED_INVALID);
        assert(executor.state()
               == PhysicalFleetExecutor::State::FAULT_LATCHED);
        assert(g_Fake.starts.empty());
        assert(motion.outputsSafe());
    }

    void testCorrectedNodeResidualHeadingWithinTenDegreesIsStored()
    {
        resetFake();
        MotionController motion;
        PhysicalFleetExecutor executor(motion);
        executor.begin(3, 0.0f);

        // Reproduces the Node 3 -> 4 command after Vision correction left a
        // -5.7605 degree start-heading residual. The nominal edge is straight.
        auto command = oneEdgeCommand(34, 3, 4, 350.0f);
        command.waypoints[0].headingRad = radians(-5.7605f);

        assert(executor.acceptTrajectory(command, true, 10)
               == PhysicalFleetExecutor::AcceptResult::STORED);
        assert(executor.state() == PhysicalFleetExecutor::State::READY);
        assert(executor.hasActiveCommand());
        assert(g_Fake.starts.empty());
        assert(motion.outputsSafe());
    }

    void testStraightHeadingOverTenDegreesIsRejectedSafely()
    {
        resetFake();
        MotionController motion;
        PhysicalFleetExecutor executor(motion);
        executor.begin(3, 0.0f);

        const float rejectedHeading = radians(10.5f);
        auto command = oneEdgeCommand(35, 3, 4, 350.0f);
        command.waypoints[1].forwardMm =
            350.0f * std::cos(rejectedHeading);
        command.waypoints[1].leftMm =
            350.0f * std::sin(rejectedHeading);

        assert(executor.acceptTrajectory(command, true, 10)
               == PhysicalFleetExecutor::AcceptResult::REJECTED_INVALID);
        assert(executor.state()
               == PhysicalFleetExecutor::State::FAULT_LATCHED);
        assert(executor.fault()
               == PhysicalFleetExecutor::Fault::INVALID_COMMAND);
        assert(g_Fake.starts.empty());
        assert(motion.outputsSafe());
    }

    void testRotateHeadingWithinTenDegreesIsStored()
    {
        for (const float nominalDegrees : {90.0f, 180.0f})
        {
            resetFake();
            MotionController motion;
            PhysicalFleetExecutor executor(motion);
            executor.begin(3, 0.0f);

            const float rotateHeading = radians(nominalDegrees + 10.0f);
            auto command = oneEdgeCommand(36, 3, 4, 100.0f);
            command.waypointCount = 3;
            command.waypoints[1] = waypoint(
                0.0f,
                0.0f,
                rotateHeading,
                0.0f,
                0,
                RobotProtocol::TRAJECTORY_FLAG_ROTATE_IN_PLACE);
            command.waypoints[2] = waypoint(
                100.0f * std::cos(rotateHeading),
                100.0f * std::sin(rotateHeading),
                rotateHeading,
                0.0f,
                4,
                flags(RobotProtocol::TRAJECTORY_FLAG_NODE_BOUNDARY,
                      RobotProtocol::TRAJECTORY_FLAG_STOP,
                      RobotProtocol::TRAJECTORY_FLAG_FINAL));

            assert(executor.acceptTrajectory(command, true, 10)
                   == PhysicalFleetExecutor::AcceptResult::STORED);
            assert(executor.state() == PhysicalFleetExecutor::State::READY);
            assert(g_Fake.starts.empty());
            assert(motion.outputsSafe());
        }
    }

    void testRotateHeadingOverTenDegreesIsRejectedSafely()
    {
        resetFake();
        MotionController motion;
        PhysicalFleetExecutor executor(motion);
        executor.begin(3, 0.0f);

        const float rotateHeading = radians(100.5f);
        auto command = oneEdgeCommand(37, 3, 4, 100.0f);
        command.waypointCount = 3;
        command.waypoints[1] = waypoint(
            0.0f,
            0.0f,
            rotateHeading,
            0.0f,
            0,
            RobotProtocol::TRAJECTORY_FLAG_ROTATE_IN_PLACE);
        command.waypoints[2] = waypoint(
            100.0f * std::cos(rotateHeading),
            100.0f * std::sin(rotateHeading),
            rotateHeading,
            0.0f,
            4,
            flags(RobotProtocol::TRAJECTORY_FLAG_NODE_BOUNDARY,
                  RobotProtocol::TRAJECTORY_FLAG_STOP,
                  RobotProtocol::TRAJECTORY_FLAG_FINAL));

        assert(executor.acceptTrajectory(command, true, 10)
               == PhysicalFleetExecutor::AcceptResult::REJECTED_INVALID);
        assert(executor.state()
               == PhysicalFleetExecutor::State::FAULT_LATCHED);
        assert(executor.fault()
               == PhysicalFleetExecutor::Fault::INVALID_COMMAND);
        assert(g_Fake.starts.empty());
        assert(motion.outputsSafe());
    }

    void testOneEdgeMayContainRotationMarker()
    {
        resetFake();
        MotionController motion;
        PhysicalFleetExecutor executor(motion);
        executor.begin(1, 0.0f);

        auto command = oneEdgeCommand(9, 1, 4);
        command.waypointCount = 3;
        command.waypoints[1] = waypoint(
            0.0f,
            0.0f,
            kHalfPi,
            0.0f,
            0,
            RobotProtocol::TRAJECTORY_FLAG_ROTATE_IN_PLACE);
        command.waypoints[2] = waypoint(
            0.0f,
            100.0f,
            kHalfPi,
            0.0f,
            4,
            flags(RobotProtocol::TRAJECTORY_FLAG_NODE_BOUNDARY,
                  RobotProtocol::TRAJECTORY_FLAG_STOP,
                  RobotProtocol::TRAJECTORY_FLAG_FINAL));

        assert(executor.acceptTrajectory(command, true, 10)
               == PhysicalFleetExecutor::AcceptResult::STORED);
        executor.update(10, true);
        assert(g_Fake.starts.back().mode == MotionController::Mode::TURN_CCW);
        assert(g_Fake.starts.back().targetCount
               == AppConfig::kTurn90CcwCount);
        assert(g_Fake.starts.back().profile
               == MotionController::Profile::PHYSICAL_FLEET);
        completeCurrentMotion(executor, 20);
        assert(executor.state() == PhysicalFleetExecutor::State::SAFE_PAUSE);
        executor.update(519, true);
        assert(g_Fake.starts.size() == 1);
        executor.update(520, true);
        assert(g_Fake.starts.size() == 2);
        assert(g_Fake.starts.back().mode == MotionController::Mode::FORWARD);
        completeCurrentMotion(executor, 530);

        uint32_t arrivalNodeID = 0;
        assert(executor.arrivalPending(arrivalNodeID));
        assert(arrivalNodeID == 4);
        executor.markArrivedSendResult(true, 531);
        assert(executor.state() == PhysicalFleetExecutor::State::NODE_WAIT);
    }

    void testClockwiseOneEdgeUsesDirectionSpecificTarget()
    {
        resetFake();
        MotionController motion;
        PhysicalFleetExecutor executor(motion);
        executor.begin(3, 0.0f);

        auto command = oneEdgeCommand(38, 3, 4);
        command.waypointCount = 3;
        command.waypoints[1] = waypoint(
            0.0f,
            0.0f,
            -kHalfPi,
            0.0f,
            0,
            RobotProtocol::TRAJECTORY_FLAG_ROTATE_IN_PLACE);
        command.waypoints[2] = waypoint(
            0.0f,
            -100.0f,
            -kHalfPi,
            0.0f,
            4,
            flags(RobotProtocol::TRAJECTORY_FLAG_NODE_BOUNDARY,
                  RobotProtocol::TRAJECTORY_FLAG_STOP,
                  RobotProtocol::TRAJECTORY_FLAG_FINAL));

        assert(executor.acceptTrajectory(command, true, 10)
               == PhysicalFleetExecutor::AcceptResult::STORED);
        executor.update(10, true);
        assert(g_Fake.starts.back().mode == MotionController::Mode::TURN_CW);
        assert(g_Fake.starts.back().targetCount
               == AppConfig::kTurn90CwCount);
        assert(g_Fake.starts.back().profile
               == MotionController::Profile::PHYSICAL_FLEET);
    }

    void testRotateDriveFinalRotateSequenceAndNoReportReplay()
    {
        resetFake();
        MotionController motion;
        PhysicalFleetExecutor executor(motion);
        executor.begin(1, 0.0f);
        enterNodeWait(executor, 10, 1, 2);

        const auto firstTurn = correction(
            10, 2, 1, RobotProtocol::NodeCorrectionAction::TURN_CW, 0.2f);
        assert(executor.acceptNodeCorrection(firstTurn, true, 100)
               == PhysicalFleetExecutor::CorrectionAcceptResult::STARTED);
        assert(g_Fake.starts.back().mode == MotionController::Mode::TURN_CW);
        assert(g_Fake.starts.back().targetCount == 9);
        assert(g_Fake.starts.back().profile
               == MotionController::Profile::CORRECTION);
        auto report = completeCorrection(executor, 110);
        assert(report.commandID == 1);

        const auto drive = correction(
            10,
            2,
            2,
            RobotProtocol::NodeCorrectionAction::DRIVE_FORWARD,
            80.0f);
        assert(executor.acceptNodeCorrection(drive, true, 120)
               == PhysicalFleetExecutor::CorrectionAcceptResult::STARTED);
        assert(g_Fake.starts.back().mode == MotionController::Mode::FORWARD);
        assert(g_Fake.starts.back().profile
               == MotionController::Profile::CORRECTION);
        report = completeCorrection(executor, 130);
        assert(report.commandID == 2);

        const auto finalTurn = correction(
            10, 2, 3, RobotProtocol::NodeCorrectionAction::TURN_CCW, 0.2f);
        assert(executor.acceptNodeCorrection(finalTurn, true, 140)
               == PhysicalFleetExecutor::CorrectionAcceptResult::STARTED);
        assert(g_Fake.starts.back().mode == MotionController::Mode::TURN_CCW);
        assert(g_Fake.starts.back().targetCount == 8);
        assert(g_Fake.starts.back().profile
               == MotionController::Profile::CORRECTION);
        report = completeCorrection(executor, 150);
        assert(report.commandID == 3);
        assert(std::fabs(executor.buildStatus().heading) < kFloatTolerance);

        const size_t completedStarts = g_Fake.starts.size();
        assert(executor.acceptNodeCorrection(finalTurn, true, 151)
               == PhysicalFleetExecutor::CorrectionAcceptResult::
                      DUPLICATE_IGNORED);
        assert(g_Fake.starts.size() == completedStarts);
        assert(!executor.correctionReportPending(report));

        const auto nextEdge = oneEdgeCommand(11, 2, 3);
        assert(executor.acceptTrajectory(nextEdge, true, 160)
               == PhysicalFleetExecutor::AcceptResult::STORED);
        assert(executor.acceptNodeCorrection(finalTurn, true, 161)
               == PhysicalFleetExecutor::CorrectionAcceptResult::
                      DUPLICATE_IGNORED);
        assert(executor.state() == PhysicalFleetExecutor::State::READY);
        assert(g_Fake.starts.size() == completedStarts);
    }

    void testCorrectionTurnCoastCompensationIsDirectionSpecificAndBounded()
    {
        resetFake();
        MotionController motion;
        PhysicalFleetExecutor executor(motion);
        executor.begin(1, 0.0f);
        enterNodeWait(executor, 12, 1, 2);

        auto turn = correction(
            12, 2, 1, RobotProtocol::NodeCorrectionAction::TURN_CW,
            radians(72.22f));
        assert(executor.acceptNodeCorrection(turn, true, 100)
               == PhysicalFleetExecutor::CorrectionAcceptResult::STARTED);
        // nominal round(72.22 / 90 * 163) = 131, less 14 measured coast.
        assert(g_Fake.starts.back().targetCount == 117);
        completeCorrection(executor, 110);

        turn = correction(
            12, 2, 2, RobotProtocol::NodeCorrectionAction::TURN_CCW,
            radians(42.35f));
        assert(executor.acceptNodeCorrection(turn, true, 120)
               == PhysicalFleetExecutor::CorrectionAcceptResult::STARTED);
        // nominal 75, less the direction-specific 12-count coast.
        assert(g_Fake.starts.back().targetCount == 63);
        completeCorrection(executor, 130);

        turn = correction(
            12, 2, 3, RobotProtocol::NodeCorrectionAction::TURN_CCW,
            radians(11.78f));
        assert(executor.acceptNodeCorrection(turn, true, 140)
               == PhysicalFleetExecutor::CorrectionAcceptResult::STARTED);
        // The smallest measured sample supports the full 12-count offset.
        assert(g_Fake.starts.back().targetCount == 9);
        completeCorrection(executor, 150);

        turn = correction(
            12, 2, 4, RobotProtocol::NodeCorrectionAction::TURN_CW,
            AppConfig::kCorrectionMinimumTurnRad);
        assert(executor.acceptNodeCorrection(turn, true, 160)
               == PhysicalFleetExecutor::CorrectionAcceptResult::STARTED);
        // A 5-degree nominal target is 9 counts. The 60% bound prevents the
        // fixed 14-count estimate from collapsing or reversing the command.
        assert(g_Fake.starts.back().targetCount == 4);
        assert(g_Fake.starts.back().mode == MotionController::Mode::TURN_CW);
        assert(g_Fake.starts.back().profile
               == MotionController::Profile::CORRECTION);
    }

    void testCompletedCorrectionSafelyGatesNextTrajectory()
    {
        resetFake();
        MotionController motion;
        PhysicalFleetExecutor executor(motion);
        executor.begin(1, radians(5.0f));
        enterNodeWait(executor, 13, 1, 2);

        const auto turn = correction(
            13, 2, 1, RobotProtocol::NodeCorrectionAction::TURN_CCW,
            radians(20.0f));
        assert(executor.acceptNodeCorrection(turn, true, 100)
               == PhysicalFleetExecutor::CorrectionAcceptResult::STARTED);
        const size_t correctionStarts = g_Fake.starts.size();
        assert(std::fabs(executor.buildStatus().heading - radians(15.0f))
               < kFloatTolerance);
        g_Fake.updateResult = MotionController::UpdateResult::SETTLING;
        executor.update(105, true);
        assert(executor.state()
               == PhysicalFleetExecutor::State::CORRECTION_SETTLING);
        assert(motion.outputsSafe());
        assert(g_Fake.starts.size() == correctionStarts);
        completeCurrentMotion(executor, 110);

        RobotProtocol::NodeCorrectionReportPayload report;
        assert(executor.correctionReportPending(report));
        assert(executor.state()
               == PhysicalFleetExecutor::State::CORRECTION_REPORT_PENDING);
        assert(motion.outputsSafe());
        assert(g_Fake.starts.size() == correctionStarts);
        assert(std::fabs(executor.buildStatus().heading - radians(25.0f))
               < kFloatTolerance);

        executor.markCorrectionReportSendResult(true);
        assert(executor.state() == PhysicalFleetExecutor::State::NODE_WAIT);
        assert(motion.outputsSafe());
        assert(std::fabs(executor.buildStatus().heading - radians(25.0f))
               < kFloatTolerance);

        const auto nextEdge = oneEdgeCommand(14, 2, 3);
        assert(executor.acceptTrajectory(nextEdge, true, 120)
               == PhysicalFleetExecutor::AcceptResult::STORED);
        assert(motion.outputsSafe());
        assert(g_Fake.starts.size() == correctionStarts);
        executor.update(120, true);
        assert(g_Fake.starts.size() == correctionStarts + 1);
        assert(g_Fake.starts.back().mode == MotionController::Mode::FORWARD);
        assert(g_Fake.starts.back().profile
               == MotionController::Profile::PHYSICAL_FLEET);
        assert(g_Fake.starts.back().targetCount
               == static_cast<int32_t>(std::lround(
                   100.0f * AppConfig::kForwardCountsPerMm)));
        assert(std::fabs(executor.buildStatus().heading - radians(25.0f))
               < kFloatTolerance);
    }

    void testColdIdleCorrectionRequiresCompletedEdgeBinding()
    {
        resetFake();
        MotionController motion;
        PhysicalFleetExecutor executor(motion);
        executor.begin(1, 0.0f);

        const auto turn = correction(
            1, 1, 1, RobotProtocol::NodeCorrectionAction::TURN_CCW,
            radians(20.0f));
        assert(executor.acceptNodeCorrection(turn, true, 100)
               == PhysicalFleetExecutor::CorrectionAcceptResult::REJECTED_BUSY);
        assert(g_Fake.starts.empty());
        assert(motion.outputsSafe());

        RobotProtocol::NodeCorrectionReportPayload report;
        assert(executor.correctionReportPending(report));
        assert(report.result == RobotProtocol::NodeCorrectionResult::REJECTED);
        executor.markCorrectionReportSendResult(true);
        assert(executor.state() == PhysicalFleetExecutor::State::IDLE);
    }

    void testTrajectoryDuringCorrectionRunningOrSettlingLatchesSafeFault()
    {
        for (const auto updateResult : {
                 MotionController::UpdateResult::RUNNING,
                 MotionController::UpdateResult::SETTLING })
        {
            resetFake();
            MotionController motion;
            PhysicalFleetExecutor executor(motion);
            executor.begin(1, 0.0f);
            enterNodeWait(executor, 14, 1, 2);

            const auto turn = correction(
                14, 2, 1, RobotProtocol::NodeCorrectionAction::TURN_CCW,
                radians(20.0f));
            assert(executor.acceptNodeCorrection(turn, true, 100)
                   == PhysicalFleetExecutor::CorrectionAcceptResult::STARTED);
            g_Fake.updateResult = updateResult;
            executor.update(110, true);
            assert(executor.state()
                   == (updateResult == MotionController::UpdateResult::RUNNING
                           ? PhysicalFleetExecutor::State::CORRECTION_RUNNING
                           : PhysicalFleetExecutor::State::CORRECTION_SETTLING));

            const size_t startCount = g_Fake.starts.size();
            const auto nextEdge = oneEdgeCommand(15, 2, 3);
            assert(executor.acceptTrajectory(nextEdge, true, 120)
                   == PhysicalFleetExecutor::AcceptResult::REJECTED_LATCHED);
            assert(executor.state()
                   == PhysicalFleetExecutor::State::FAULT_LATCHED);
            assert(g_Fake.starts.size() == startCount);
            assert(motion.outputsSafe());
        }
    }

    void testFailedCorrectionCannotStartNextTrajectory()
    {
        resetFake();
        MotionController motion;
        PhysicalFleetExecutor executor(motion);
        executor.begin(1, 0.0f);
        enterNodeWait(executor, 15, 1, 2);

        const auto turn = correction(
            15, 2, 1, RobotProtocol::NodeCorrectionAction::TURN_CW,
            radians(20.0f));
        assert(executor.acceptNodeCorrection(turn, true, 100)
               == PhysicalFleetExecutor::CorrectionAcceptResult::STARTED);
        const size_t startCount = g_Fake.starts.size();
        g_Fake.injectedFault = MotionController::Fault::STALL;
        g_Fake.updateResult = MotionController::UpdateResult::FAULTED;
        executor.update(110, true);
        assert(executor.state()
               == PhysicalFleetExecutor::State::FAULT_LATCHED);
        assert(motion.outputsSafe());

        RobotProtocol::NodeCorrectionReportPayload report;
        assert(executor.correctionReportPending(report));
        assert(report.result == RobotProtocol::NodeCorrectionResult::FAULT);

        executor.markCorrectionReportSendResult(true);
        assert(executor.state()
               == PhysicalFleetExecutor::State::FAULT_LATCHED);

        const auto nextEdge = oneEdgeCommand(16, 2, 3);
        assert(executor.acceptTrajectory(nextEdge, true, 120)
               == PhysicalFleetExecutor::AcceptResult::REJECTED_LATCHED);
        assert(g_Fake.starts.size() == startCount);
        assert(motion.outputsSafe());
    }

    void testCorrectionBoundsAndIdentity()
    {
        resetFake();
        MotionController motion;
        PhysicalFleetExecutor executor(motion);
        executor.begin(1, 0.0f);
        enterNodeWait(executor, 20, 1, 2);

        auto invalid = correction(
            20,
            2,
            1,
            RobotProtocol::NodeCorrectionAction::DRIVE_FORWARD,
            120.01f);
        assert(executor.acceptNodeCorrection(invalid, true, 100)
               == PhysicalFleetExecutor::CorrectionAcceptResult::
                      REJECTED_INVALID);
        RobotProtocol::NodeCorrectionReportPayload report;
        assert(executor.correctionReportPending(report));
        assert(report.result == RobotProtocol::NodeCorrectionResult::REJECTED);
        executor.markCorrectionReportSendResult(true);

        assert(executor.acceptNodeCorrection(invalid, true, 101)
               == PhysicalFleetExecutor::CorrectionAcceptResult::
                      DUPLICATE_IGNORED);
        assert(!executor.correctionReportPending(report));

        invalid = correction(
            999,
            2,
            2,
            RobotProtocol::NodeCorrectionAction::DRIVE_FORWARD,
            20.0f);
        assert(executor.acceptNodeCorrection(invalid, true, 110)
               == PhysicalFleetExecutor::CorrectionAcceptResult::
                      REJECTED_INVALID);
        assert(executor.correctionReportPending(report));
        assert(report.routeID == 999);
        assert(report.nodeID == 2);
        assert(report.commandID == 2);
        assert(report.result == RobotProtocol::NodeCorrectionResult::REJECTED);
        executor.markCorrectionReportSendResult(true);

        const auto maximum = correction(
            20,
            2,
            3,
            RobotProtocol::NodeCorrectionAction::DRIVE_FORWARD,
            120.0f);
        assert(executor.acceptNodeCorrection(maximum, true, 120)
               == PhysicalFleetExecutor::CorrectionAcceptResult::STARTED);
        assert(g_Fake.starts.back().targetCount
               == static_cast<int32_t>(std::lround(
                   120.0f * AppConfig::kForwardCountsPerMm)));
    }

    void testConflictingDuplicateCannotCreateSecondReport()
    {
        resetFake();
        MotionController motion;
        PhysicalFleetExecutor executor(motion);
        executor.begin(1, 0.0f);
        enterNodeWait(executor, 25, 1, 2);

        const auto accepted = correction(
            25,
            2,
            1,
            RobotProtocol::NodeCorrectionAction::DRIVE_FORWARD,
            40.0f);
        assert(executor.acceptNodeCorrection(accepted, true, 200)
               == PhysicalFleetExecutor::CorrectionAcceptResult::STARTED);
        auto report = completeCorrection(executor, 210);
        assert(report.commandID == 1);

        auto conflicting = accepted;
        conflicting.magnitude = 60.0f;
        assert(executor.acceptNodeCorrection(conflicting, true, 220)
               == PhysicalFleetExecutor::CorrectionAcceptResult::
                      REJECTED_LATCHED);
        assert(executor.state()
               == PhysicalFleetExecutor::State::FAULT_LATCHED);
        assert(!executor.correctionReportPending(report));
        assert(motion.outputsSafe());
    }

    void testEightPrimitiveRecoveryBudgetAndNinthRejection()
    {
        resetFake();
        MotionController motion;
        PhysicalFleetExecutor executor(motion);
        executor.begin(1, 0.0f);
        enterNodeWait(executor, 26, 1, 2);

        static_assert(
            AppConfig::kMaximumCorrectionPrimitivesPerNode == 8,
            "Server and ESP32 correction budgets must stay aligned");
        for (uint32_t commandID = 1;
             commandID <= AppConfig::kMaximumCorrectionPrimitivesPerNode;
             ++commandID)
        {
            const auto command = correction(
                26,
                2,
                commandID,
                RobotProtocol::NodeCorrectionAction::TURN_CW,
                0.1f);
            assert(executor.acceptNodeCorrection(
                       command, true, 300 + commandID * 10)
                   == PhysicalFleetExecutor::CorrectionAcceptResult::STARTED);
            const auto report = completeCorrection(
                executor, 301 + commandID * 10);
            assert(report.commandID == commandID);
        }

        const size_t acceptedStarts = g_Fake.starts.size();
        const auto ninth = correction(
            26,
            2,
            9,
            RobotProtocol::NodeCorrectionAction::TURN_CW,
            0.1f);
        assert(executor.acceptNodeCorrection(ninth, true, 400)
               == PhysicalFleetExecutor::CorrectionAcceptResult::
                      REJECTED_INVALID);
        assert(g_Fake.starts.size() == acceptedStarts);

        RobotProtocol::NodeCorrectionReportPayload report;
        assert(executor.correctionReportPending(report));
        assert(report.commandID == 9);
        assert(report.result == RobotProtocol::NodeCorrectionResult::REJECTED);
        assert(motion.outputsSafe());
    }

    void testCorrectionDeadlineStopsAndReportsFault()
    {
        resetFake();
        MotionController motion;
        PhysicalFleetExecutor executor(motion);
        executor.begin(1, 0.0f);
        enterNodeWait(executor, 30, 1, 2);

        const auto drive = correction(
            30,
            2,
            1,
            RobotProtocol::NodeCorrectionAction::DRIVE_FORWARD,
            80.0f);
        assert(executor.acceptNodeCorrection(drive, true, 1000)
               == PhysicalFleetExecutor::CorrectionAcceptResult::STARTED);
        executor.update(10999, true);
        assert(executor.state()
               == PhysicalFleetExecutor::State::CORRECTION_RUNNING);
        executor.update(11000, true);
        assert(executor.state()
               == PhysicalFleetExecutor::State::FAULT_LATCHED);
        assert(motion.outputsSafe());

        RobotProtocol::NodeCorrectionReportPayload report;
        assert(executor.correctionReportPending(report));
        assert(report.commandID == 1);
        assert(report.result == RobotProtocol::NodeCorrectionResult::FAULT);
        assert(report.detail != 0);
    }

    void testStaleLoopTimestampCannotInstantlyTimeoutCorrection()
    {
        resetFake();
        MotionController motion;
        PhysicalFleetExecutor executor(motion);
        executor.begin(1, 0.0f);
        enterNodeWait(executor, 31, 1, 2);

        const auto turn = correction(
            31, 2, 1, RobotProtocol::NodeCorrectionAction::TURN_CW, 0.2f);
        assert(executor.acceptNodeCorrection(turn, true, 1001)
               == PhysicalFleetExecutor::CorrectionAcceptResult::STARTED);

        // RobotClient callbacks can start a command after the loop captured
        // its timestamp. This one-millisecond skew must not look like a
        // wrapped multi-day elapsed interval.
        executor.update(1000, true);
        assert(executor.state()
               == PhysicalFleetExecutor::State::CORRECTION_RUNNING);
        RobotProtocol::NodeCorrectionReportPayload report;
        assert(!executor.correctionReportPending(report));

        executor.update(1001, true);
        assert(executor.state()
               == PhysicalFleetExecutor::State::CORRECTION_RUNNING);
        assert(!executor.correctionReportPending(report));
    }

    void testEmergencyStopAndCancelAreSafe()
    {
        resetFake();
        MotionController motion;
        PhysicalFleetExecutor executor(motion);
        executor.begin(1, 0.0f);
        enterNodeWait(executor, 40, 1, 2);

        const auto turn = correction(
            40, 2, 1, RobotProtocol::NodeCorrectionAction::TURN_CCW, 0.2f);
        assert(executor.acceptNodeCorrection(turn, true, 200)
               == PhysicalFleetExecutor::CorrectionAcceptResult::STARTED);
        executor.emergencyStop();
        assert(executor.state()
               == PhysicalFleetExecutor::State::ESTOP_LATCHED);
        assert(motion.outputsSafe());
        RobotProtocol::NodeCorrectionReportPayload report;
        assert(executor.correctionReportPending(report));
        assert(report.result == RobotProtocol::NodeCorrectionResult::FAULT);
        const size_t estopStarts = g_Fake.starts.size();
        const auto estopEdge = oneEdgeCommand(41, 2, 3);
        assert(executor.acceptTrajectory(estopEdge, true, 250)
               == PhysicalFleetExecutor::AcceptResult::REJECTED_LATCHED);
        assert(g_Fake.starts.size() == estopStarts);
        assert(motion.outputsSafe());

        resetFake();
        MotionController waitMotion;
        PhysicalFleetExecutor waitExecutor(waitMotion);
        waitExecutor.begin(1, 0.0f);
        enterNodeWait(waitExecutor, 41, 1, 2);
        waitExecutor.cancel();
        assert(waitExecutor.state() == PhysicalFleetExecutor::State::IDLE);
        assert(waitMotion.outputsSafe());

        const auto edge = oneEdgeCommand(42, 2, 3);
        assert(waitExecutor.acceptTrajectory(edge, true, 300)
               == PhysicalFleetExecutor::AcceptResult::STORED);
        waitExecutor.update(300, true);
        waitExecutor.cancel();
        assert(waitExecutor.state()
               == PhysicalFleetExecutor::State::FAULT_LATCHED);
        assert(waitMotion.outputsSafe());
        const size_t cancelStarts = g_Fake.starts.size();
        const auto afterCancel = oneEdgeCommand(43, 2, 3);
        assert(waitExecutor.acceptTrajectory(afterCancel, true, 310)
               == PhysicalFleetExecutor::AcceptResult::REJECTED_LATCHED);
        assert(g_Fake.starts.size() == cancelStarts);
    }

    void testCorrectionCancelAndNetworkLossCannotResumeOrReplay()
    {
        resetFake();
        MotionController cancelMotion;
        PhysicalFleetExecutor cancelExecutor(cancelMotion);
        cancelExecutor.begin(1, 0.0f);
        enterNodeWait(cancelExecutor, 45, 1, 2);

        const auto drive = correction(
            45,
            2,
            1,
            RobotProtocol::NodeCorrectionAction::DRIVE_FORWARD,
            40.0f);
        assert(cancelExecutor.acceptNodeCorrection(drive, true, 300)
               == PhysicalFleetExecutor::CorrectionAcceptResult::STARTED);
        cancelExecutor.cancel();
        assert(cancelExecutor.state()
               == PhysicalFleetExecutor::State::FAULT_LATCHED);
        assert(cancelMotion.outputsSafe());
        RobotProtocol::NodeCorrectionReportPayload report;
        assert(!cancelExecutor.correctionReportPending(report));

        resetFake();
        MotionController networkMotion;
        PhysicalFleetExecutor networkExecutor(networkMotion);
        networkExecutor.begin(1, 0.0f);
        enterNodeWait(networkExecutor, 46, 1, 2);
        const auto turn = correction(
            46, 2, 1, RobotProtocol::NodeCorrectionAction::TURN_CW, 0.2f);
        assert(networkExecutor.acceptNodeCorrection(turn, true, 400)
               == PhysicalFleetExecutor::CorrectionAcceptResult::STARTED);
        networkExecutor.onNetworkLost();
        assert(networkExecutor.state()
               == PhysicalFleetExecutor::State::FAULT_LATCHED);
        assert(networkMotion.outputsSafe());
        assert(!networkExecutor.correctionReportPending(report));

        // A later accepted TCP session must not revive the old primitive or
        // emit its report.
        const size_t startCount = g_Fake.starts.size();
        networkExecutor.update(500, true);
        assert(g_Fake.starts.size() == startCount);
        assert(!networkExecutor.correctionReportPending(report));
        const auto afterDisconnect = oneEdgeCommand(47, 2, 3);
        assert(networkExecutor.acceptTrajectory(afterDisconnect, true, 510)
               == PhysicalFleetExecutor::AcceptResult::REJECTED_LATCHED);
        assert(g_Fake.starts.size() == startCount);
        assert(networkMotion.outputsSafe());
    }

    void testFailedReportAttemptIsNotRetried()
    {
        resetFake();
        MotionController motion;
        PhysicalFleetExecutor executor(motion);
        executor.begin(1, 0.0f);
        enterNodeWait(executor, 50, 1, 2);

        const auto drive = correction(
            50,
            2,
            1,
            RobotProtocol::NodeCorrectionAction::DRIVE_FORWARD,
            40.0f);
        assert(executor.acceptNodeCorrection(drive, true, 400)
               == PhysicalFleetExecutor::CorrectionAcceptResult::STARTED);
        completeCurrentMotion(executor, 410);

        RobotProtocol::NodeCorrectionReportPayload report;
        assert(executor.correctionReportPending(report));
        executor.markCorrectionReportSendResult(false);
        assert(executor.state()
               == PhysicalFleetExecutor::State::FAULT_LATCHED);
        assert(!executor.correctionReportPending(report));
        const size_t startCount = g_Fake.starts.size();
        const auto afterFailedReport = oneEdgeCommand(51, 2, 3);
        assert(executor.acceptTrajectory(afterFailedReport, true, 420)
               == PhysicalFleetExecutor::AcceptResult::REJECTED_LATCHED);
        assert(g_Fake.starts.size() == startCount);
        assert(motion.outputsSafe());
    }

    void testCorrectionWheelMismatchFreezesTaggedDiagnostic()
    {
        resetFake();
        MotionController motion;
        PhysicalFleetExecutor executor(motion);
        executor.begin(1, 0.0f);
        enterNodeWait(executor, 60, 1, 2);

        const auto drive = correction(
            60,
            2,
            1,
            RobotProtocol::NodeCorrectionAction::DRIVE_FORWARD,
            120.0f);
        assert(executor.acceptNodeCorrection(drive, true, 500)
               == PhysicalFleetExecutor::CorrectionAcceptResult::STARTED);

        // Reproduce the useful shape of the field failure: the left wheel is
        // near its target while the right wheel trails by more than the
        // unchanged controller mismatch limit.
        g_Fake.leftProgress = 205;
        g_Fake.rightProgress = 117;
        g_Fake.leftTarget = 196;
        g_Fake.rightTarget = 196;
        g_Fake.injectedFault = MotionController::Fault::WHEEL_MISMATCH;
        g_Fake.updateResult = MotionController::UpdateResult::FAULTED;
        executor.update(510, true);

        assert(executor.state()
               == PhysicalFleetExecutor::State::FAULT_LATCHED);
        assert(executor.fault()
               == PhysicalFleetExecutor::Fault::MOTION_CONTROLLER);
        assert(executor.faultDetail() == 65539U);
        assert(motion.outputsSafe());

        PhysicalFleetExecutor::WheelMismatchDiagnostic diagnostic;
        assert(executor.wheelMismatchDiagnostic(diagnostic));
        assert(diagnostic.operation
               == PhysicalFleetExecutor::DiagnosticOperation::
                      CORRECTION_DRIVE);
        assert(diagnostic.motionMode == MotionController::Mode::FORWARD);
        assert(diagnostic.motionProfile
               == MotionController::Profile::CORRECTION);
        assert(diagnostic.leftProgress == 205);
        assert(diagnostic.rightProgress == 117);
        assert(diagnostic.leftTarget == 196);
        assert(diagnostic.rightTarget == 196);

        assert(RobotProtocol::encodeMotorFaultDiagnosticContext(
                   static_cast<uint8_t>(diagnostic.operation),
                   static_cast<uint8_t>(diagnostic.motionMode),
                   static_cast<uint8_t>(diagnostic.motionProfile))
               == 0xD0010312UL);
        assert(RobotProtocol::encodeMotorFaultDiagnosticValue(
                   RobotProtocol::MotorFaultDiagnosticTag::LEFT_PROGRESS,
                   diagnostic.leftProgress)
               == 0xD10000CDUL);
        assert(RobotProtocol::encodeMotorFaultDiagnosticValue(
                   RobotProtocol::MotorFaultDiagnosticTag::RIGHT_PROGRESS,
                   diagnostic.rightProgress)
               == 0xD2000075UL);
        assert(RobotProtocol::encodeMotorFaultDiagnosticValue(
                   RobotProtocol::MotorFaultDiagnosticTag::LEFT_TARGET,
                   diagnostic.leftTarget)
               == 0xD30000C4UL);
        assert(RobotProtocol::encodeMotorFaultDiagnosticValue(
                   RobotProtocol::MotorFaultDiagnosticTag::RIGHT_TARGET,
                   diagnostic.rightTarget)
               == 0xD40000C4UL);
    }
}

void MotionController::begin()
{
    m_Initialized = true;
    m_State = State::IDLE;
    m_Fault = Fault::NONE;
    m_Mode = Mode::NONE;
    m_LeftPwm = 0;
    m_RightPwm = 0;
}

MotionController::StartResult MotionController::startMotion(
    Mode mode,
    int32_t targetCount,
    uint32_t nowMs)
{
    return startMotion(mode, targetCount, nowMs, Profile::NORMAL);
}

MotionController::StartResult MotionController::startMotion(
    Mode mode,
    int32_t targetCount,
    uint32_t nowMs,
    Profile profile)
{
    g_Fake.starts.push_back({mode, targetCount, nowMs, profile});
    if (g_Fake.startResult != StartResult::STARTED)
        return g_Fake.startResult;
    m_Mode = mode;
    m_Profile = profile;
    m_TargetCount = targetCount;
    m_LeftTargetCount = targetCount;
    m_RightTargetCount = targetCount;
    m_MotionStartedMs = nowMs;
    m_State = State::RUNNING;
    m_LeftPwm = 1;
    m_RightPwm = 1;
    return StartResult::STARTED;
}

MotionController::UpdateResult MotionController::update(uint32_t nowMs)
{
    m_FinalElapsedMs = nowMs - m_MotionStartedMs;
    switch (g_Fake.updateResult)
    {
    case UpdateResult::RUNNING:
        m_State = State::RUNNING;
        m_LeftPwm = 1;
        m_RightPwm = 1;
        break;
    case UpdateResult::SETTLING:
        m_State = State::SETTLING;
        m_LeftPwm = 0;
        m_RightPwm = 0;
        break;
    case UpdateResult::COMPLETE:
        m_State = State::COMPLETE;
        m_LeftPwm = 0;
        m_RightPwm = 0;
        break;
    case UpdateResult::FAULTED:
        m_State = State::FAULTED;
        m_Fault = g_Fake.injectedFault;
        m_LeftPwm = 0;
        m_RightPwm = 0;
        break;
    case UpdateResult::IDLE:
    default:
        m_State = State::IDLE;
        m_LeftPwm = 0;
        m_RightPwm = 0;
        break;
    }
    return g_Fake.updateResult;
}

void MotionController::stopImmediately()
{
    ++g_Fake.stopCount;
    m_State = State::IDLE;
    m_Mode = Mode::NONE;
    m_LeftPwm = 0;
    m_RightPwm = 0;
}

void MotionController::emergencyStop(Fault cause)
{
    m_State = State::FAULTED;
    m_Fault = cause;
    m_LeftPwm = 0;
    m_RightPwm = 0;
}

MotionController::Snapshot MotionController::snapshot() const
{
    Snapshot result;
    result.targetCount = m_TargetCount;
    result.leftTargetCount = g_Fake.leftTarget != 0
        ? g_Fake.leftTarget : m_LeftTargetCount;
    result.rightTargetCount = g_Fake.rightTarget != 0
        ? g_Fake.rightTarget : m_RightTargetCount;
    result.leftProgress = g_Fake.leftProgress;
    result.rightProgress = g_Fake.rightProgress;
    result.leftPwm = m_LeftPwm;
    result.rightPwm = m_RightPwm;
    result.elapsedMs = m_FinalElapsedMs;
    result.running = m_State == State::RUNNING;
    result.settling = m_State == State::SETTLING;
    result.completed = m_State == State::COMPLETE;
    result.faultLatched = m_State == State::FAULTED;
    result.outputsSafe = outputsSafe();
    result.mode = m_Mode;
    result.profile = m_Profile;
    result.fault = m_Fault;
    result.progress = result.completed ? 1.0f : 0.5f;
    return result;
}

bool MotionController::outputsSafe() const
{
    return !g_Fake.forceUnsafe && m_LeftPwm == 0 && m_RightPwm == 0;
}

bool MotionController::faultLatched() const
{
    return m_State == State::FAULTED;
}

MotionController::Fault MotionController::fault() const
{
    return m_Fault;
}

int main()
{
    testOneEdgeArrivalCannotAutoResume();
    testMultiEdgeTrajectoryIsRejected();
    testCorrectedNodeResidualHeadingWithinTenDegreesIsStored();
    testStraightHeadingOverTenDegreesIsRejectedSafely();
    testRotateHeadingWithinTenDegreesIsStored();
    testRotateHeadingOverTenDegreesIsRejectedSafely();
    testOneEdgeMayContainRotationMarker();
    testClockwiseOneEdgeUsesDirectionSpecificTarget();
    testRotateDriveFinalRotateSequenceAndNoReportReplay();
    testCorrectionTurnCoastCompensationIsDirectionSpecificAndBounded();
    testCompletedCorrectionSafelyGatesNextTrajectory();
    testColdIdleCorrectionRequiresCompletedEdgeBinding();
    testTrajectoryDuringCorrectionRunningOrSettlingLatchesSafeFault();
    testFailedCorrectionCannotStartNextTrajectory();
    testCorrectionBoundsAndIdentity();
    testConflictingDuplicateCannotCreateSecondReport();
    testEightPrimitiveRecoveryBudgetAndNinthRejection();
    testCorrectionDeadlineStopsAndReportsFault();
    testStaleLoopTimestampCannotInstantlyTimeoutCorrection();
    testEmergencyStopAndCancelAreSafe();
    testCorrectionCancelAndNetworkLossCannotResumeOrReplay();
    testFailedReportAttemptIsNotRetried();
    testCorrectionWheelMismatchFreezesTaggedDiagnostic();
    return 0;
}
