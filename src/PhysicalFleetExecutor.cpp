#include "PhysicalFleetExecutor.hpp"

#include "Config.hpp"

#include <cmath>
#include <cstring>
#include <limits>

namespace
{
    constexpr float kPi = 3.14159265358979323846f;
    constexpr float kHalfPi = kPi * 0.5f;
    constexpr float kPositionToleranceMm = 1.0f;
    constexpr float kHeadingToleranceRad = 0.08f;
    constexpr float kScaleTolerance = 0.01f;
    constexpr float kMinimumDriveMm = 10.0f;
    constexpr float kMaximumDriveMm = 500.0f;

    constexpr uint32_t kDetailInvalidCommand = 1;
    constexpr uint32_t kDetailNetworkLost = 2;
    constexpr uint32_t kDetailMotionStart = 3;
    constexpr uint32_t kDetailUnsafeOutput = 4;
    constexpr uint32_t kDetailArrivedSend = 5;
    constexpr uint32_t kDetailConflictingCommand = 6;
    constexpr uint32_t kDetailCancelledDuringMotion = 7;
    constexpr uint32_t kDetailCorrectionInvalidID = 20;
    constexpr uint32_t kDetailCorrectionBinding = 21;
    constexpr uint32_t kDetailCorrectionState = 22;
    constexpr uint32_t kDetailCorrectionMagnitude = 23;
    constexpr uint32_t kDetailCorrectionLimit = 24;
    constexpr uint32_t kDetailCorrectionConflict = 25;
    constexpr uint32_t kDetailCorrectionReportSend = 26;
    constexpr uint32_t kDetailCorrectionTimeout = 27;
    constexpr uint32_t kDetailCorrectionEmergencyStop = 28;
    constexpr uint32_t kMotionFaultPrefix = 0x10000UL;

    bool finiteWaypoint(const RobotProtocol::TrajectoryWaypoint& waypoint)
    {
        return std::isfinite(waypoint.forwardMm)
            && std::isfinite(waypoint.leftMm)
            && std::isfinite(waypoint.headingRad)
            && std::isfinite(waypoint.targetSpeedMmPerSecond);
    }

    bool near(float lhs, float rhs, float tolerance)
    {
        return std::fabs(lhs - rhs) <= tolerance;
    }
}

PhysicalFleetExecutor::PhysicalFleetExecutor(MotionController& motion)
    : m_Motion(motion)
{
}

void PhysicalFleetExecutor::begin(uint32_t startNodeID,
                                  float startWorldHeadingRad)
{
    m_Motion.begin();
    m_Motion.stopImmediately();
    clearCommand();
    m_CurrentNodeID = startNodeID;
    m_WorldHeadingRad = normalizeAngle(startWorldHeadingRad);
    m_State = State::IDLE;
    m_Fault = Fault::NONE;
    m_FaultDetail = 0;
    resetCorrectionSession(0);
}

PhysicalFleetExecutor::AcceptResult PhysicalFleetExecutor::acceptTrajectory(
    const RobotProtocol::TrajectoryCommandPayload& command,
    bool sessionReady,
    uint32_t nowMs)
{
    if (!sessionReady)
        return AcceptResult::REJECTED_SESSION_NOT_READY;
    if (terminalLatch())
        return AcceptResult::REJECTED_LATCHED;
    if (!validateCommand(command) || command.startNodeID != m_CurrentNodeID)
    {
        latchFault(Fault::INVALID_COMMAND, kDetailInvalidCommand);
        return AcceptResult::REJECTED_INVALID;
    }

    if (m_HasCommand)
    {
        if (commandEquals(m_Command, command))
            return AcceptResult::DUPLICATE_IGNORED;
        latchFault(Fault::CONFLICTING_COMMAND, kDetailConflictingCommand);
        return AcceptResult::REJECTED_LATCHED;
    }
    if (m_State != State::IDLE && m_State != State::NODE_WAIT)
    {
        latchFault(Fault::CONFLICTING_COMMAND, kDetailConflictingCommand);
        return AcceptResult::REJECTED_LATCHED;
    }

    m_Motion.stopImmediately();
    if (!m_Motion.outputsSafe())
    {
        latchFault(Fault::OUTPUTS_NOT_SAFE, kDetailUnsafeOutput);
        return AcceptResult::REJECTED_LATCHED;
    }

    m_Command = command;
    m_HasCommand = true;
    m_Cursor = 1; // The first boundary is the already occupied start node.
    m_TargetNodeID = 0;
    m_ArrivalNodeID = 0;
    m_CurrentForwardMm = command.waypoints[0].forwardMm;
    m_CurrentLeftMm = command.waypoints[0].leftMm;
    m_CurrentLocalHeadingRad = command.waypoints[0].headingRad;
    m_CommandOriginWorldHeadingRad = m_WorldHeadingRad;
    m_Primitive = PrimitiveKind::NONE;
    m_MotionMode = MotionController::Mode::NONE;
    m_RemainingTurnQuarters = 0;
    m_PauseStartedMs = nowMs;
    m_State = State::READY;
    return AcceptResult::STORED;
}

PhysicalFleetExecutor::CorrectionAcceptResult
PhysicalFleetExecutor::acceptNodeCorrection(
    const RobotProtocol::NodeCorrectionCommandPayload& command,
    bool sessionReady,
    uint32_t nowMs)
{
    if (!sessionReady)
        return CorrectionAcceptResult::REJECTED_SESSION_NOT_READY;
    if (terminalLatch())
        return CorrectionAcceptResult::REJECTED_LATCHED;

    if (m_HasLastCorrectionCommand
        && command.commandID == m_LastCorrectionCommand.commandID)
    {
        if (!correctionCommandEquals(command, m_LastCorrectionCommand))
        {
            // Reusing one command ID with different bytes destroys report
            // correlation. If the accepted command is still moving, produce
            // its one terminal FAULT report; otherwise never replay/report an
            // already-finished ID and rely on the latched ERROR path.
            if (m_State == State::CORRECTION_RUNNING
                || m_State == State::CORRECTION_SETTLING)
            {
                stageActiveCorrectionFault(kDetailCorrectionConflict);
            }
            latchFault(Fault::INVALID_CORRECTION,
                       kDetailCorrectionConflict);
            return CorrectionAcceptResult::REJECTED_LATCHED;
        }

        // A command has exactly one report. Exact TCP/application retries are
        // harmlessly ignored even after the next edge has started; replaying
        // a cached report would be a stale correlation on the Server.
        return CorrectionAcceptResult::DUPLICATE_IGNORED;
    }

    if (m_State != State::NODE_WAIT || m_HasCommand
        || m_HasPendingCorrectionReport)
    {
        if (m_State == State::CORRECTION_RUNNING
            || m_State == State::CORRECTION_SETTLING)
        {
            stageActiveCorrectionFault(kDetailCorrectionState);
            latchFault(Fault::INVALID_CORRECTION, kDetailCorrectionState);
            return CorrectionAcceptResult::REJECTED_LATCHED;
        }
        if (m_HasCommand || m_State == State::READY
            || m_State == State::SAFE_PAUSE
            || m_State == State::RUNNING
            || m_State == State::SETTLING
            || m_State == State::ARRIVAL_PENDING)
        {
            m_Motion.stopImmediately();
            stageCorrectionReportFor(
                command,
                RobotProtocol::NodeCorrectionResult::FAULT,
                kDetailCorrectionState,
                State::FAULT_LATCHED);
            latchFault(Fault::INVALID_CORRECTION,
                       kDetailCorrectionState);
            return CorrectionAcceptResult::REJECTED_LATCHED;
        }
        if (m_State != State::CORRECTION_REPORT_PENDING)
        {
            const State returnState = m_State;
            stageCorrectionReportFor(
                command,
                RobotProtocol::NodeCorrectionResult::REJECTED,
                kDetailCorrectionState,
                returnState);
        }
        return CorrectionAcceptResult::REJECTED_BUSY;
    }

    if (command.routeID == 0 || command.nodeID == 0
        || command.commandID == 0)
    {
        stageCorrectionReportFor(
            command,
            RobotProtocol::NodeCorrectionResult::REJECTED,
            kDetailCorrectionInvalidID,
            State::NODE_WAIT);
        return CorrectionAcceptResult::REJECTED_INVALID;
    }
    if (command.routeID != m_LastCompletedRouteID
        || command.nodeID != m_CurrentNodeID)
    {
        stageCorrectionReportFor(
            command,
            RobotProtocol::NodeCorrectionResult::REJECTED,
            kDetailCorrectionBinding,
            State::NODE_WAIT);
        return CorrectionAcceptResult::REJECTED_INVALID;
    }
    if (m_HasLastCorrectionCommand
        && command.commandID <= m_LastCorrectionCommand.commandID)
    {
        // An older ID is a stale replay. Reporting it would create a late
        // correlation after the Server has already advanced.
        latchFault(Fault::INVALID_CORRECTION,
                   kDetailCorrectionInvalidID);
        return CorrectionAcceptResult::REJECTED_LATCHED;
    }

    const bool drive = command.action
        == RobotProtocol::NodeCorrectionAction::DRIVE_FORWARD;
    const bool turn = command.action
            == RobotProtocol::NodeCorrectionAction::TURN_CW
        || command.action
            == RobotProtocol::NodeCorrectionAction::TURN_CCW;
    const bool validMagnitude = std::isfinite(command.magnitude)
        && ((drive
             && command.magnitude >= AppConfig::kCorrectionMinimumDriveMm
             && command.magnitude <= AppConfig::kCorrectionMaximumDriveMm)
            || (turn
                && command.magnitude >= AppConfig::kCorrectionMinimumTurnRad
                && command.magnitude <= AppConfig::kCorrectionMaximumTurnRad));
    if ((!drive && !turn) || !validMagnitude)
    {
        stageCorrectionReportFor(
            command,
            RobotProtocol::NodeCorrectionResult::REJECTED,
            kDetailCorrectionMagnitude,
            State::NODE_WAIT);
        return CorrectionAcceptResult::REJECTED_INVALID;
    }
    if (m_CorrectionPrimitiveCount
        >= AppConfig::kMaximumCorrectionPrimitivesPerNode)
    {
        stageCorrectionReportFor(
            command,
            RobotProtocol::NodeCorrectionResult::REJECTED,
            kDetailCorrectionLimit,
            State::NODE_WAIT);
        return CorrectionAcceptResult::REJECTED_INVALID;
    }

    m_Motion.stopImmediately();
    if (!m_Motion.outputsSafe())
    {
        stageCorrectionReportFor(
            command,
            RobotProtocol::NodeCorrectionResult::FAULT,
            kDetailUnsafeOutput,
            State::FAULT_LATCHED);
        latchFault(Fault::OUTPUTS_NOT_SAFE, kDetailUnsafeOutput);
        return CorrectionAcceptResult::REJECTED_LATCHED;
    }

    m_LastCorrectionCommand = command;
    m_HasLastCorrectionCommand = true;
    m_ActiveCorrectionHeadingDeltaRad = 0.0f;

    MotionController::Mode mode = MotionController::Mode::FORWARD;
    int32_t targetCounts = 0;
    if (drive)
    {
        m_Primitive = PrimitiveKind::CORRECTION_DRIVE;
        targetCounts = static_cast<int32_t>(std::lround(
            command.magnitude * AppConfig::kForwardCountsPerMm));
    }
    else
    {
        const bool clockwise = command.action
            == RobotProtocol::NodeCorrectionAction::TURN_CW;
        m_Primitive = PrimitiveKind::CORRECTION_TURN;
        mode = clockwise ? MotionController::Mode::TURN_CW
                         : MotionController::Mode::TURN_CCW;
        m_ActiveCorrectionHeadingDeltaRad = clockwise
            ? -command.magnitude : command.magnitude;
        targetCounts = static_cast<int32_t>(std::lround(
            command.magnitude * AppConfig::kTurnCountsPerRadian));
    }
    m_MotionMode = mode;

    const MotionController::StartResult startResult =
        m_Motion.startMotion(mode, targetCounts, nowMs);
    if (startResult == MotionController::StartResult::OUTPUT_DISABLED)
    {
        stageCorrectionReport(
            RobotProtocol::NodeCorrectionResult::FAULT,
            kDetailMotionStart
                | (static_cast<uint32_t>(startResult) << 8),
            State::OUTPUT_LOCKED);
        m_State = State::OUTPUT_LOCKED;
        return CorrectionAcceptResult::REJECTED_LATCHED;
    }
    if (startResult != MotionController::StartResult::STARTED)
    {
        const uint32_t detail = kDetailMotionStart
            | (static_cast<uint32_t>(startResult) << 8);
        stageCorrectionReport(
            RobotProtocol::NodeCorrectionResult::FAULT,
            detail,
            State::FAULT_LATCHED);
        latchFault(Fault::MOTION_START_FAILED, detail);
        return CorrectionAcceptResult::REJECTED_LATCHED;
    }

    ++m_CorrectionPrimitiveCount;
    m_CorrectionStartedMs = nowMs;
    m_State = State::CORRECTION_RUNNING;
    return CorrectionAcceptResult::STARTED;
}

void PhysicalFleetExecutor::update(uint32_t nowMs, bool sessionReady)
{
    if (!sessionReady)
    {
        if (m_HasCommand
            || m_State == State::CORRECTION_RUNNING
            || m_State == State::CORRECTION_SETTLING
            || m_State == State::CORRECTION_REPORT_PENDING
            || m_State == State::NODE_WAIT)
            onNetworkLost();
        return;
    }

    if (m_State == State::CORRECTION_REPORT_PENDING)
    {
        m_Motion.stopImmediately();
        if (!m_Motion.outputsSafe())
            latchFault(Fault::OUTPUTS_NOT_SAFE, kDetailUnsafeOutput);
        return;
    }

    if (m_State == State::NODE_WAIT)
    {
        m_Motion.stopImmediately();
        if (!m_Motion.outputsSafe())
            latchFault(Fault::OUTPUTS_NOT_SAFE, kDetailUnsafeOutput);
        return;
    }

    if (m_State == State::READY)
    {
        startCurrentWaypoint(nowMs);
        return;
    }
    if (m_State == State::SAFE_PAUSE)
    {
        m_Motion.stopImmediately();
        if (!m_Motion.outputsSafe())
        {
            latchFault(Fault::OUTPUTS_NOT_SAFE, kDetailUnsafeOutput);
            return;
        }
        if (nowMs - m_PauseStartedMs < AppConfig::kPrimitiveSafePauseMs)
            return;

        if (m_RemainingTurnQuarters > 0)
            startTurnQuarter(nowMs);
        else
        {
            m_State = State::READY;
            startCurrentWaypoint(nowMs);
        }
        return;
    }
    const bool correctionMotion =
        m_State == State::CORRECTION_RUNNING
        || m_State == State::CORRECTION_SETTLING;
    if (m_State != State::RUNNING && m_State != State::SETTLING
        && !correctionMotion)
        return;

    if (correctionMotion
        && nowMs - m_CorrectionStartedMs
               >= AppConfig::kCorrectionPrimitiveTimeoutMs)
    {
        stageActiveCorrectionFault(kDetailCorrectionTimeout);
        latchFault(Fault::MOTION_CONTROLLER, kDetailCorrectionTimeout);
        return;
    }

    const MotionController::UpdateResult result = m_Motion.update(nowMs);
    if (m_Motion.faultLatched()
        || result == MotionController::UpdateResult::FAULTED)
    {
        const uint32_t detail = kMotionFaultPrefix
            | (static_cast<uint32_t>(m_Motion.fault()) & 0xFFFFUL);
        if (correctionMotion)
            stageActiveCorrectionFault(detail);
        latchFault(Fault::MOTION_CONTROLLER, detail);
        return;
    }
    if (result == MotionController::UpdateResult::RUNNING)
    {
        m_State = correctionMotion ? State::CORRECTION_RUNNING
                                   : State::RUNNING;
        return;
    }
    if (result == MotionController::UpdateResult::SETTLING)
    {
        if (!m_Motion.outputsSafe())
        {
            if (correctionMotion)
                stageActiveCorrectionFault(kDetailUnsafeOutput);
            latchFault(Fault::OUTPUTS_NOT_SAFE, kDetailUnsafeOutput);
        }
        else
            m_State = correctionMotion ? State::CORRECTION_SETTLING
                                       : State::SETTLING;
        return;
    }
    if (result != MotionController::UpdateResult::COMPLETE
        || !m_Motion.outputsSafe())
    {
        if (correctionMotion)
            stageActiveCorrectionFault(kDetailUnsafeOutput);
        latchFault(Fault::OUTPUTS_NOT_SAFE, kDetailUnsafeOutput);
        return;
    }

    if (correctionMotion)
    {
        if (m_Primitive == PrimitiveKind::CORRECTION_TURN)
        {
            m_WorldHeadingRad = normalizeAngle(
                m_WorldHeadingRad + m_ActiveCorrectionHeadingDeltaRad);
        }
        else if (m_Primitive != PrimitiveKind::CORRECTION_DRIVE)
        {
            stageActiveCorrectionFault(kDetailCorrectionState);
            latchFault(Fault::INVALID_CORRECTION,
                       kDetailCorrectionState);
            return;
        }

        m_Primitive = PrimitiveKind::NONE;
        m_MotionMode = MotionController::Mode::NONE;
        m_ActiveCorrectionHeadingDeltaRad = 0.0f;
        m_CorrectionStartedMs = 0;
        stageCorrectionReport(
            RobotProtocol::NodeCorrectionResult::COMPLETED,
            0,
            State::NODE_WAIT);
        return;
    }

    handleMotionComplete(nowMs);
}

void PhysicalFleetExecutor::startCurrentWaypoint(uint32_t nowMs)
{
    if (!m_HasCommand || m_Cursor >= m_Command.waypointCount)
    {
        latchFault(Fault::INVALID_COMMAND, kDetailInvalidCommand);
        return;
    }

    const RobotProtocol::TrajectoryWaypoint& waypoint =
        m_Command.waypoints[m_Cursor];
    if (hasFlag(waypoint, RobotProtocol::TRAJECTORY_FLAG_ROTATE_IN_PLACE))
    {
        const float delta = normalizeAngle(
            waypoint.headingRad - m_CurrentLocalHeadingRad);
        const int quarters = static_cast<int>(std::lround(delta / kHalfPi));
        m_RemainingTurnQuarters = static_cast<uint8_t>(
            quarters < 0 ? -quarters : quarters);
        m_TurnQuarterDirectionRad = quarters < 0 ? -kHalfPi : kHalfPi;
        m_TurnStartLocalHeadingRad = m_CurrentLocalHeadingRad;
        startTurnQuarter(nowMs);
        return;
    }

    const float deltaForward = waypoint.forwardMm - m_CurrentForwardMm;
    const float deltaLeft = waypoint.leftMm - m_CurrentLeftMm;
    const float distanceMm = std::hypot(deltaForward, deltaLeft);
    const int32_t targetCounts = static_cast<int32_t>(
        std::lround(distanceMm * AppConfig::kForwardCountsPerMm));

    m_Primitive = PrimitiveKind::DRIVE;
    m_MotionMode = MotionController::Mode::FORWARD;
    m_TargetNodeID = waypoint.nodeID;
    const MotionController::StartResult result =
        m_Motion.startMotion(m_MotionMode, targetCounts, nowMs);
    if (result == MotionController::StartResult::OUTPUT_DISABLED)
    {
        m_State = State::OUTPUT_LOCKED;
        return;
    }
    if (result != MotionController::StartResult::STARTED)
    {
        latchFault(Fault::MOTION_START_FAILED,
                   kDetailMotionStart
                       | (static_cast<uint32_t>(result) << 8));
        return;
    }
    m_State = State::RUNNING;
}

void PhysicalFleetExecutor::startTurnQuarter(uint32_t nowMs)
{
    if (m_RemainingTurnQuarters == 0)
    {
        ++m_Cursor;
        m_State = State::READY;
        return;
    }

    m_Primitive = PrimitiveKind::TURN;
    m_MotionMode = m_TurnQuarterDirectionRad < 0.0f
        ? MotionController::Mode::TURN_CW
        : MotionController::Mode::TURN_CCW;
    const int32_t targetCounts = static_cast<int32_t>(
        std::lround(kHalfPi * AppConfig::kTurnCountsPerRadian));
    const MotionController::StartResult result =
        m_Motion.startMotion(m_MotionMode, targetCounts, nowMs);
    if (result == MotionController::StartResult::OUTPUT_DISABLED)
    {
        m_State = State::OUTPUT_LOCKED;
        return;
    }
    if (result != MotionController::StartResult::STARTED)
    {
        latchFault(Fault::MOTION_START_FAILED,
                   kDetailMotionStart
                       | (static_cast<uint32_t>(result) << 8));
        return;
    }
    m_State = State::RUNNING;
}

void PhysicalFleetExecutor::handleMotionComplete(uint32_t nowMs)
{
    if (m_Primitive == PrimitiveKind::TURN)
    {
        m_CurrentLocalHeadingRad = normalizeAngle(
            m_CurrentLocalHeadingRad + m_TurnQuarterDirectionRad);
        m_WorldHeadingRad = normalizeAngle(
            m_CommandOriginWorldHeadingRad + m_CurrentLocalHeadingRad);
        if (m_RemainingTurnQuarters > 0)
            --m_RemainingTurnQuarters;
        if (m_RemainingTurnQuarters == 0)
        {
            const RobotProtocol::TrajectoryWaypoint& rotate =
                m_Command.waypoints[m_Cursor];
            m_CurrentLocalHeadingRad = rotate.headingRad;
            m_WorldHeadingRad = normalizeAngle(
                m_CommandOriginWorldHeadingRad + m_CurrentLocalHeadingRad);
            ++m_Cursor;
        }
        m_Primitive = PrimitiveKind::NONE;
        m_PauseStartedMs = nowMs;
        m_State = State::SAFE_PAUSE;
        return;
    }

    if (m_Primitive != PrimitiveKind::DRIVE)
    {
        latchFault(Fault::INVALID_COMMAND, kDetailInvalidCommand);
        return;
    }

    const RobotProtocol::TrajectoryWaypoint& boundary =
        m_Command.waypoints[m_Cursor];
    m_CurrentForwardMm = boundary.forwardMm;
    m_CurrentLeftMm = boundary.leftMm;
    m_CurrentLocalHeadingRad = boundary.headingRad;
    m_WorldHeadingRad = normalizeAngle(
        m_CommandOriginWorldHeadingRad + m_CurrentLocalHeadingRad);
    m_CurrentNodeID = boundary.nodeID;
    m_TargetNodeID = 0;
    m_ArrivalNodeID = boundary.nodeID;
    m_Primitive = PrimitiveKind::NONE;
    m_State = State::ARRIVAL_PENDING;
}

bool PhysicalFleetExecutor::arrivalPending(uint32_t& outNodeID) const
{
    if (m_State != State::ARRIVAL_PENDING
        || m_ArrivalNodeID == 0
        || !m_Motion.outputsSafe())
    {
        return false;
    }
    outNodeID = m_ArrivalNodeID;
    return true;
}

void PhysicalFleetExecutor::markArrivedSendResult(bool sent, uint32_t)
{
    if (m_State != State::ARRIVAL_PENDING)
        return;
    if (!sent)
    {
        latchFault(Fault::ARRIVED_SEND_FAILED, kDetailArrivedSend);
        return;
    }
    if (!m_Motion.outputsSafe())
    {
        latchFault(Fault::OUTPUTS_NOT_SAFE, kDetailUnsafeOutput);
        return;
    }

    m_ArrivalNodeID = 0;
    const bool final = hasFlag(
        m_Command.waypoints[m_Cursor],
        RobotProtocol::TRAJECTORY_FLAG_FINAL);
    if (!final)
    {
        // Physical-fleet commands are deliberately one-edge only. Never
        // continue an in-memory route after reporting a node arrival.
        latchFault(Fault::INVALID_COMMAND, kDetailInvalidCommand);
        return;
    }

    const uint32_t completedRouteID = m_Command.routeID;
    clearCommand();
    resetCorrectionSession(completedRouteID);
    m_State = State::NODE_WAIT;
}

bool PhysicalFleetExecutor::correctionReportPending(
    RobotProtocol::NodeCorrectionReportPayload& outReport) const
{
    if (!m_HasPendingCorrectionReport)
        return false;
    outReport = m_PendingCorrectionReport;
    return true;
}

void PhysicalFleetExecutor::markCorrectionReportSendResult(bool sent)
{
    if (!m_HasPendingCorrectionReport)
        return;

    // One completion/fault produces one wire attempt. A failed or partial
    // write closes the TCP stream; replay after reconnect would be stale.
    m_HasPendingCorrectionReport = false;
    m_PendingCorrectionReport = {};
    if (!sent)
    {
        latchFault(Fault::CORRECTION_REPORT_SEND_FAILED,
                   kDetailCorrectionReportSend);
        return;
    }

    if (m_State == State::CORRECTION_REPORT_PENDING)
        m_State = m_AfterCorrectionReportState;
}

void PhysicalFleetExecutor::cancel()
{
    const bool wasMoving = m_State == State::RUNNING
        || m_State == State::SETTLING
        || m_State == State::CORRECTION_RUNNING
        || m_State == State::CORRECTION_SETTLING;
    m_Motion.stopImmediately();
    if (!m_Motion.outputsSafe())
    {
        latchFault(Fault::OUTPUTS_NOT_SAFE, kDetailUnsafeOutput);
        return;
    }
    if (terminalLatch())
        return;
    if (wasMoving)
    {
        latchFault(Fault::CANCELLED_DURING_MOTION,
                   kDetailCancelledDuringMotion);
        return;
    }
    clearCommand();
    resetCorrectionSession(0);
    m_State = State::IDLE;
}

void PhysicalFleetExecutor::onNetworkLost()
{
    const bool correctionActive =
        m_State == State::CORRECTION_RUNNING
        || m_State == State::CORRECTION_SETTLING;
    const bool operationActive = m_HasCommand || correctionActive
        || m_State == State::CORRECTION_REPORT_PENDING;
    m_Motion.stopImmediately();
    if (!m_Motion.outputsSafe())
    {
        latchFault(Fault::OUTPUTS_NOT_SAFE, kDetailUnsafeOutput);
        return;
    }
    if (correctionActive || m_State == State::CORRECTION_REPORT_PENDING)
    {
        // The command belonged to the dead TCP session. Never carry its
        // terminal report into a later HELLO session, where it could match a
        // different Server correction state.
        m_HasPendingCorrectionReport = false;
        m_PendingCorrectionReport = {};
    }
    if (operationActive)
        latchFault(Fault::NETWORK_LOST, kDetailNetworkLost);
}

void PhysicalFleetExecutor::emergencyStop()
{
    const bool correctionActive =
        m_State == State::CORRECTION_RUNNING
        || m_State == State::CORRECTION_SETTLING;
    if (correctionActive)
        stageActiveCorrectionFault(kDetailCorrectionEmergencyStop);
    m_Motion.emergencyStop(MotionController::Fault::EXTERNAL_STOP);
    m_State = State::ESTOP_LATCHED;
    m_Fault = Fault::NONE;
    m_FaultDetail = 0;
}

RobotProtocol::StatusPayload PhysicalFleetExecutor::buildStatus() const
{
    const MotionController::Snapshot motion = m_Motion.snapshot();
    RobotProtocol::StatusPayload status;
    status.currentNodeID = m_CurrentNodeID;
    status.currentLinkID = 0;
    status.progress = 0.0f;
    status.x = 0.0f;
    status.z = 0.0f;
    status.heading = m_WorldHeadingRad;
    status.velocity = 0.0f;
    status.battery = 100.0f;
    status.state = RobotProtocol::RobotState::IDLE;

    if ((m_State == State::RUNNING || m_State == State::SETTLING)
        && m_Primitive == PrimitiveKind::DRIVE)
    {
        status.currentLinkID = m_TargetNodeID;
        status.progress = motion.progress;
        status.state = RobotProtocol::RobotState::MOVING;
    }
    else if ((m_State == State::RUNNING || m_State == State::SETTLING)
             && m_Primitive == PrimitiveKind::TURN)
    {
        status.progress = motion.progress;
        status.heading = normalizeAngle(
            m_CommandOriginWorldHeadingRad
            + m_CurrentLocalHeadingRad
            + m_TurnQuarterDirectionRad * motion.progress);
        status.state = RobotProtocol::RobotState::MOVING;
    }
    else if ((m_State == State::CORRECTION_RUNNING
              || m_State == State::CORRECTION_SETTLING)
             && (m_Primitive == PrimitiveKind::CORRECTION_DRIVE
                 || m_Primitive == PrimitiveKind::CORRECTION_TURN))
    {
        status.progress = motion.progress;
        if (m_Primitive == PrimitiveKind::CORRECTION_TURN)
        {
            status.heading = normalizeAngle(
                m_WorldHeadingRad
                + m_ActiveCorrectionHeadingDeltaRad * motion.progress);
        }
        status.state = RobotProtocol::RobotState::MOVING;
    }
    else if (m_State == State::FAULT_LATCHED)
        status.state = RobotProtocol::RobotState::FAULT;
    else if (m_State == State::ESTOP_LATCHED)
        status.state = RobotProtocol::RobotState::EMERGENCY_STOPPED;

    return status;
}

bool PhysicalFleetExecutor::validateCommand(
    const RobotProtocol::TrajectoryCommandPayload& command)
{
    if (command.routeID == 0
        || command.formatVersion != RobotProtocol::kTrajectoryFormatVersion
        || command.waypointCount < 2
        || command.waypointCount > RobotProtocol::kMaxTrajectoryWaypoints
        || command.startNodeID == 0
        || command.finalNodeID == 0
        || !std::isfinite(command.millimetersPerMapUnit)
        || !near(command.millimetersPerMapUnit,
                 AppConfig::kPhysicalFleetScaleMmPerMapUnit,
                 kScaleTolerance))
    {
        return false;
    }

    const RobotProtocol::TrajectoryWaypoint& first = command.waypoints[0];
    if (!finiteWaypoint(first)
        || std::hypot(first.forwardMm, first.leftMm) > kPositionToleranceMm
        || std::fabs(normalizeAngle(first.headingRad)) > kHeadingToleranceRad
        || first.nodeID != command.startNodeID
        || !hasFlag(first, RobotProtocol::TRAJECTORY_FLAG_NODE_BOUNDARY)
        || hasFlag(first, RobotProtocol::TRAJECTORY_FLAG_FINAL))
    {
        return false;
    }

    float currentForward = first.forwardMm;
    float currentLeft = first.leftMm;
    float currentHeading = first.headingRad;
    uint32_t lastBoundaryNode = first.nodeID;
    for (uint16_t i = 1; i < command.waypointCount; ++i)
    {
        const RobotProtocol::TrajectoryWaypoint& waypoint = command.waypoints[i];
        if (!finiteWaypoint(waypoint))
            return false;

        const bool boundary = hasFlag(
            waypoint, RobotProtocol::TRAJECTORY_FLAG_NODE_BOUNDARY);
        const bool stop = hasFlag(waypoint, RobotProtocol::TRAJECTORY_FLAG_STOP);
        const bool rotate = hasFlag(
            waypoint, RobotProtocol::TRAJECTORY_FLAG_ROTATE_IN_PLACE);
        const bool final = hasFlag(waypoint, RobotProtocol::TRAJECTORY_FLAG_FINAL);
        if (final != (i + 1 == command.waypointCount))
            return false;

        if (rotate)
        {
            if (boundary || stop || final || waypoint.nodeID != 0
                || std::fabs(waypoint.targetSpeedMmPerSecond) > 0.001f
                || !near(waypoint.forwardMm, currentForward,
                         kPositionToleranceMm)
                || !near(waypoint.leftMm, currentLeft,
                         kPositionToleranceMm))
            {
                return false;
            }
            const float delta = normalizeAngle(waypoint.headingRad - currentHeading);
            const int quarters = static_cast<int>(std::lround(delta / kHalfPi));
            if (quarters == 0 || std::abs(quarters) > 2
                || std::fabs(delta - quarters * kHalfPi) > kHeadingToleranceRad)
            {
                return false;
            }
            currentHeading = waypoint.headingRad;
            continue;
        }

        // Each accepted command is exactly one edge. Rotation markers may
        // precede it, but the only new node boundary must be the final node.
        // This makes ARRIVED -> NODE_WAIT structural, even if a Server sends
        // an obsolete multi-edge trajectory.
        if (!boundary || !stop || !final || waypoint.nodeID == 0
            || waypoint.nodeID == lastBoundaryNode
            || waypoint.nodeID != command.finalNodeID
            || std::fabs(waypoint.targetSpeedMmPerSecond) > 0.001f)
        {
            return false;
        }

        const float deltaForward = waypoint.forwardMm - currentForward;
        const float deltaLeft = waypoint.leftMm - currentLeft;
        const float distance = std::hypot(deltaForward, deltaLeft);
        const float pathHeading = std::atan2(deltaLeft, deltaForward);
        if (distance < kMinimumDriveMm || distance > kMaximumDriveMm
            || std::fabs(normalizeAngle(pathHeading - currentHeading))
                   > kHeadingToleranceRad
            || std::fabs(normalizeAngle(waypoint.headingRad - currentHeading))
                   > kHeadingToleranceRad)
        {
            return false;
        }
        currentForward = waypoint.forwardMm;
        currentLeft = waypoint.leftMm;
        currentHeading = waypoint.headingRad;
        lastBoundaryNode = waypoint.nodeID;
    }
    return lastBoundaryNode == command.finalNodeID;
}

bool PhysicalFleetExecutor::commandEquals(
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
        const auto& a = lhs.waypoints[i];
        const auto& b = rhs.waypoints[i];
        if (a.forwardMm != b.forwardMm || a.leftMm != b.leftMm
            || a.headingRad != b.headingRad
            || a.targetSpeedMmPerSecond != b.targetSpeedMmPerSecond
            || a.nodeID != b.nodeID || a.flags != b.flags)
            return false;
    }
    return true;
}

bool PhysicalFleetExecutor::correctionCommandEquals(
    const RobotProtocol::NodeCorrectionCommandPayload& lhs,
    const RobotProtocol::NodeCorrectionCommandPayload& rhs)
{
    return lhs.routeID == rhs.routeID
        && lhs.nodeID == rhs.nodeID
        && lhs.commandID == rhs.commandID
        && lhs.action == rhs.action
        && lhs.magnitude == rhs.magnitude;
}

bool PhysicalFleetExecutor::hasFlag(
    const RobotProtocol::TrajectoryWaypoint& waypoint,
    uint8_t flag)
{
    return (waypoint.flags & flag) != 0;
}

float PhysicalFleetExecutor::normalizeAngle(float angle)
{
    if (!std::isfinite(angle))
        return 0.0f;
    return std::remainder(angle, 2.0f * kPi);
}

void PhysicalFleetExecutor::clearCommand()
{
    m_Command = {};
    m_HasCommand = false;
    m_Cursor = 0;
    m_TargetNodeID = 0;
    m_ArrivalNodeID = 0;
    m_Primitive = PrimitiveKind::NONE;
    m_MotionMode = MotionController::Mode::NONE;
    m_RemainingTurnQuarters = 0;
}

void PhysicalFleetExecutor::stageCorrectionReport(
    RobotProtocol::NodeCorrectionResult result,
    uint32_t detail,
    State nextState)
{
    if (!m_HasLastCorrectionCommand)
        return;
    stageCorrectionReportFor(m_LastCorrectionCommand,
                             result,
                             detail,
                             nextState);
}

void PhysicalFleetExecutor::stageCorrectionReportFor(
    const RobotProtocol::NodeCorrectionCommandPayload& command,
    RobotProtocol::NodeCorrectionResult result,
    uint32_t detail,
    State nextState)
{
    // A rejected command also consumes its command ID. Remember every
    // command for which a terminal report is staged so an exact retry cannot
    // produce a second report after the first wire attempt has completed.
    m_LastCorrectionCommand = command;
    m_HasLastCorrectionCommand = true;

    RobotProtocol::NodeCorrectionReportPayload report;
    report.routeID = command.routeID;
    report.nodeID = command.nodeID;
    report.commandID = command.commandID;
    report.result = result;
    report.detail = detail;

    m_PendingCorrectionReport = report;
    m_HasPendingCorrectionReport = true;
    m_AfterCorrectionReportState = nextState;
    m_State = State::CORRECTION_REPORT_PENDING;
}

void PhysicalFleetExecutor::stageActiveCorrectionFault(uint32_t detail)
{
    stageCorrectionReport(RobotProtocol::NodeCorrectionResult::FAULT,
                          detail,
                          State::FAULT_LATCHED);
}

void PhysicalFleetExecutor::resetCorrectionSession(uint32_t completedRouteID)
{
    m_LastCompletedRouteID = completedRouteID;
    m_ActiveCorrectionHeadingDeltaRad = 0.0f;
    m_CorrectionStartedMs = 0;
    m_CorrectionPrimitiveCount = 0;
    m_HasLastCorrectionCommand = false;
    m_LastCorrectionCommand = {};
    m_HasPendingCorrectionReport = false;
    m_PendingCorrectionReport = {};
    m_AfterCorrectionReportState = State::NODE_WAIT;
}

void PhysicalFleetExecutor::latchFault(Fault fault, uint32_t detail)
{
    m_Motion.stopImmediately();
    if (m_Fault == Fault::NONE)
    {
        m_Fault = fault;
        m_FaultDetail = detail;
    }
    m_State = State::FAULT_LATCHED;
}

bool PhysicalFleetExecutor::terminalLatch() const
{
    return m_State == State::OUTPUT_LOCKED
        || m_State == State::FAULT_LATCHED
        || m_State == State::ESTOP_LATCHED;
}

const char* PhysicalFleetExecutor::stateName(State state)
{
    switch (state)
    {
    case State::IDLE:            return "IDLE";
    case State::READY:           return "READY";
    case State::SAFE_PAUSE:      return "SAFE_PAUSE";
    case State::RUNNING:         return "RUNNING";
    case State::SETTLING:        return "SETTLING";
    case State::ARRIVAL_PENDING: return "ARRIVAL_PENDING";
    case State::NODE_WAIT:       return "NODE_WAIT";
    case State::CORRECTION_RUNNING:
        return "CORRECTION_RUNNING";
    case State::CORRECTION_SETTLING:
        return "CORRECTION_SETTLING";
    case State::CORRECTION_REPORT_PENDING:
        return "CORRECTION_REPORT_PENDING";
    case State::OUTPUT_LOCKED:   return "OUTPUT_LOCKED";
    case State::FAULT_LATCHED:   return "FAULT_LATCHED";
    case State::ESTOP_LATCHED:   return "ESTOP_LATCHED";
    default:                     return "UNKNOWN";
    }
}

const char* PhysicalFleetExecutor::correctionAcceptResultName(
    CorrectionAcceptResult result)
{
    switch (result)
    {
    case CorrectionAcceptResult::STARTED:
        return "STARTED";
    case CorrectionAcceptResult::DUPLICATE_IGNORED:
        return "DUPLICATE_IGNORED";
    case CorrectionAcceptResult::REJECTED_INVALID:
        return "REJECTED_INVALID";
    case CorrectionAcceptResult::REJECTED_SESSION_NOT_READY:
        return "REJECTED_SESSION_NOT_READY";
    case CorrectionAcceptResult::REJECTED_BUSY:
        return "REJECTED_BUSY";
    case CorrectionAcceptResult::REJECTED_LATCHED:
        return "REJECTED_LATCHED";
    default:
        return "UNKNOWN";
    }
}

const char* PhysicalFleetExecutor::acceptResultName(AcceptResult result)
{
    switch (result)
    {
    case AcceptResult::STORED:                     return "STORED";
    case AcceptResult::DUPLICATE_IGNORED:          return "DUPLICATE_IGNORED";
    case AcceptResult::REJECTED_INVALID:            return "REJECTED_INVALID";
    case AcceptResult::REJECTED_SESSION_NOT_READY:  return "REJECTED_SESSION_NOT_READY";
    case AcceptResult::REJECTED_BUSY:               return "REJECTED_BUSY";
    case AcceptResult::REJECTED_LATCHED:            return "REJECTED_LATCHED";
    default:                                        return "UNKNOWN";
    }
}

const char* PhysicalFleetExecutor::faultName(Fault fault)
{
    switch (fault)
    {
    case Fault::NONE:                return "NONE";
    case Fault::INVALID_COMMAND:     return "INVALID_COMMAND";
    case Fault::CONFLICTING_COMMAND: return "CONFLICTING_COMMAND";
    case Fault::CANCELLED_DURING_MOTION:
        return "CANCELLED_DURING_MOTION";
    case Fault::NETWORK_LOST:        return "NETWORK_LOST";
    case Fault::MOTION_START_FAILED: return "MOTION_START_FAILED";
    case Fault::MOTION_CONTROLLER:   return "MOTION_CONTROLLER";
    case Fault::OUTPUTS_NOT_SAFE:    return "OUTPUTS_NOT_SAFE";
    case Fault::ARRIVED_SEND_FAILED: return "ARRIVED_SEND_FAILED";
    case Fault::INVALID_CORRECTION:  return "INVALID_CORRECTION";
    case Fault::CORRECTION_REPORT_SEND_FAILED:
        return "CORRECTION_REPORT_SEND_FAILED";
    default:                         return "UNKNOWN";
    }
}
