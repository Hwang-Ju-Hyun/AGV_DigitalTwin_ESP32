#pragma once

#include "MotionController.hpp"
#include "RobotProtocol.hpp"

#include <cstdint>

class PhysicalFleetExecutor
{
public:
    enum class State : uint8_t
    {
        IDLE,
        READY,
        SAFE_PAUSE,
        RUNNING,
        SETTLING,
        ARRIVAL_PENDING,
        NODE_WAIT,
        CORRECTION_RUNNING,
        CORRECTION_SETTLING,
        CORRECTION_REPORT_PENDING,
        OUTPUT_LOCKED,
        FAULT_LATCHED,
        ESTOP_LATCHED
    };

    enum class AcceptResult : uint8_t
    {
        STORED,
        DUPLICATE_IGNORED,
        REJECTED_INVALID,
        REJECTED_SESSION_NOT_READY,
        REJECTED_BUSY,
        REJECTED_LATCHED
    };

    enum class CorrectionAcceptResult : uint8_t
    {
        STARTED,
        DUPLICATE_IGNORED,
        REJECTED_INVALID,
        REJECTED_SESSION_NOT_READY,
        REJECTED_BUSY,
        REJECTED_LATCHED
    };

    enum class Fault : uint8_t
    {
        NONE,
        INVALID_COMMAND,
        CONFLICTING_COMMAND,
        CANCELLED_DURING_MOTION,
        NETWORK_LOST,
        MOTION_START_FAILED,
        MOTION_CONTROLLER,
        OUTPUTS_NOT_SAFE,
        ARRIVED_SEND_FAILED,
        INVALID_CORRECTION,
        CORRECTION_REPORT_SEND_FAILED
    };

    explicit PhysicalFleetExecutor(MotionController& motion);

    void begin(uint32_t startNodeID, float startWorldHeadingRad);
    AcceptResult acceptTrajectory(
        const RobotProtocol::TrajectoryCommandPayload& command,
        bool sessionReady,
        uint32_t nowMs);
    CorrectionAcceptResult acceptNodeCorrection(
        const RobotProtocol::NodeCorrectionCommandPayload& command,
        bool sessionReady,
        uint32_t nowMs);
    void update(uint32_t nowMs, bool sessionReady);
    void cancel();
    void onNetworkLost();
    void emergencyStop();

    bool arrivalPending(uint32_t& outNodeID) const;
    void markArrivedSendResult(bool sent, uint32_t nowMs);
    bool correctionReportPending(
        RobotProtocol::NodeCorrectionReportPayload& outReport) const;
    void markCorrectionReportSendResult(bool sent);
    RobotProtocol::StatusPayload buildStatus() const;

    State state() const { return m_State; }
    Fault fault() const { return m_Fault; }
    uint32_t faultDetail() const { return m_FaultDetail; }
    bool hasActiveCommand() const { return m_HasCommand; }
    uint32_t routeID() const { return m_HasCommand ? m_Command.routeID : 0; }
    uint32_t currentNodeID() const { return m_CurrentNodeID; }
    uint16_t waypointIndex() const { return m_Cursor; }

    static const char* stateName(State state);
    static const char* acceptResultName(AcceptResult result);
    static const char* correctionAcceptResultName(
        CorrectionAcceptResult result);
    static const char* faultName(Fault fault);

private:
    enum class PrimitiveKind : uint8_t
    {
        NONE,
        DRIVE,
        TURN,
        CORRECTION_DRIVE,
        CORRECTION_TURN
    };

    static bool hasFlag(const RobotProtocol::TrajectoryWaypoint& waypoint,
                        uint8_t flag);
    static float normalizeAngle(float angle);
    static bool commandEquals(
        const RobotProtocol::TrajectoryCommandPayload& lhs,
        const RobotProtocol::TrajectoryCommandPayload& rhs);
    static bool validateCommand(
        const RobotProtocol::TrajectoryCommandPayload& command);
    static bool correctionCommandEquals(
        const RobotProtocol::NodeCorrectionCommandPayload& lhs,
        const RobotProtocol::NodeCorrectionCommandPayload& rhs);

    void startCurrentWaypoint(uint32_t nowMs);
    void startTurnQuarter(uint32_t nowMs);
    void handleMotionComplete(uint32_t nowMs);
    void stageCorrectionReport(
        RobotProtocol::NodeCorrectionResult result,
        uint32_t detail,
        State nextState);
    void stageCorrectionReportFor(
        const RobotProtocol::NodeCorrectionCommandPayload& command,
        RobotProtocol::NodeCorrectionResult result,
        uint32_t detail,
        State nextState);
    void stageActiveCorrectionFault(uint32_t detail);
    void resetCorrectionSession(uint32_t completedRouteID);
    void clearCommand();
    void latchFault(Fault fault, uint32_t detail);
    bool terminalLatch() const;

    MotionController& m_Motion;
    RobotProtocol::TrajectoryCommandPayload m_Command{};
    State m_State = State::IDLE;
    Fault m_Fault = Fault::NONE;
    uint32_t m_FaultDetail = 0;
    bool m_HasCommand = false;
    uint16_t m_Cursor = 0;
    uint32_t m_CurrentNodeID = 0;
    uint32_t m_TargetNodeID = 0;
    uint32_t m_ArrivalNodeID = 0;
    uint32_t m_LastCompletedRouteID = 0;
    uint32_t m_PauseStartedMs = 0;
    PrimitiveKind m_Primitive = PrimitiveKind::NONE;
    MotionController::Mode m_MotionMode = MotionController::Mode::NONE;
    uint8_t m_RemainingTurnQuarters = 0;
    float m_CurrentForwardMm = 0.0f;
    float m_CurrentLeftMm = 0.0f;
    float m_CurrentLocalHeadingRad = 0.0f;
    float m_TurnStartLocalHeadingRad = 0.0f;
    float m_TurnQuarterDirectionRad = 0.0f;
    float m_CommandOriginWorldHeadingRad = 0.0f;
    float m_WorldHeadingRad = 0.0f;
    float m_ActiveCorrectionHeadingDeltaRad = 0.0f;
    uint8_t m_CorrectionPrimitiveCount = 0;
    bool m_HasLastCorrectionCommand = false;
    RobotProtocol::NodeCorrectionCommandPayload m_LastCorrectionCommand{};
    bool m_HasPendingCorrectionReport = false;
    RobotProtocol::NodeCorrectionReportPayload m_PendingCorrectionReport{};
    uint32_t m_CorrectionStartedMs = 0;
    State m_AfterCorrectionReportState = State::NODE_WAIT;
};
