#include "RouteExecutor.hpp"

#include "Config.hpp"
#include "MotionController.hpp"

namespace
{
    constexpr uint32_t kFaultDetailNetworkLost = 1;
    constexpr uint32_t kFaultDetailMotionStartFailed = 2;
    constexpr uint32_t kFaultDetailOutputsNotSafe = 3;
    constexpr uint32_t kFaultDetailArrivedSendFailed = 4;
    constexpr uint32_t kMotionFaultDetailPrefix = 0x10000UL;
}

RouteExecutor::RouteExecutor(MotionController& motionController)
    : m_Motion(motionController)
{
}

void RouteExecutor::begin()
{
    m_Motion.begin();
    m_Motion.stopImmediately();
    clearStoredRoute();
    m_State = State::DISARMED_NO_ROUTE;
    m_Fault = Fault::NONE;
    m_FaultDetail = 0;
    m_CountdownStartedMs = 0;
    m_MotionCompleted = false;
}

RouteExecutor::RouteResult RouteExecutor::acceptRoute(
    const RobotProtocol::RouteCommandPayload& route,
    bool sessionReady)
{
    if (!isExactDemoRoute(route))
        return RouteResult::REJECTED_INVALID_ROUTE;

    if (!sessionReady)
        return RouteResult::REJECTED_SESSION_NOT_READY;

    if (isTerminalLatch())
        return RouteResult::REJECTED_LATCHED;

    if (m_HasRoute)
    {
        return isSameStoredRoute(route)
            ? RouteResult::DUPLICATE_IGNORED
            : RouteResult::REJECTED_BUSY;
    }

    if (m_State != State::DISARMED_NO_ROUTE)
        return RouteResult::REJECTED_BUSY;

    m_Motion.stopImmediately();
    if (!m_Motion.outputsSafe())
    {
        latchFault(Fault::OUTPUTS_NOT_SAFE, kFaultDetailOutputsNotSafe);
        return RouteResult::REJECTED_LATCHED;
    }

    m_Route = route;
    m_HasRoute = true;
    m_MotionCompleted = false;
    m_State = State::WAIT_BOOT;
    return RouteResult::STORED;
}

RouteExecutor::BootResult RouteExecutor::handleBootPress(uint32_t nowMs,
                                                         bool sessionReady)
{
    if (m_State == State::COUNTDOWN)
    {
        cancelRoute();
        return BootResult::COUNTDOWN_CANCELLED;
    }

    if (m_State != State::WAIT_BOOT)
        return BootResult::IGNORED;

    if (!m_HasRoute || !sessionReady)
    {
        if (!sessionReady)
            onNetworkLost();
        else
            cancelRoute();
        return BootResult::REJECTED_NOT_READY;
    }

    m_Motion.stopImmediately();
    if (!m_Motion.outputsSafe())
    {
        latchFault(Fault::OUTPUTS_NOT_SAFE, kFaultDetailOutputsNotSafe);
        return BootResult::REJECTED_NOT_READY;
    }

    m_CountdownStartedMs = nowMs;
    m_State = State::COUNTDOWN;
    return BootResult::COUNTDOWN_STARTED;
}

void RouteExecutor::update(uint32_t nowMs, bool sessionReady)
{
    if (!sessionReady)
    {
        onNetworkLost();
        return;
    }

    if (m_State == State::COUNTDOWN)
    {
        if (!m_HasRoute)
        {
            cancelRoute();
            return;
        }

        if (nowMs - m_CountdownStartedMs < AppConfig::kApprovalCountdownMs)
            return;

        if (!AppConfig::kEnableMotorOutputs)
        {
            m_Motion.stopImmediately();
            if (!m_Motion.outputsSafe())
                latchFault(Fault::OUTPUTS_NOT_SAFE, kFaultDetailOutputsNotSafe);
            else
                m_State = State::OUTPUT_LOCKED;
            return;
        }

        const MotionController::StartResult startResult =
            m_Motion.startForward(AppConfig::kForward30CmCount);
        if (startResult != MotionController::StartResult::STARTED)
        {
            latchFault(Fault::MOTION_START_FAILED,
                       kFaultDetailMotionStartFailed
                           | (static_cast<uint32_t>(startResult) << 8));
            return;
        }

        m_State = State::RUNNING;
        return;
    }

    if (m_State != State::RUNNING)
        return;

    const MotionController::UpdateResult updateResult = m_Motion.update(nowMs);
    if (m_Motion.faultLatched()
        || updateResult == MotionController::UpdateResult::FAULTED)
    {
        latchFault(Fault::MOTION_CONTROLLER,
                   kMotionFaultDetailPrefix
                       | (static_cast<uint32_t>(m_Motion.fault()) & 0xFFFFUL));
        return;
    }

    if (updateResult == MotionController::UpdateResult::RUNNING)
        return;

    if (updateResult != MotionController::UpdateResult::COMPLETE)
    {
        latchFault(Fault::MOTION_CONTROLLER,
                   kMotionFaultDetailPrefix
                       | (static_cast<uint32_t>(m_Motion.fault()) & 0xFFFFUL));
        return;
    }

    // MotionController must have already set PWM=0 and STBY=LOW.
    if (!m_Motion.outputsSafe())
    {
        latchFault(Fault::OUTPUTS_NOT_SAFE, kFaultDetailOutputsNotSafe);
        return;
    }

    m_MotionCompleted = true;
    m_State = State::ARRIVAL_PENDING;
}

void RouteExecutor::onNetworkLost()
{
    // This is intentionally the first operation. Reconnect/protocol handling
    // belongs after this call in the application loop.
    m_Motion.stopImmediately();

    if (!m_Motion.outputsSafe())
    {
        latchFault(Fault::OUTPUTS_NOT_SAFE, kFaultDetailOutputsNotSafe);
        return;
    }

    switch (m_State)
    {
    case State::WAIT_BOOT:
    case State::COUNTDOWN:
        clearStoredRoute();
        m_State = State::DISARMED_NO_ROUTE;
        break;

    case State::RUNNING:
    case State::ARRIVAL_PENDING:
        latchFault(Fault::NETWORK_LOST, kFaultDetailNetworkLost);
        break;

    default:
        break;
    }
}

void RouteExecutor::cancelRoute()
{
    m_Motion.stopImmediately();
    if (!m_Motion.outputsSafe())
    {
        latchFault(Fault::OUTPUTS_NOT_SAFE, kFaultDetailOutputsNotSafe);
        return;
    }

    if (m_Motion.faultLatched())
    {
        latchFault(Fault::MOTION_CONTROLLER,
                   kMotionFaultDetailPrefix
                       | (static_cast<uint32_t>(m_Motion.fault()) & 0xFFFFUL));
        return;
    }

    if (isTerminalLatch())
        return;

    clearStoredRoute();
    m_State = State::DISARMED_NO_ROUTE;
}

void RouteExecutor::emergencyStop()
{
    m_Motion.emergencyStop(MotionController::Fault::EXTERNAL_STOP);
    m_State = State::ESTOP_LATCHED;
    m_Fault = Fault::NONE;
    m_FaultDetail = 0;
}

bool RouteExecutor::arrivalPending(uint32_t& outNodeID) const
{
    if (m_State != State::ARRIVAL_PENDING
        || !m_HasRoute
        || !m_MotionCompleted
        || !m_Motion.outputsSafe())
    {
        return false;
    }

    outNodeID = AppConfig::kDemoTargetNodeID;
    return true;
}

void RouteExecutor::markArrivedSendResult(bool sent)
{
    if (m_State != State::ARRIVAL_PENDING)
        return;

    if (!sent)
    {
        latchFault(Fault::ARRIVED_SEND_FAILED,
                   kFaultDetailArrivedSendFailed);
        return;
    }

    if (!m_Motion.outputsSafe())
    {
        latchFault(Fault::OUTPUTS_NOT_SAFE, kFaultDetailOutputsNotSafe);
        return;
    }

    m_State = State::ARRIVAL_REPORTED;
}

RobotProtocol::StatusPayload RouteExecutor::buildStatus() const
{
    const MotionController::Snapshot snapshot = m_Motion.snapshot();
    RobotProtocol::StatusPayload status;
    status.currentNodeID = m_MotionCompleted
        ? AppConfig::kDemoTargetNodeID
        : AppConfig::kDemoStartNodeID;
    const bool hasEncoderLegProgress = !m_MotionCompleted
        && snapshot.progress > 0.0f
        && (m_State == State::RUNNING
            || m_State == State::FAULT_LATCHED
            || m_State == State::ESTOP_LATCHED);
    // Server ee3244f interprets this field as the target node ID when deriving
    // the node-to-node pose for Unity, despite its historical field name.
    status.currentLinkID = (m_State == State::RUNNING || hasEncoderLegProgress)
        ? AppConfig::kDemoTargetNodeID
        : 0;
    status.progress = m_MotionCompleted ? 1.0f : 0.0f;
    status.x = 0.0f;
    status.z = 0.0f;
    status.heading = 0.0f;
    status.velocity = 0.0f;
    status.battery = 100.0f; // Placeholder while the battery remains isolated.
    status.state = RobotProtocol::RobotState::IDLE;

    if (m_State == State::RUNNING)
    {
        status.progress = snapshot.progress;
        status.state = RobotProtocol::RobotState::MOVING;
    }
    else if (m_State == State::FAULT_LATCHED)
    {
        if (!m_MotionCompleted)
            status.progress = snapshot.progress;
        status.state = RobotProtocol::RobotState::FAULT;
    }
    else if (m_State == State::ESTOP_LATCHED)
    {
        if (!m_MotionCompleted)
            status.progress = snapshot.progress;
        status.state = RobotProtocol::RobotState::EMERGENCY_STOPPED;
    }

    return status;
}

uint32_t RouteExecutor::countdownRemainingMs(uint32_t nowMs) const
{
    if (m_State != State::COUNTDOWN)
        return 0;

    const uint32_t elapsed = nowMs - m_CountdownStartedMs;
    return elapsed >= AppConfig::kApprovalCountdownMs
        ? 0
        : AppConfig::kApprovalCountdownMs - elapsed;
}

const char* RouteExecutor::stateName(State state)
{
    switch (state)
    {
    case State::DISARMED_NO_ROUTE: return "DISARMED_NO_ROUTE";
    case State::WAIT_BOOT:         return "WAIT_BOOT";
    case State::COUNTDOWN:         return "COUNTDOWN";
    case State::RUNNING:           return "RUNNING";
    case State::ARRIVAL_PENDING:   return "ARRIVAL_PENDING";
    case State::ARRIVAL_REPORTED:  return "ARRIVAL_REPORTED";
    case State::OUTPUT_LOCKED:     return "OUTPUT_LOCKED";
    case State::FAULT_LATCHED:     return "FAULT_LATCHED";
    case State::ESTOP_LATCHED:     return "ESTOP_LATCHED";
    default:                       return "UNKNOWN";
    }
}

const char* RouteExecutor::routeResultName(RouteResult result)
{
    switch (result)
    {
    case RouteResult::STORED:                     return "STORED";
    case RouteResult::DUPLICATE_IGNORED:          return "DUPLICATE_IGNORED";
    case RouteResult::REJECTED_INVALID_ROUTE:     return "REJECTED_INVALID_ROUTE";
    case RouteResult::REJECTED_SESSION_NOT_READY: return "REJECTED_SESSION_NOT_READY";
    case RouteResult::REJECTED_BUSY:              return "REJECTED_BUSY";
    case RouteResult::REJECTED_LATCHED:           return "REJECTED_LATCHED";
    default:                                      return "UNKNOWN";
    }
}

const char* RouteExecutor::bootResultName(BootResult result)
{
    switch (result)
    {
    case BootResult::COUNTDOWN_STARTED:   return "COUNTDOWN_STARTED";
    case BootResult::COUNTDOWN_CANCELLED: return "COUNTDOWN_CANCELLED";
    case BootResult::REJECTED_NOT_READY:  return "REJECTED_NOT_READY";
    case BootResult::IGNORED:             return "IGNORED";
    default:                              return "UNKNOWN";
    }
}

const char* RouteExecutor::faultName(Fault fault)
{
    switch (fault)
    {
    case Fault::NONE:                return "NONE";
    case Fault::NETWORK_LOST:        return "NETWORK_LOST";
    case Fault::MOTION_START_FAILED: return "MOTION_START_FAILED";
    case Fault::MOTION_CONTROLLER:   return "MOTION_CONTROLLER";
    case Fault::OUTPUTS_NOT_SAFE:    return "OUTPUTS_NOT_SAFE";
    case Fault::ARRIVED_SEND_FAILED: return "ARRIVED_SEND_FAILED";
    default:                         return "UNKNOWN";
    }
}

bool RouteExecutor::isExactDemoRoute(
    const RobotProtocol::RouteCommandPayload& route)
{
    return route.routeID != 0
        && route.nodeCount == 2
        && route.nodes[0].nodeID == AppConfig::kDemoStartNodeID
        && route.nodes[1].nodeID == AppConfig::kDemoTargetNodeID;
}

bool RouteExecutor::isSameStoredRoute(
    const RobotProtocol::RouteCommandPayload& route) const
{
    return m_HasRoute
        && route.routeID == m_Route.routeID
        && route.nodeCount == m_Route.nodeCount
        && route.nodes[0].nodeID == m_Route.nodes[0].nodeID
        && route.nodes[1].nodeID == m_Route.nodes[1].nodeID;
}

bool RouteExecutor::isTerminalLatch() const
{
    return m_State == State::ARRIVAL_REPORTED
        || m_State == State::OUTPUT_LOCKED
        || m_State == State::FAULT_LATCHED
        || m_State == State::ESTOP_LATCHED;
}

void RouteExecutor::clearStoredRoute()
{
    m_Route = RobotProtocol::RouteCommandPayload{};
    m_HasRoute = false;
    m_MotionCompleted = false;
    m_CountdownStartedMs = 0;
}

void RouteExecutor::latchFault(Fault fault, uint32_t detail)
{
    m_Motion.stopImmediately();
    m_Fault = fault;
    m_FaultDetail = detail;
    m_State = State::FAULT_LATCHED;
}
