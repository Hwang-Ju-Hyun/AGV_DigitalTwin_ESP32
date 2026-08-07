#pragma once

#include "RobotProtocol.hpp"

#include <cstdint>

class MotionController;

class RouteExecutor
{
public:
    enum class State : uint8_t
    {
        DISARMED_NO_ROUTE,
        WAIT_BOOT,
        COUNTDOWN,
        RUNNING,
        SETTLING,
        ARRIVAL_PENDING,
        ARRIVAL_REPORTED,
        OUTPUT_LOCKED,
        FAULT_LATCHED,
        ESTOP_LATCHED
    };

    enum class RouteResult : uint8_t
    {
        STORED,
        DUPLICATE_IGNORED,
        REJECTED_INVALID_ROUTE,
        REJECTED_SESSION_NOT_READY,
        REJECTED_BUSY,
        REJECTED_LATCHED
    };

    enum class BootResult : uint8_t
    {
        COUNTDOWN_STARTED,
        COUNTDOWN_CANCELLED,
        ESTOP_LATCHED,
        REJECTED_NOT_READY,
        IGNORED
    };

    enum class Fault : uint8_t
    {
        NONE,
        NETWORK_LOST,
        MOTION_START_FAILED,
        MOTION_CONTROLLER,
        OUTPUTS_NOT_SAFE,
        ARRIVED_SEND_FAILED
    };

    explicit RouteExecutor(MotionController& motionController);

    void begin();

    RouteResult acceptRoute(const RobotProtocol::RouteCommandPayload& route,
                            bool sessionReady);
    BootResult handleBootPress(uint32_t nowMs, bool sessionReady);
    void update(uint32_t nowMs, bool sessionReady);

    // Call this before any reconnect or protocol work when TCP loss is seen.
    void onNetworkLost();
    void cancelRoute();
    void emergencyStop();

    // The caller makes one send attempt and must immediately report its result.
    bool arrivalPending(uint32_t& outNodeID) const;
    void markArrivedSendResult(bool sent);

    RobotProtocol::StatusPayload buildStatus() const;

    State state() const { return m_State; }
    Fault fault() const { return m_Fault; }
    uint32_t faultDetail() const { return m_FaultDetail; }
    bool hasRoute() const { return m_HasRoute; }
    uint32_t routeID() const { return m_HasRoute ? m_Route.routeID : 0; }
    uint32_t countdownRemainingMs(uint32_t nowMs) const;

    static const char* stateName(State state);
    static const char* routeResultName(RouteResult result);
    static const char* bootResultName(BootResult result);
    static const char* faultName(Fault fault);

private:
    static bool isExactDemoRoute(const RobotProtocol::RouteCommandPayload& route);
    bool isSameStoredRoute(const RobotProtocol::RouteCommandPayload& route) const;
    bool isTerminalLatch() const;
    void clearStoredRoute();
    void latchFault(Fault fault, uint32_t detail);

    MotionController& m_Motion;
    RobotProtocol::RouteCommandPayload m_Route{};
    State m_State = State::DISARMED_NO_ROUTE;
    Fault m_Fault = Fault::NONE;
    uint32_t m_FaultDetail = 0;
    uint32_t m_CountdownStartedMs = 0;
    bool m_HasRoute = false;
    bool m_MotionCompleted = false;
};
