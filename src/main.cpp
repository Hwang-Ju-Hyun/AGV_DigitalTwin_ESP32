#include <Arduino.h>
#include <cstring>

#include "Config.hpp"
#include "MotionController.hpp"
#include "RobotClient.hpp"
#include "RouteExecutor.hpp"

static_assert(AppConfig::kRaisedWheelBuild
                  == AppConfig::kEnableMotorOutputs,
              "SAFETY FAILURE: build profile and motor output disagree");

#if AGV_RAISED_WHEEL_BUILD
static_assert(AppConfig::kRaisedWheelBuild
                  && AppConfig::kEnableMotorOutputs,
              "RAISED-WHEEL SAFETY FAILURE: outputs must be enabled");
#else
static_assert(!AppConfig::kRaisedWheelBuild
                  && !AppConfig::kEnableMotorOutputs,
              "MOTOR-LOCK SAFETY FAILURE: outputs must remain disabled");
#endif

namespace
{
    RobotClient robotClient;
    MotionController motionController;
    RouteExecutor routeExecutor(motionController);

    bool networkConfigured = false;
    uint32_t lastStatusMs = 0;
    uint32_t lastStatusLogMs = 0;
    bool faultReported = false;
    uint32_t lastFaultDetail = 0;
    RouteExecutor::State lastLoggedState =
        RouteExecutor::State::DISARMED_NO_ROUTE;

    bool lastButtonReading = HIGH;
    bool stableButtonState = HIGH;
    bool buttonArmed = true;
    uint32_t lastButtonChangeMs = 0;

    bool sessionReady()
    {
        return robotClient.connected() && robotClient.accepted();
    }

    bool credentialsAreConfigured()
    {
        return std::strcmp(AppConfig::kWifiSsid, "CHANGE_ME_WIFI_SSID") != 0
            && std::strcmp(AppConfig::kWifiPassword,
                           "CHANGE_ME_WIFI_PASSWORD") != 0;
    }

    void initializeBootButton()
    {
        pinMode(AppConfig::kBootButtonPin, INPUT_PULLUP);
        const bool initialState = digitalRead(AppConfig::kBootButtonPin);
        lastButtonReading = initialState;
        stableButtonState = initialState;
        buttonArmed = initialState == HIGH;
        lastButtonChangeMs = millis();
    }

    bool bootButtonPressed(uint32_t nowMs)
    {
        const bool reading = digitalRead(AppConfig::kBootButtonPin);

        if (reading != lastButtonReading)
        {
            lastButtonReading = reading;
            lastButtonChangeMs = nowMs;
        }

        if (nowMs - lastButtonChangeMs < AppConfig::kButtonDebounceMs)
            return false;

        if (reading == stableButtonState)
            return false;

        stableButtonState = reading;
        if (stableButtonState == HIGH)
        {
            buttonArmed = true;
            return false;
        }

        if (!buttonArmed)
            return false;

        buttonArmed = false;
        return true;
    }

    void printRoute(const RobotProtocol::RouteCommandPayload& route,
                    bool includeNodeDetails)
    {
        Serial.printf("[ROUTE] routeID=%lu nodeCount=%u\n",
                      static_cast<unsigned long>(route.routeID),
                      route.nodeCount);

        // Only the newly stored two-node demo route needs detailed logging.
        // Invalid, duplicate, or in-motion packets cannot create a long Serial
        // burst that delays encoder and fault servicing.
        if (!includeNodeDetails)
            return;

        for (uint16_t i = 0; i < route.nodeCount; ++i)
        {
            Serial.printf("  node[%u]=%lu arrival=%.3f departure=%.3f\n",
                          i,
                          static_cast<unsigned long>(route.nodes[i].nodeID),
                          route.nodes[i].arrivalTime,
                          route.nodes[i].departureTime);
        }
    }

    void printStateTransition(RouteExecutor::State state)
    {
        if (state == lastLoggedState)
            return;

        lastLoggedState = state;
        const MotionController::Snapshot snapshot = motionController.snapshot();
        Serial.printf("[EXECUTOR] state=%s | L=%ld R=%ld | PWM=%d/%d | safe=%u\n",
                      RouteExecutor::stateName(state),
                      static_cast<long>(snapshot.leftCount),
                      static_cast<long>(snapshot.rightCount),
                      snapshot.leftPwm,
                      snapshot.rightPwm,
                      snapshot.outputsSafe ? 1U : 0U);

        switch (state)
        {
        case RouteExecutor::State::WAIT_BOOT:
            Serial.println("[SAFE] Exact [1 -> 2] stored; press BOOT once");
            break;
        case RouteExecutor::State::COUNTDOWN:
            Serial.println("[SAFE] 5 second countdown; press BOOT again to cancel");
            break;
        case RouteExecutor::State::RUNNING:
            Serial.println("[MOTION] 30 cm encoder drive started; target=520");
            break;
        case RouteExecutor::State::SETTLING:
            Serial.println("[SAFE] Target reached; PWM=0 STBY=LOW");
            Serial.println("[MOTION] Verifying encoder stability for 150 ms");
            break;
        case RouteExecutor::State::ARRIVAL_PENDING:
            Serial.println("[MOTION] Encoder stability verified; arrival pending");
            break;
        case RouteExecutor::State::ARRIVAL_REPORTED:
            Serial.println("[RobotProtocol] ARRIVED sent; run latched complete");
            break;
        case RouteExecutor::State::OUTPUT_LOCKED:
            Serial.println("[SAFE] Motor compile lock blocked physical start");
            Serial.println("[SAFE] STBY=LOW PWM=0; ARRIVED remains blocked");
            break;
        case RouteExecutor::State::FAULT_LATCHED:
            Serial.printf("[FAULT] %s detail=%lu; reboot required\n",
                          RouteExecutor::faultName(routeExecutor.fault()),
                          static_cast<unsigned long>(routeExecutor.faultDetail()));
            break;
        case RouteExecutor::State::ESTOP_LATCHED:
            Serial.println("[SAFE] EMERGENCY_STOP latched; reboot required");
            break;
        default:
            break;
        }
    }

    void handleBootPress(uint32_t nowMs)
    {
        const RouteExecutor::BootResult result =
            routeExecutor.handleBootPress(nowMs, sessionReady());
        Serial.printf("[BOOT] %s\n", RouteExecutor::bootResultName(result));

        if (result == RouteExecutor::BootResult::COUNTDOWN_CANCELLED)
        {
            Serial.printf("[SAFE] Countdown cancelled; routeID=%lu retained\n",
                          static_cast<unsigned long>(routeExecutor.routeID()));
        }
        else if (result == RouteExecutor::BootResult::ESTOP_LATCHED)
        {
            Serial.println("[SAFE] BOOT emergency stop: PWM=0 STBY=LOW");
        }
    }

    void configureCallbacks()
    {
        robotClient.onAccepted = [](uint32_t agvID)
        {
            Serial.printf("[SAFE] SERVER ACCEPTED AGV %lu\n",
                          static_cast<unsigned long>(agvID));
#if AGV_RAISED_WHEEL_BUILD
            Serial.println("[WARNING] MOTOR OUTPUTS ENABLED: RAISED-WHEEL ONLY");
#else
            Serial.println("[SAFE] MOTOR OUTPUTS REMAIN COMPILE-LOCKED OFF");
#endif
        };

        robotClient.onDisconnected = []()
        {
            // RobotClient invokes this before closing/clearing its socket.
            routeExecutor.onNetworkLost();
        };

        robotClient.onRouteCommand = [](
            const RobotProtocol::RouteCommandPayload& route)
        {
            const RouteExecutor::RouteResult result =
                routeExecutor.acceptRoute(route, sessionReady());
            printRoute(route, result == RouteExecutor::RouteResult::STORED);
            Serial.printf("[ROUTE] result=%s\n",
                          RouteExecutor::routeResultName(result));

            if (result == RouteExecutor::RouteResult::REJECTED_INVALID_ROUTE)
                Serial.println("[SAFE] Only exact [1 -> 2] is allowed");
            else if (result != RouteExecutor::RouteResult::STORED
                     && result != RouteExecutor::RouteResult::DUPLICATE_IGNORED)
                Serial.println("[SAFE] Route not executable in current state");
        };

        robotClient.onCancelRoute = []()
        {
            routeExecutor.cancelRoute();
            Serial.println("[SAFE] CANCEL_ROUTE handled; outputs forced safe");
        };

        robotClient.onEmergencyStop = []()
        {
            routeExecutor.emergencyStop();
            Serial.println("[SAFE] EMERGENCY_STOP handled immediately");
        };
    }

    void reportFaultIfNeeded()
    {
        if (routeExecutor.state() != RouteExecutor::State::FAULT_LATCHED)
        {
            faultReported = false;
            lastFaultDetail = 0;
            return;
        }

        const uint32_t detail = routeExecutor.faultDetail();
        if (detail != lastFaultDetail)
        {
            lastFaultDetail = detail;
            faultReported = false;
        }

        if (!faultReported && sessionReady())
        {
            faultReported = robotClient.sendError(
                RobotProtocol::ErrorCode::MOTOR_FAULT,
                detail);
            if (faultReported)
                Serial.println("[RobotProtocol] Fault ERROR_PACKET sent");
        }
    }

    void sendArrivalIfReady()
    {
        uint32_t arrivedNode = 0;
        if (!routeExecutor.arrivalPending(arrivedNode) || !sessionReady())
            return;

        // Publish the final encoder-derived pose before ARRIVED. Any failed
        // write closes the session, whose callback stops/latches the executor.
        if (!robotClient.sendStatus(routeExecutor.buildStatus()))
        {
            routeExecutor.markArrivedSendResult(false);
            return;
        }

        const bool sent = robotClient.sendArrived(arrivedNode);
        routeExecutor.markArrivedSendResult(sent);
    }

    void sendStatusIfDue(uint32_t nowMs)
    {
        if (!sessionReady()
            || nowMs - lastStatusMs < AppConfig::kStatusIntervalMs)
        {
            return;
        }

        lastStatusMs = nowMs;
        const RobotProtocol::StatusPayload status = routeExecutor.buildStatus();
        if (!robotClient.sendStatus(status))
            return;

        if (nowMs - lastStatusLogMs < AppConfig::kStatusLogIntervalMs)
            return;

        lastStatusLogMs = nowMs;
        const MotionController::Snapshot snapshot = motionController.snapshot();
        Serial.printf(
            "[STATUS] node=%lu target=%lu progress=%.3f state=%s "
            "L=%ld R=%ld PWM=%d/%d STBY=%s\n",
            static_cast<unsigned long>(status.currentNodeID),
            static_cast<unsigned long>(status.currentLinkID),
            status.progress,
            RouteExecutor::stateName(routeExecutor.state()),
            static_cast<long>(snapshot.leftCount),
            static_cast<long>(snapshot.rightCount),
            snapshot.leftPwm,
            snapshot.rightPwm,
            digitalRead(AppConfig::kMotorStandbyPin) == LOW ? "LOW" : "HIGH");
    }
}

void setup()
{
    // Motor/encoder hardware enters the safe state before Serial or Wi-Fi.
    routeExecutor.begin();
    initializeBootButton();

    Serial.begin(AppConfig::kSerialBaud);
    delay(300);

    Serial.println();
    Serial.println("================================");
#if AGV_RAISED_WHEEL_BUILD
    Serial.println("PHASE 2C: SERVER 30CM RAISED-WHEEL TEST");
#else
    Serial.println("PHASE 2C: SERVER 30CM MOTOR-LOCKED BUILD");
#endif
    Serial.println("ACCEPTED ROUTE: EXACT [1 -> 2] ONLY");
    Serial.println("MOTION TARGET: 520 ENCODER COUNTS");
#if AGV_RAISED_WHEEL_BUILD
    Serial.println("BUILD PROFILE: esp32dev-raised-wheel");
    Serial.println("MOTOR OUTPUTS: ENABLED");
    Serial.println("WARNING: WHEELS MUST REMAIN OFF THE FLOOR");
#else
    Serial.println("BUILD PROFILE: esp32dev (MOTOR LOCKED)");
    Serial.println("MOTOR OUTPUTS: COMPILE-LOCKED OFF");
#endif
    Serial.println("TB6612 STBY AT BOOT: LOW");
    Serial.println("ARRIVED: SAFE-COMPLETION GATED");
    Serial.println("BOOT REQUIRED BEFORE COUNTDOWN AND MOTION");
    Serial.println("================================");

    configureCallbacks();
    networkConfigured = credentialsAreConfigured();
    if (!networkConfigured)
    {
        Serial.println("[CONFIG] Configure local Secrets.hpp, then upload again");
        return;
    }

    robotClient.begin(AppConfig::kWifiSsid,
                      AppConfig::kWifiPassword,
                      AppConfig::kServerHost,
                      AppConfig::kServerPort,
                      AppConfig::kRequestedAgvID);
}

void loop()
{
    const uint32_t nowMs = millis();

    // With the current lock this is also a continuous runtime invariant.
    if (!AppConfig::kEnableMotorOutputs)
        motionController.stopImmediately();

    // If the socket is already known down, stop before reconnect processing.
    if (routeExecutor.hasRoute() && !sessionReady())
        routeExecutor.onNetworkLost();

    if (bootButtonPressed(nowMs))
        handleBootPress(nowMs);

    // Encoder, timeout, stall and immediate motion safety are serviced before
    // any potentially variable-duration TCP work.
    routeExecutor.update(nowMs, sessionReady());

    if (networkConfigured)
        robotClient.update();

    // RobotClient also invokes onDisconnected before socket cleanup. This
    // second check covers a state change discovered during update().
    if (routeExecutor.hasRoute() && !sessionReady())
        routeExecutor.onNetworkLost();

    printStateTransition(routeExecutor.state());

    if (networkConfigured && sessionReady())
    {
        reportFaultIfNeeded();
        sendArrivalIfReady();
        sendStatusIfDue(millis());
    }

    if (!AppConfig::kEnableMotorOutputs)
        motionController.stopImmediately();

    delay(2);
}
