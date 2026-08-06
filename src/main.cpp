#include <Arduino.h>
#include <cstring>

#include "Config.hpp"
#include "RobotClient.hpp"

static_assert(!AppConfig::kEnableMotorOutputs,
              "PHASE 2A SAFETY FAILURE: motor outputs must remain disabled");

namespace
{
    enum class DemoState : uint8_t
    {
        DISARMED_NO_ROUTE,
        WAIT_BOOT,
        COUNTDOWN,
        DRY_RUN_APPROVED,
        FAULT_LATCHED,
        ESTOP_LATCHED
    };

    RobotClient robotClient;
    RobotProtocol::RouteCommandPayload pendingRoute;

    DemoState demoState = DemoState::DISARMED_NO_ROUTE;
    bool networkConfigured = false;
    bool routePending = false;
    bool previousTcpConnected = false;
    uint32_t countdownStartedMs = 0;
    uint32_t lastStatusMs = 0;
    uint32_t lastStatusLogMs = 0;

    bool lastButtonReading = HIGH;
    bool stableButtonState = HIGH;
    bool buttonArmed = true;
    uint32_t lastButtonChangeMs = 0;

    const char* stateName(DemoState state)
    {
        switch (state)
        {
        case DemoState::DISARMED_NO_ROUTE: return "DISARMED_NO_ROUTE";
        case DemoState::WAIT_BOOT:         return "WAIT_BOOT";
        case DemoState::COUNTDOWN:         return "COUNTDOWN";
        case DemoState::DRY_RUN_APPROVED:  return "DRY_RUN_APPROVED";
        case DemoState::FAULT_LATCHED:     return "FAULT_LATCHED";
        case DemoState::ESTOP_LATCHED:     return "ESTOP_LATCHED";
        default:                           return "UNKNOWN";
        }
    }

    void forceMotorSafe()
    {
        ledcWrite(AppConfig::kLeftPwmChannel, 0);
        ledcWrite(AppConfig::kRightPwmChannel, 0);

        digitalWrite(AppConfig::kLeftMotorIn1Pin, LOW);
        digitalWrite(AppConfig::kLeftMotorIn2Pin, LOW);
        digitalWrite(AppConfig::kRightMotorIn1Pin, LOW);
        digitalWrite(AppConfig::kRightMotorIn2Pin, LOW);
        digitalWrite(AppConfig::kMotorStandbyPin, LOW);
    }

    void initializeSafetyHardware()
    {
        // STBY is forced LOW before Serial, Wi-Fi, or any route handling starts.
        pinMode(AppConfig::kMotorStandbyPin, OUTPUT);
        digitalWrite(AppConfig::kMotorStandbyPin, LOW);

        pinMode(AppConfig::kLeftMotorIn1Pin, OUTPUT);
        pinMode(AppConfig::kLeftMotorIn2Pin, OUTPUT);
        pinMode(AppConfig::kRightMotorIn1Pin, OUTPUT);
        pinMode(AppConfig::kRightMotorIn2Pin, OUTPUT);

        ledcSetup(AppConfig::kLeftPwmChannel,
                  AppConfig::kPwmFrequency,
                  AppConfig::kPwmResolutionBits);
        ledcSetup(AppConfig::kRightPwmChannel,
                  AppConfig::kPwmFrequency,
                  AppConfig::kPwmResolutionBits);
        ledcAttachPin(AppConfig::kLeftMotorPwmPin, AppConfig::kLeftPwmChannel);
        ledcAttachPin(AppConfig::kRightMotorPwmPin, AppConfig::kRightPwmChannel);

        pinMode(AppConfig::kBootButtonPin, INPUT_PULLUP);
        const bool initialButtonState = digitalRead(AppConfig::kBootButtonPin);
        lastButtonReading = initialButtonState;
        stableButtonState = initialButtonState;
        buttonArmed = initialButtonState == HIGH;
        lastButtonChangeMs = millis();

        forceMotorSafe();
    }

    bool credentialsAreConfigured()
    {
        return std::strcmp(AppConfig::kWifiSsid, "CHANGE_ME_WIFI_SSID") != 0
            && std::strcmp(AppConfig::kWifiPassword, "CHANGE_ME_WIFI_PASSWORD") != 0;
    }

    bool isExactDemoRoute(const RobotProtocol::RouteCommandPayload& route)
    {
        return route.routeID != 0
            && route.nodeCount == 2
            && route.nodes[0].nodeID == AppConfig::kDemoStartNodeID
            && route.nodes[1].nodeID == AppConfig::kDemoTargetNodeID;
    }

    bool sameNodeSequence(const RobotProtocol::RouteCommandPayload& left,
                          const RobotProtocol::RouteCommandPayload& right)
    {
        if (left.nodeCount != right.nodeCount)
            return false;

        for (uint16_t i = 0; i < left.nodeCount; ++i)
        {
            if (left.nodes[i].nodeID != right.nodes[i].nodeID)
                return false;
        }
        return true;
    }

    void printRoute(const RobotProtocol::RouteCommandPayload& route)
    {
        Serial.printf("[ROUTE] routeID=%lu nodeCount=%u\n",
                      static_cast<unsigned long>(route.routeID), route.nodeCount);
        for (uint16_t i = 0; i < route.nodeCount; ++i)
        {
            Serial.printf("  node[%u]=%lu arrival=%.3f departure=%.3f\n",
                          i,
                          static_cast<unsigned long>(route.nodes[i].nodeID),
                          route.nodes[i].arrivalTime,
                          route.nodes[i].departureTime);
        }
    }

    bool bootButtonPressed()
    {
        const bool reading = digitalRead(AppConfig::kBootButtonPin);

        if (reading != lastButtonReading)
        {
            lastButtonReading = reading;
            lastButtonChangeMs = millis();
        }

        if (millis() - lastButtonChangeMs < AppConfig::kButtonDebounceMs)
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

    void clearPendingRoute(const char* reason)
    {
        forceMotorSafe();
        routePending = false;
        std::memset(&pendingRoute, 0, sizeof(pendingRoute));
        if (demoState != DemoState::FAULT_LATCHED
            && demoState != DemoState::ESTOP_LATCHED
            && demoState != DemoState::DRY_RUN_APPROVED)
        {
            demoState = DemoState::DISARMED_NO_ROUTE;
        }
        Serial.printf("[SAFE] %s | STBY=LOW PWM=0\n", reason);
    }

    void handleBootApproval()
    {
        if (demoState == DemoState::WAIT_BOOT)
        {
            if (!routePending || !robotClient.connected() || !robotClient.accepted())
            {
                clearPendingRoute("BOOT REJECTED: ROUTE OR SERVER NOT READY");
                return;
            }

            countdownStartedMs = millis();
            demoState = DemoState::COUNTDOWN;
            Serial.println();
            Serial.println("================================");
            Serial.println("BOOT APPROVAL RECEIVED");
            Serial.println("5 SECOND SAFETY COUNTDOWN");
            Serial.println("DRY RUN: MOTORS WILL NOT MOVE");
            Serial.println("PRESS BOOT AGAIN TO CANCEL");
            Serial.println("================================");
            return;
        }

        if (demoState == DemoState::COUNTDOWN)
        {
            clearPendingRoute("USER CANCELLED DURING COUNTDOWN");
            return;
        }

        Serial.printf("[BOOT] Ignored in state=%s\n", stateName(demoState));
    }

    void configureCallbacks()
    {
        robotClient.onAccepted = [](uint32_t agvID)
        {
            Serial.printf("[SAFE] SERVER ACCEPTED AGV %lu\n",
                          static_cast<unsigned long>(agvID));
            Serial.println("[SAFE] MOTOR OUTPUTS REMAIN COMPILE-LOCKED OFF");
        };

        robotClient.onRouteCommand = [](const RobotProtocol::RouteCommandPayload& route)
        {
            forceMotorSafe();
            printRoute(route);

            if (!isExactDemoRoute(route))
            {
                Serial.println("[SAFE] ROUTE REJECTED: ONLY EXACT [1 -> 2] IS ALLOWED");
                Serial.println("[SAFE] STBY=LOW PWM=0");
                return;
            }

            if (demoState == DemoState::FAULT_LATCHED
                || demoState == DemoState::ESTOP_LATCHED
                || demoState == DemoState::DRY_RUN_APPROVED)
            {
                Serial.printf("[SAFE] ROUTE REJECTED: STATE IS LATCHED (%s)\n",
                              stateName(demoState));
                return;
            }

            if (routePending)
            {
                if (sameNodeSequence(pendingRoute, route))
                    Serial.println("[SAFE] DUPLICATE [1 -> 2] IGNORED; NO RESTART");
                else
                    Serial.println("[SAFE] NEW ROUTE REJECTED WHILE ONE IS PENDING");
                return;
            }

            if (!robotClient.accepted())
            {
                Serial.println("[SAFE] ROUTE REJECTED: SERVER SESSION NOT ACCEPTED");
                return;
            }

            pendingRoute = route;
            routePending = true;
            demoState = DemoState::WAIT_BOOT;
            Serial.println("[SAFE] EXACT [1 -> 2] ROUTE STORED");
            Serial.println("[SAFE] WAITING FOR LOCAL BOOT APPROVAL");
            Serial.println("[SAFE] STBY=LOW PWM=0");
        };

        robotClient.onCancelRoute = []()
        {
            clearPendingRoute("SERVER CANCEL_ROUTE RECEIVED");
        };

        robotClient.onEmergencyStop = []()
        {
            forceMotorSafe();
            routePending = false;
            demoState = DemoState::ESTOP_LATCHED;
            Serial.println("[SAFE] EMERGENCY STOP LATCHED");
            Serial.println("[SAFE] REBOOT REQUIRED; STBY=LOW PWM=0");
        };
    }

    void updateDryRunState(uint32_t nowMs)
    {
        if (demoState != DemoState::COUNTDOWN)
            return;

        if (!routePending || !robotClient.connected() || !robotClient.accepted())
        {
            clearPendingRoute("COUNTDOWN ABORTED: TCP OR ROUTE LOST");
            return;
        }

        if (nowMs - countdownStartedMs < AppConfig::kApprovalCountdownMs)
            return;

        forceMotorSafe();
        demoState = DemoState::DRY_RUN_APPROVED;
        Serial.println();
        Serial.println("================================");
        Serial.println("PHASE 2A DRY RUN PASSED");
        Serial.println("BOOT APPROVAL RECEIVED");
        Serial.println("DRY RUN ONLY - NO MOTION");
        Serial.println("STBY=LOW PWM=0");
        Serial.println("ARRIVED BLOCKED");
        Serial.println("POWER CYCLE REQUIRED FOR ANOTHER RUN");
        Serial.println("================================");
    }

    void sendSafeStatus(uint32_t nowMs)
    {
        if (!robotClient.accepted()
            || nowMs - lastStatusMs < AppConfig::kStatusIntervalMs)
        {
            return;
        }

        lastStatusMs = nowMs;
        RobotProtocol::StatusPayload status;
        status.currentNodeID = AppConfig::kDemoStartNodeID;
        status.currentLinkID = 0;
        status.progress = 0.0f;
        status.x = 0.0f;
        status.z = 0.0f;
        status.heading = 0.0f;
        status.velocity = 0.0f;
        status.battery = 100.0f; // Placeholder; battery is physically removed.
        status.state = demoState == DemoState::ESTOP_LATCHED
            ? RobotProtocol::RobotState::EMERGENCY_STOPPED
            : (demoState == DemoState::FAULT_LATCHED
                ? RobotProtocol::RobotState::FAULT
                : RobotProtocol::RobotState::IDLE);

        if (!robotClient.sendStatus(status))
            Serial.println("[STATUS] Send failed");

        if (nowMs - lastStatusLogMs >= AppConfig::kStatusLogIntervalMs)
        {
            lastStatusLogMs = nowMs;
            Serial.printf("[STATUS] node=1 IDLE | state=%s | pending=%u | STBY=LOW PWM=0\n",
                          stateName(demoState), routePending ? 1U : 0U);
        }
    }
}

void setup()
{
    initializeSafetyHardware();

    Serial.begin(AppConfig::kSerialBaud);
    delay(300);

    Serial.println();
    Serial.println("================================");
    Serial.println("PHASE 2A: SERVER ROUTE + BOOT DRY RUN");
    Serial.println("ACCEPTED ROUTE: EXACT [1 -> 2] ONLY");
    Serial.println("MOTOR OUTPUTS: COMPILE-LOCKED OFF");
    Serial.println("TB6612 STBY: LOW");
    Serial.println("ARRIVED TRANSMISSION: BLOCKED");
    Serial.println("BATTERY AND VM MUST STAY DISCONNECTED");
    Serial.println("================================");

    configureCallbacks();
    networkConfigured = credentialsAreConfigured();
    if (!networkConfigured)
    {
        Serial.println("[CONFIG] Fill include/Secrets.hpp, then upload again");
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
    // Phase 2A invariant: every pass through loop drives all motor outputs safe.
    forceMotorSafe();

    if (bootButtonPressed())
        handleBootApproval();

    if (networkConfigured)
    {
        robotClient.update();
        forceMotorSafe();

        const bool tcpConnected = robotClient.connected();
        if (previousTcpConnected && !tcpConnected)
        {
            if (demoState != DemoState::DRY_RUN_APPROVED
                && demoState != DemoState::FAULT_LATCHED
                && demoState != DemoState::ESTOP_LATCHED)
            {
                clearPendingRoute("TCP LOST: ROUTE CLEARED");
            }
        }
        previousTcpConnected = tcpConnected;

        const uint32_t nowMs = millis();
        updateDryRunState(nowMs);
        sendSafeStatus(nowMs);
    }

    forceMotorSafe();
    delay(2);
}
