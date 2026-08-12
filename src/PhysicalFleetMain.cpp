#include "Config.hpp"

#if AGV_PHYSICAL_FLEET_ENABLED

#include <Arduino.h>
#include <cstring>

#include "MotionController.hpp"
#include "PhysicalFleetAuthorization.hpp"
#include "PhysicalFleetExecutor.hpp"
#include "RobotClient.hpp"
#include "RobotProtocol.hpp"

static_assert(AppConfig::kPhysicalFleetEnabled,
              "FLEET PROFILE FAILURE: executor must be enabled");
static_assert(!AppConfig::kRaisedWheelBuild,
              "FLEET PROFILE FAILURE: demo profile must be disabled");
static_assert(AppConfig::kEnableMotorOutputs
                  == AppConfig::kPhysicalFleetMotorBuild,
              "FLEET SAFETY FAILURE: motor output/profile mismatch");
#if AGV_PHYSICAL_FLEET_MOTOR_ENABLED
static_assert(AppConfig::kEnableMotorOutputs,
              "LIVE FLEET SAFETY FAILURE: motor outputs must be explicit");
#else
static_assert(!AppConfig::kEnableMotorOutputs,
              "LOCKED FLEET SAFETY FAILURE: outputs must remain disabled");
#endif

namespace
{
    MotionController motionController;
    PhysicalFleetExecutor fleetExecutor(motionController);
    PhysicalFleetAuthorization authorization;
    RobotClient robotClient;

    bool credentialsConfigured = false;
    bool networkStarted = false;
    bool faultReported = false;
    uint32_t lastStatusMs = 0;
    uint32_t lastStatusLogMs = 0;
    uint32_t lastCountdownLogSecond = UINT32_MAX;
    PhysicalFleetExecutor::State lastExecutorState =
        PhysicalFleetExecutor::State::IDLE;

    bool lastButtonReading = HIGH;
    bool stableButtonState = HIGH;
    bool buttonArmed = true;
    uint32_t lastButtonChangeMs = 0;

    bool sessionReady()
    {
        return networkStarted
            && robotClient.connected()
            && robotClient.accepted();
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

    void startNetworkAfterApproval()
    {
        if (networkStarted || !credentialsConfigured
            || !authorization.networkAllowed())
        {
            return;
        }

        const uint32_t capability = AppConfig::kPhysicalFleetMotorBuild
            ? RobotProtocol::CAPABILITY_TRAJECTORY_COMMAND
            : RobotProtocol::CAPABILITY_NONE;
        robotClient.begin(AppConfig::kWifiSsid,
                          AppConfig::kWifiPassword,
                          AppConfig::kServerHost,
                          AppConfig::kServerPort,
                          AppConfig::kRequestedAgvID,
                          capability);
        networkStarted = true;
        if (capability == RobotProtocol::CAPABILITY_TRAJECTORY_COMMAND)
            Serial.println("[ARMED] TCP enabled; COMMAND HELLO will start automatic dispatch");
        else
            Serial.println("[LOCKED] TCP enabled without COMMAND capability");
    }

    void handleBoot(uint32_t nowMs)
    {
        const PhysicalFleetAuthorization::BootResult result =
            authorization.onBootPress(nowMs);
        Serial.printf("[BOOT] %s\n",
                      PhysicalFleetAuthorization::bootResultName(result));

        if (result == PhysicalFleetAuthorization::BootResult::ESTOP_LATCHED)
        {
            // Motor power is removed before any later TCP or log work.
            fleetExecutor.emergencyStop();
            Serial.println("[SAFE] Local BOOT E-stop latched; PWM=0 STBY=LOW");
        }
        else if (result
                 == PhysicalFleetAuthorization::BootResult::COUNTDOWN_STARTED)
        {
            Serial.println("[SAFE] 5 second pre-network countdown started");
        }
        else if (result
                 == PhysicalFleetAuthorization::BootResult::COUNTDOWN_CANCELLED)
        {
            motionController.stopImmediately();
            Serial.println("[SAFE] Approval cancelled; TCP remains disabled");
        }
    }

    void configureCallbacks()
    {
        robotClient.onAccepted = [](uint32_t agvID)
        {
            Serial.printf("[FLEET] SERVER ACCEPTED AGV %lu\n",
                          static_cast<unsigned long>(agvID));
        };

        robotClient.onDisconnected = []()
        {
            // RobotClient calls this before socket stop/cleanup/logging.
            fleetExecutor.onNetworkLost();
        };

        robotClient.onRouteCommand = [](
            const RobotProtocol::RouteCommandPayload&)
        {
            Serial.println("[SAFE] Legacy ROUTE_COMMAND ignored in physical-fleet mode");
        };

        robotClient.onTrajectoryCommand = [](
            const RobotProtocol::TrajectoryCommandPayload& trajectory)
        {
            const auto result = fleetExecutor.acceptTrajectory(
                trajectory, sessionReady(), millis());
            Serial.printf(
                "[FLEET] trajectory routeID=%lu start=%lu final=%lu "
                "waypoints=%u scale=%.2f result=%s\n",
                static_cast<unsigned long>(trajectory.routeID),
                static_cast<unsigned long>(trajectory.startNodeID),
                static_cast<unsigned long>(trajectory.finalNodeID),
                trajectory.waypointCount,
                trajectory.millimetersPerMapUnit,
                PhysicalFleetExecutor::acceptResultName(result));
        };

        robotClient.onCancelRoute = []()
        {
            fleetExecutor.cancel();
            Serial.println("[SAFE] CANCEL_ROUTE: PWM=0 STBY=LOW");
        };

        robotClient.onEmergencyStop = []()
        {
            fleetExecutor.emergencyStop();
            Serial.println("[SAFE] Server EMERGENCY_STOP latched");
        };
    }

    void sendArrivalIfReady(uint32_t nowMs)
    {
        uint32_t nodeID = 0;
        if (!sessionReady() || !fleetExecutor.arrivalPending(nodeID))
            return;

        if (!robotClient.sendStatus(fleetExecutor.buildStatus()))
        {
            fleetExecutor.markArrivedSendResult(false, nowMs);
            return;
        }
        const bool sent = robotClient.sendArrived(nodeID);
        fleetExecutor.markArrivedSendResult(sent, nowMs);
        if (sent)
            Serial.printf("[RobotProtocol] ARRIVED node=%lu\n",
                          static_cast<unsigned long>(nodeID));
    }

    void sendStatusIfDue(uint32_t nowMs)
    {
        if (!sessionReady()
            || nowMs - lastStatusMs < AppConfig::kStatusIntervalMs)
            return;

        lastStatusMs = nowMs;
        const RobotProtocol::StatusPayload status = fleetExecutor.buildStatus();
        if (!robotClient.sendStatus(status)
            || nowMs - lastStatusLogMs < AppConfig::kStatusLogIntervalMs)
            return;

        lastStatusLogMs = nowMs;
        const MotionController::Snapshot motion = motionController.snapshot();
        Serial.printf(
            "[STATUS] node=%lu target=%lu progress=%.3f state=%s "
            "L=%ld R=%ld PWM=%d/%d STBY=%s\n",
            static_cast<unsigned long>(status.currentNodeID),
            static_cast<unsigned long>(status.currentLinkID),
            status.progress,
            PhysicalFleetExecutor::stateName(fleetExecutor.state()),
            static_cast<long>(motion.leftProgress),
            static_cast<long>(motion.rightProgress),
            motion.leftPwm,
            motion.rightPwm,
            digitalRead(AppConfig::kMotorStandbyPin) == LOW ? "LOW" : "HIGH");
    }

    void reportFaultIfNeeded()
    {
        if (fleetExecutor.state() !=
            PhysicalFleetExecutor::State::FAULT_LATCHED)
        {
            faultReported = false;
            return;
        }
        if (!faultReported && sessionReady())
        {
            faultReported = robotClient.sendError(
                RobotProtocol::ErrorCode::MOTOR_FAULT,
                fleetExecutor.faultDetail());
        }
    }

    void logStateTransition()
    {
        const auto state = fleetExecutor.state();
        if (state == lastExecutorState)
            return;
        lastExecutorState = state;
        const MotionController::Snapshot motion = motionController.snapshot();
        Serial.printf("[EXECUTOR] state=%s wp=%u node=%lu mode=%u safe=%u\n",
                      PhysicalFleetExecutor::stateName(state),
                      fleetExecutor.waypointIndex(),
                      static_cast<unsigned long>(fleetExecutor.currentNodeID()),
                      static_cast<unsigned>(motion.mode),
                      motion.outputsSafe ? 1U : 0U);
        if (state == PhysicalFleetExecutor::State::FAULT_LATCHED)
        {
            Serial.printf("[FAULT] %s detail=%lu; reboot required\n",
                          PhysicalFleetExecutor::faultName(fleetExecutor.fault()),
                          static_cast<unsigned long>(fleetExecutor.faultDetail()));
        }
    }
}

void setup()
{
    // Hardware is made safe before Serial, Wi-Fi, or TCP can start.
    fleetExecutor.begin(AppConfig::kPhysicalFleetStartNodeID,
                        AppConfig::kPhysicalFleetStartHeadingRad);
    authorization.begin();
    initializeBootButton();

    Serial.begin(AppConfig::kSerialBaud);
    delay(300);
    Serial.println();
    Serial.println("================================");
    Serial.println("PHASE 2F: TESTCASE0 PHYSICAL FLEET");
#if AGV_PHYSICAL_FLEET_MOTOR_ENABLED
    Serial.println("BUILD PROFILE: esp32dev-physical-fleet");
    Serial.println("MOTOR OUTPUTS: ENABLED");
    Serial.println("WARNING: AUTOMATIC FLOOR ROUTES CAN START AFTER ARMING");
#else
    Serial.println("BUILD PROFILE: esp32dev-physical-fleet-locked");
    Serial.println("MOTOR OUTPUTS: COMPILE-LOCKED OFF");
#endif
    Serial.println("MAP: TESTCASE0 LINE ONLY, 50 MM/UNIT");
    Serial.println("START: NODE 1, EAST (+X, 0 RAD)");
    Serial.println("PLACE ROBOT AT NODE 1 FACING EAST BEFORE BOOT");
    Serial.println("TCP: DISABLED UNTIL BOOT + 5 SECOND APPROVAL");
    Serial.println("BOOT AFTER ARMING: LOCAL E-STOP");
    Serial.println("================================");

    configureCallbacks();
    credentialsConfigured = credentialsAreConfigured();
    if (!credentialsConfigured)
        Serial.println("[CONFIG] Configure local Secrets.hpp, then upload again");
    else
        Serial.println("[SAFE] Press BOOT once to begin the 5 second approval");
}

void loop()
{
    const uint32_t nowMs = millis();

    if (!AppConfig::kEnableMotorOutputs)
        motionController.stopImmediately();

    // BOOT has first priority over motion and all network processing.
    if (bootButtonPressed(nowMs))
        handleBoot(nowMs);

    if (authorization.update(nowMs, AppConfig::kApprovalCountdownMs))
    {
        Serial.println("[ARMED] Operator confirmed physical pose: node 1, east");
        startNetworkAfterApproval();
    }
    else if (authorization.state()
             == PhysicalFleetAuthorization::State::COUNTDOWN)
    {
        const uint32_t seconds =
            (authorization.countdownRemainingMs(
                 nowMs, AppConfig::kApprovalCountdownMs) + 999U) / 1000U;
        if (seconds != lastCountdownLogSecond)
        {
            lastCountdownLogSecond = seconds;
            Serial.printf("[SAFE] ARMING IN %lu\n",
                          static_cast<unsigned long>(seconds));
        }
    }

    // Poll the socket before advancing motion so a newly detected FIN/RST or
    // Wi-Fi loss invokes the safe-stop callback before another motor update.
    if (networkStarted)
        robotClient.update();

    if (fleetExecutor.hasActiveCommand() && !sessionReady())
        fleetExecutor.onNetworkLost();

    fleetExecutor.update(nowMs, sessionReady());

    logStateTransition();
    if (sessionReady())
    {
        reportFaultIfNeeded();
        sendArrivalIfReady(millis());
        sendStatusIfDue(millis());
    }

    if (!AppConfig::kEnableMotorOutputs)
        motionController.stopImmediately();
    delay(2);
}

#endif // AGV_PHYSICAL_FLEET_ENABLED
