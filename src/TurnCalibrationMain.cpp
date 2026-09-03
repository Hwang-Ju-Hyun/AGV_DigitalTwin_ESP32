#include "Config.hpp"

#if AGV_TURN_CALIBRATION_ENABLED

#if AGV_TRAJECTORY_PREVIEW_ENABLED || AGV_TRAJECTORY_TRACE_ENABLED
#error "Turn calibration cannot be combined with trajectory preview/trace"
#endif

#include <Arduino.h>
#include <cstdint>

#include "MotionController.hpp"
#include "PhysicalFleetAuthorization.hpp"

static_assert(AppConfig::kTurnCalibrationEnabled,
              "TURN PROFILE FAILURE: mode must be enabled");
static_assert(!AppConfig::kStraightCalibrationEnabled,
              "TURN SAFETY FAILURE: straight calibration must be disabled");
static_assert(!AppConfig::kPhysicalFleetEnabled,
              "TURN SAFETY FAILURE: fleet mode must be disabled");
static_assert(!AppConfig::kRaisedWheelBuild,
              "TURN SAFETY FAILURE: legacy raised mode must be disabled");
static_assert(AppConfig::kEnableMotorOutputs
                  == AppConfig::kTurnCalibrationMotorBuild,
              "TURN SAFETY FAILURE: motor output/profile mismatch");
static_assert(AppConfig::kTurnCalibrationDirection == 1
                  || AppConfig::kTurnCalibrationDirection == 2,
              "TURN PROFILE FAILURE: direction must be CW=1 or CCW=2");
static_assert(AppConfig::kTurn90CwCount == 163,
              "TURN CALIBRATION FAILURE: CW target changed");
static_assert(AppConfig::kTurn90CcwCount == 159,
              "TURN CALIBRATION FAILURE: CCW target changed");

namespace
{
    constexpr uint16_t kMaximumSamples = 180;
    constexpr uint8_t kDumpLinesPerLoop = 6;
    constexpr bool kClockwise = AppConfig::kTurnCalibrationDirection == 1;
    constexpr MotionController::Mode kTurnMode =
        kClockwise ? MotionController::Mode::TURN_CW
                   : MotionController::Mode::TURN_CCW;
    constexpr int32_t kTurnTargetCount = kClockwise
        ? AppConfig::kTurn90CwCount : AppConfig::kTurn90CcwCount;

    enum class RunState : uint8_t
    {
        WAIT_BOOT,
        COUNTDOWN,
        RUNNING,
        SETTLING,
        COMPLETE,
        OUTPUT_LOCKED,
        FAULT_LATCHED,
        ESTOP_LATCHED
    };

    struct TurnSample
    {
        uint32_t elapsedMs = 0;
        uint32_t intervalMs = 0;
        int32_t rawLeft = 0;
        int32_t rawRight = 0;
        int32_t leftProgress = 0;
        int32_t rightProgress = 0;
        int16_t leftPwm = 0;
        int16_t rightPwm = 0;
        bool settling = false;
    };

    MotionController motionController;
    PhysicalFleetAuthorization authorization;
    TurnSample samples[kMaximumSamples];
    uint16_t sampleCount = 0;
    uint16_t dumpIndex = 0;
    bool sampleOverflow = false;
    bool startAttempted = false;
    bool terminalReported = false;
    uint32_t runStartedMs = 0;
    uint32_t lastSampleMs = 0;
    uint32_t lastCountdownSecond = UINT32_MAX;
    RunState runState = RunState::WAIT_BOOT;

    bool lastButtonReading = HIGH;
    bool stableButtonState = HIGH;
    bool buttonArmed = true;
    uint32_t lastButtonChangeMs = 0;

    const char* directionName()
    {
        return kClockwise ? "CW" : "CCW";
    }

    const char* stateName(RunState state)
    {
        switch (state)
        {
        case RunState::WAIT_BOOT:       return "WAIT_BOOT";
        case RunState::COUNTDOWN:       return "COUNTDOWN";
        case RunState::RUNNING:         return "RUNNING";
        case RunState::SETTLING:        return "SETTLING";
        case RunState::COMPLETE:        return "COMPLETE";
        case RunState::OUTPUT_LOCKED:   return "OUTPUT_LOCKED";
        case RunState::FAULT_LATCHED:   return "FAULT_LATCHED";
        case RunState::ESTOP_LATCHED:   return "ESTOP_LATCHED";
        default:                        return "UNKNOWN";
        }
    }

    bool terminalState()
    {
        return runState == RunState::COMPLETE
            || runState == RunState::OUTPUT_LOCKED
            || runState == RunState::FAULT_LATCHED
            || runState == RunState::ESTOP_LATCHED;
    }

    void initializeBootButton()
    {
        pinMode(AppConfig::kBootButtonPin, INPUT_PULLUP);
        const bool initialState = digitalRead(AppConfig::kBootButtonPin);
        lastButtonReading = initialState;
        stableButtonState = initialState;
        // Holding BOOT during reset is never approval.
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
        if (nowMs - lastButtonChangeMs < AppConfig::kButtonDebounceMs
            || reading == stableButtonState)
        {
            return false;
        }

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

    void recordSample(uint32_t nowMs, bool force)
    {
        const uint32_t intervalMs = nowMs - lastSampleMs;
        if (!force && intervalMs < AppConfig::kCalibrationSampleIntervalMs)
            return;
        if (force && intervalMs == 0 && sampleCount > 0)
            return;
        if (sampleCount >= kMaximumSamples)
        {
            sampleOverflow = true;
            return;
        }

        const MotionController::Snapshot snapshot = motionController.snapshot();
        TurnSample& sample = samples[sampleCount++];
        sample.elapsedMs = nowMs - runStartedMs;
        sample.intervalMs = intervalMs;
        sample.rawLeft = snapshot.rawLeftCount;
        sample.rawRight = snapshot.rawRightCount;
        sample.leftProgress = snapshot.leftProgress;
        sample.rightProgress = snapshot.rightProgress;
        sample.leftPwm = static_cast<int16_t>(snapshot.leftPwm);
        sample.rightPwm = static_cast<int16_t>(snapshot.rightPwm);
        sample.settling = snapshot.settling;
        lastSampleMs = nowMs;
    }

    void beginRun(uint32_t nowMs)
    {
        if (startAttempted)
            return;
        startAttempted = true;
        sampleCount = 0;
        dumpIndex = 0;
        sampleOverflow = false;
        runStartedMs = nowMs;
        lastSampleMs = nowMs;

        const MotionController::StartResult result =
            motionController.startMotion(
                kTurnMode, kTurnTargetCount, nowMs);
        if (result == MotionController::StartResult::STARTED)
        {
            runState = RunState::RUNNING;
            return;
        }

        motionController.stopImmediately();
        if (result == MotionController::StartResult::OUTPUT_DISABLED)
        {
            runState = RunState::OUTPUT_LOCKED;
            Serial.println("[LOCKED] Turn safely blocked; PWM=0 STBY=LOW");
            return;
        }

        motionController.emergencyStop(
            MotionController::Fault::OUTPUT_INVARIANT);
        runState = RunState::FAULT_LATCHED;
        Serial.printf("[FAULT] Turn start result=%u\n",
                      static_cast<unsigned>(result));
    }

    void handleBoot(uint32_t nowMs)
    {
        const auto result = authorization.onBootPress(nowMs);
        Serial.printf("[BOOT] %s\n",
                      PhysicalFleetAuthorization::bootResultName(result));
        if (result == PhysicalFleetAuthorization::BootResult::COUNTDOWN_STARTED)
        {
            runState = RunState::COUNTDOWN;
            lastCountdownSecond = UINT32_MAX;
        }
        else if (result
                 == PhysicalFleetAuthorization::BootResult::COUNTDOWN_CANCELLED)
        {
            motionController.stopImmediately();
            runState = RunState::WAIT_BOOT;
            Serial.println("[SAFE] Turn approval cancelled");
        }
        else if (result
                 == PhysicalFleetAuthorization::BootResult::ESTOP_LATCHED)
        {
            motionController.emergencyStop(
                MotionController::Fault::EXTERNAL_STOP);
            runState = RunState::ESTOP_LATCHED;
            recordSample(nowMs, true);
            Serial.println("[SAFE] BOOT E-stop latched; reboot required");
        }
    }

    void updateMotion(uint32_t nowMs)
    {
        if (runState != RunState::RUNNING
            && runState != RunState::SETTLING)
        {
            return;
        }

        const MotionController::UpdateResult result =
            motionController.update(nowMs);
        recordSample(nowMs, false);
        if (result == MotionController::UpdateResult::RUNNING)
        {
            runState = RunState::RUNNING;
            return;
        }
        if (result == MotionController::UpdateResult::SETTLING)
        {
            runState = RunState::SETTLING;
            return;
        }

        recordSample(nowMs, true);
        if (result == MotionController::UpdateResult::COMPLETE
            && motionController.outputsSafe())
        {
            runState = RunState::COMPLETE;
            return;
        }

        motionController.emergencyStop(
            motionController.faultLatched()
                ? motionController.fault()
                : MotionController::Fault::OUTPUT_INVARIANT);
        runState = RunState::FAULT_LATCHED;
    }

    void reportTerminalOnce()
    {
        if (!terminalState() || terminalReported)
            return;
        terminalReported = true;
        motionController.stopImmediately();
        const MotionController::Snapshot snapshot = motionController.snapshot();
        Serial.printf(
            "[TURN_SUMMARY] direction=%s state=%s elapsed=%lu "
            "rawL=%ld rawR=%ld L=%ld R=%ld diff=%ld samples=%u "
            "overflow=%u fault=%u\n",
            directionName(),
            stateName(runState),
            static_cast<unsigned long>(snapshot.elapsedMs),
            static_cast<long>(snapshot.rawLeftCount),
            static_cast<long>(snapshot.rawRightCount),
            static_cast<long>(snapshot.leftProgress),
            static_cast<long>(snapshot.rightProgress),
            static_cast<long>(snapshot.leftProgress
                              - snapshot.rightProgress),
            sampleCount,
            sampleOverflow ? 1U : 0U,
            static_cast<unsigned>(snapshot.fault));
        Serial.printf("[SAFE] outputsSafe=%u PWM=%d/%d STBY=%s\n",
                      snapshot.outputsSafe ? 1U : 0U,
                      snapshot.leftPwm,
                      snapshot.rightPwm,
                      digitalRead(AppConfig::kMotorStandbyPin) == LOW
                          ? "LOW" : "HIGH");
        if (sampleCount > 0)
        {
            Serial.println(
                "[TURN_CSV] t_ms,dt_ms,rawL,rawR,L,R,diff,pwmL,pwmR,phase");
        }
    }

    void dumpSamplesStep()
    {
        if (!terminalReported)
            return;
        for (uint8_t lines = 0;
             lines < kDumpLinesPerLoop && dumpIndex < sampleCount;
             ++lines, ++dumpIndex)
        {
            const TurnSample& sample = samples[dumpIndex];
            Serial.printf(
                "[TURN_DATA] %lu,%lu,%ld,%ld,%ld,%ld,%ld,%d,%d,%c\n",
                static_cast<unsigned long>(sample.elapsedMs),
                static_cast<unsigned long>(sample.intervalMs),
                static_cast<long>(sample.rawLeft),
                static_cast<long>(sample.rawRight),
                static_cast<long>(sample.leftProgress),
                static_cast<long>(sample.rightProgress),
                static_cast<long>(sample.leftProgress
                                  - sample.rightProgress),
                sample.leftPwm,
                sample.rightPwm,
                sample.settling ? 'S' : 'R');
        }
        if (dumpIndex == sampleCount && sampleCount > 0)
        {
            Serial.println("[TURN] DATA DUMP COMPLETE; reboot required");
            ++dumpIndex;
        }
    }
}

void setup()
{
    // Establish the safe bridge state before Serial or user interaction.
    motionController.begin();
    motionController.stopImmediately();
    authorization.begin();
    initializeBootButton();

    Serial.begin(AppConfig::kSerialBaud);
    delay(300);
    Serial.println();
    Serial.println("================================");
    Serial.printf("TURN CALIBRATION: ONE 90 DEG %s RUN\n",
                  directionName());
#if AGV_TURN_CALIBRATION_MOTOR_ENABLED
    if (kClockwise)
        Serial.println("BUILD PROFILE: esp32dev-turn-calibration-cw");
    else
        Serial.println("BUILD PROFILE: esp32dev-turn-calibration-ccw");
    Serial.println("MOTOR OUTPUTS: ENABLED AFTER LOCAL APPROVAL");
    Serial.println("FIRST RUN MUST KEEP BOTH WHEELS OFF THE FLOOR");
#else
    Serial.println("BUILD PROFILE: esp32dev-turn-calibration-locked");
    Serial.println("MOTOR OUTPUTS: COMPILE-LOCKED OFF");
#endif
    Serial.println("NETWORK: DISABLED");
    Serial.println("TARGET: 176 NORMALIZED COUNTS PER WHEEL");
    Serial.println("BOOT: START/CANCEL; AFTER ARMING: LATCHED E-STOP");
    Serial.println("================================");
    Serial.println("[SAFE] Press BOOT once to begin the 5 second countdown");
}

void loop()
{
    const uint32_t nowMs = millis();

    if (!AppConfig::kEnableMotorOutputs || terminalState())
        motionController.stopImmediately();

    // BOOT has priority over starting or advancing motion.
    if (bootButtonPressed(nowMs))
        handleBoot(nowMs);

    // Approval is cancelled if BOOT is still held at the deadline, ensuring
    // the same input is released and available as an E-stop during the run.
    if (!terminalState()
        && authorization.state()
               == PhysicalFleetAuthorization::State::COUNTDOWN
        && authorization.countdownRemainingMs(
               nowMs, AppConfig::kApprovalCountdownMs) == 0
        && stableButtonState != HIGH)
    {
        authorization.onBootPress(nowMs);
        motionController.stopImmediately();
        runState = RunState::WAIT_BOOT;
        lastCountdownSecond = UINT32_MAX;
        Serial.println("[SAFE] BOOT must be released; approval cancelled");
    }

    if (!terminalState()
        && authorization.update(nowMs, AppConfig::kApprovalCountdownMs))
    {
        Serial.printf("[ARMED] One-shot 90 degree %s turn approved\n",
                      directionName());
        beginRun(nowMs);
    }
    else if (authorization.state()
             == PhysicalFleetAuthorization::State::COUNTDOWN)
    {
        const uint32_t seconds =
            (authorization.countdownRemainingMs(
                 nowMs, AppConfig::kApprovalCountdownMs) + 999U) / 1000U;
        if (seconds != lastCountdownSecond)
        {
            lastCountdownSecond = seconds;
            Serial.printf("[SAFE] STARTING IN %lu\n",
                          static_cast<unsigned long>(seconds));
        }
    }

    updateMotion(nowMs);
    reportTerminalOnce();
    dumpSamplesStep();

    if (!AppConfig::kEnableMotorOutputs || terminalState())
        motionController.stopImmediately();
    delay(2);
}

#endif // AGV_TURN_CALIBRATION_ENABLED
