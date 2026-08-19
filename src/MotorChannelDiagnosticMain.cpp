#if defined(AGV_CHANNEL_DIAGNOSTIC_ENABLED) \
    && AGV_CHANNEL_DIAGNOSTIC_ENABLED

#include <Arduino.h>
#include <cstdint>

#if !defined(AGV_CHANNEL_DIAGNOSTIC_MOTOR_ENABLED) \
    || !defined(AGV_CHANNEL_DIAGNOSTIC_CHANNEL) \
    || !defined(AGV_MOTOR_OUTPUTS_ENABLED)
#error "Select an explicit motor-channel diagnostic profile"
#endif

#if (AGV_CHANNEL_DIAGNOSTIC_MOTOR_ENABLED != 0 \
        && AGV_CHANNEL_DIAGNOSTIC_MOTOR_ENABLED != 1) \
    || (AGV_MOTOR_OUTPUTS_ENABLED != 0 \
        && AGV_MOTOR_OUTPUTS_ENABLED != 1)
#error "Diagnostic motor-safety flags must be 0 or 1"
#endif

#if AGV_CHANNEL_DIAGNOSTIC_CHANNEL != 1 \
    && AGV_CHANNEL_DIAGNOSTIC_CHANNEL != 2
#error "Diagnostic channel must be 1 (A/left) or 2 (B/right)"
#endif

#if AGV_MOTOR_OUTPUTS_ENABLED \
    != AGV_CHANNEL_DIAGNOSTIC_MOTOR_ENABLED
#error "Diagnostic motor-output flags do not match"
#endif

static_assert(AGV_CHANNEL_DIAGNOSTIC_MOTOR_ENABLED == 0
                  || AGV_CHANNEL_DIAGNOSTIC_MOTOR_ENABLED == 1,
              "DIAGNOSTIC SAFETY FAILURE: invalid motor flag");
static_assert(AGV_MOTOR_OUTPUTS_ENABLED
                  == AGV_CHANNEL_DIAGNOSTIC_MOTOR_ENABLED,
              "DIAGNOSTIC SAFETY FAILURE: output/profile mismatch");

namespace
{
    constexpr uint32_t kSerialBaud = 115200;
    constexpr uint32_t kApprovalCountdownMs = 5000;
    constexpr uint32_t kButtonDebounceMs = 50;
    constexpr uint32_t kPulseDurationMs = 300;
    constexpr uint32_t kCoastCaptureMs = 150;
    constexpr uint32_t kManualEncoderLogIntervalMs = 150;
    constexpr int32_t kMaximumDiagnosticCount = 100;
    constexpr int32_t kMovementThreshold = 3;

    constexpr int kBootButtonPin = 0;
    constexpr int kMotorStandbyPin = 13;
    constexpr int kLeftMotorIn1Pin = 25;
    constexpr int kLeftMotorIn2Pin = 26;
    constexpr int kLeftMotorPwmPin = 27;
    constexpr int kRightMotorIn1Pin = 33;
    constexpr int kRightMotorIn2Pin = 32;
    constexpr int kRightMotorPwmPin = 14;
    constexpr int kLeftEncoderAPin = 19;
    constexpr int kLeftEncoderBPin = 18;
    constexpr int kRightEncoderAPin = 17;
    constexpr int kRightEncoderBPin = 16;
    constexpr int kLeftPwmChannel = 0;
    constexpr int kRightPwmChannel = 1;
    constexpr int kPwmFrequency = 20000;
    constexpr int kPwmResolutionBits = 8;
    constexpr int kLeftDiagnosticPwm = 50;
    constexpr int kRightDiagnosticPwm = 55;

    enum class State : uint8_t
    {
        WAIT_BOOT,
        COUNTDOWN,
        PULSING,
        COASTING,
        COMPLETE,
        OUTPUT_LOCKED,
        ESTOP_LATCHED
    };

    volatile int32_t leftEncoderCount = 0;
    volatile int32_t rightEncoderCount = 0;
    portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;

    State state = State::WAIT_BOOT;
    uint32_t countdownStartedMs = 0;
    uint32_t pulseStartedMs = 0;
    uint32_t outputsStoppedMs = 0;
    uint32_t lastCountdownSecond = UINT32_MAX;
    uint32_t lastManualEncoderLogMs = 0;
    int32_t lastManualLeft = 0;
    int32_t lastManualRight = 0;
    bool resultReported = false;

    bool lastButtonReading = HIGH;
    bool stableButtonState = HIGH;
    bool buttonArmed = true;
    uint32_t lastButtonChangeMs = 0;

    int32_t absoluteCount(int32_t value)
    {
        return value < 0 ? -value : value;
    }

    void IRAM_ATTR leftEncoderISR()
    {
        const int encoderB = digitalRead(kLeftEncoderBPin);
        portENTER_CRITICAL_ISR(&encoderMux);
        if (encoderB == HIGH)
            --leftEncoderCount;
        else
            ++leftEncoderCount;
        portEXIT_CRITICAL_ISR(&encoderMux);
    }

    void IRAM_ATTR rightEncoderISR()
    {
        const int encoderB = digitalRead(kRightEncoderBPin);
        portENTER_CRITICAL_ISR(&encoderMux);
        if (encoderB == HIGH)
            ++rightEncoderCount;
        else
            --rightEncoderCount;
        portEXIT_CRITICAL_ISR(&encoderMux);
    }

    void readEncoderCounts(int32_t& left, int32_t& right)
    {
        portENTER_CRITICAL(&encoderMux);
        left = leftEncoderCount;
        right = rightEncoderCount;
        portEXIT_CRITICAL(&encoderMux);
    }

    void resetEncoderCounts()
    {
        portENTER_CRITICAL(&encoderMux);
        leftEncoderCount = 0;
        rightEncoderCount = 0;
        portEXIT_CRITICAL(&encoderMux);
        lastManualLeft = 0;
        lastManualRight = 0;
    }

    void forceSafeOutputs()
    {
        // Remove bridge power first, then clear PWM and direction commands.
        digitalWrite(kMotorStandbyPin, LOW);
        ledcWrite(kLeftPwmChannel, 0);
        ledcWrite(kRightPwmChannel, 0);
        digitalWrite(kLeftMotorIn1Pin, LOW);
        digitalWrite(kLeftMotorIn2Pin, LOW);
        digitalWrite(kRightMotorIn1Pin, LOW);
        digitalWrite(kRightMotorIn2Pin, LOW);
    }

    void initializeHardware()
    {
        pinMode(kMotorStandbyPin, OUTPUT);
        digitalWrite(kMotorStandbyPin, LOW);

        pinMode(kLeftMotorIn1Pin, OUTPUT);
        pinMode(kLeftMotorIn2Pin, OUTPUT);
        pinMode(kRightMotorIn1Pin, OUTPUT);
        pinMode(kRightMotorIn2Pin, OUTPUT);
        digitalWrite(kLeftMotorIn1Pin, LOW);
        digitalWrite(kLeftMotorIn2Pin, LOW);
        digitalWrite(kRightMotorIn1Pin, LOW);
        digitalWrite(kRightMotorIn2Pin, LOW);

        ledcSetup(kLeftPwmChannel, kPwmFrequency, kPwmResolutionBits);
        ledcSetup(kRightPwmChannel, kPwmFrequency, kPwmResolutionBits);
        ledcAttachPin(kLeftMotorPwmPin, kLeftPwmChannel);
        ledcAttachPin(kRightMotorPwmPin, kRightPwmChannel);
        ledcWrite(kLeftPwmChannel, 0);
        ledcWrite(kRightPwmChannel, 0);

        pinMode(kLeftEncoderAPin, INPUT_PULLUP);
        pinMode(kLeftEncoderBPin, INPUT_PULLUP);
        pinMode(kRightEncoderAPin, INPUT_PULLUP);
        pinMode(kRightEncoderBPin, INPUT_PULLUP);
        resetEncoderCounts();
        attachInterrupt(digitalPinToInterrupt(kLeftEncoderAPin),
                        leftEncoderISR,
                        RISING);
        attachInterrupt(digitalPinToInterrupt(kRightEncoderAPin),
                        rightEncoderISR,
                        RISING);

        forceSafeOutputs();
    }

    void initializeBootButton()
    {
        pinMode(kBootButtonPin, INPUT_PULLUP);
        const bool initialState = digitalRead(kBootButtonPin);
        lastButtonReading = initialState;
        stableButtonState = initialState;
        // BOOT held during reset never grants approval.
        buttonArmed = initialState == HIGH;
        lastButtonChangeMs = millis();
    }

    bool bootButtonPressed(uint32_t nowMs)
    {
        const bool reading = digitalRead(kBootButtonPin);
        if (reading != lastButtonReading)
        {
            lastButtonReading = reading;
            lastButtonChangeMs = nowMs;
        }
        if (nowMs - lastButtonChangeMs < kButtonDebounceMs
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

    bool terminalState()
    {
        return state == State::COMPLETE
            || state == State::OUTPUT_LOCKED
            || state == State::ESTOP_LATCHED;
    }

    void startDiagnosticPulse(uint32_t nowMs)
    {
        forceSafeOutputs();
        resetEncoderCounts();

#if AGV_CHANNEL_DIAGNOSTIC_MOTOR_ENABLED
#if AGV_CHANNEL_DIAGNOSTIC_CHANNEL == 1
        digitalWrite(kLeftMotorIn1Pin, LOW);
        digitalWrite(kLeftMotorIn2Pin, HIGH);
        ledcWrite(kLeftPwmChannel, kLeftDiagnosticPwm);
        ledcWrite(kRightPwmChannel, 0);
#else
        digitalWrite(kRightMotorIn1Pin, HIGH);
        digitalWrite(kRightMotorIn2Pin, LOW);
        ledcWrite(kLeftPwmChannel, 0);
        ledcWrite(kRightPwmChannel, kRightDiagnosticPwm);
#endif
        // STBY is asserted only after direction and PWM are prepared.
        digitalWrite(kMotorStandbyPin, HIGH);
        pulseStartedMs = nowMs;
        state = State::PULSING;
#else
        state = State::OUTPUT_LOCKED;
        Serial.println("[LOCKED] Pulse blocked; PWM=0/0 STBY=LOW");
#endif
    }

    void handleBootPress(uint32_t nowMs)
    {
        if (state == State::WAIT_BOOT)
        {
            countdownStartedMs = nowMs;
            lastCountdownSecond = UINT32_MAX;
            state = State::COUNTDOWN;
            Serial.println("[BOOT] COUNTDOWN_STARTED");
            return;
        }
        if (state == State::COUNTDOWN)
        {
            countdownStartedMs = 0;
            state = State::WAIT_BOOT;
            forceSafeOutputs();
            Serial.println("[BOOT] COUNTDOWN_CANCELLED");
            return;
        }
        if (state == State::PULSING || state == State::COASTING)
        {
            forceSafeOutputs();
            state = State::ESTOP_LATCHED;
            Serial.println("[BOOT] ESTOP_LATCHED; reboot required");
        }
    }

    void updateCountdown(uint32_t nowMs)
    {
        if (state != State::COUNTDOWN)
            return;

        const uint32_t elapsedMs = nowMs - countdownStartedMs;
        const uint32_t remainingMs = elapsedMs >= kApprovalCountdownMs
            ? 0 : kApprovalCountdownMs - elapsedMs;
        const uint32_t seconds = (remainingMs + 999U) / 1000U;
        if (seconds != lastCountdownSecond)
        {
            lastCountdownSecond = seconds;
            Serial.printf("[SAFE] STARTING IN %lu\n",
                          static_cast<unsigned long>(seconds));
        }

        if (remainingMs != 0)
            return;
        if (stableButtonState != HIGH)
        {
            countdownStartedMs = 0;
            state = State::WAIT_BOOT;
            forceSafeOutputs();
            Serial.println("[SAFE] BOOT must be released; approval cancelled");
            return;
        }

        Serial.println("[ARMED] One 300 ms channel pulse approved");
        startDiagnosticPulse(nowMs);
    }

    void updatePulse(uint32_t nowMs)
    {
        if (state != State::PULSING)
            return;

        int32_t left = 0;
        int32_t right = 0;
        readEncoderCounts(left, right);
        if (nowMs - pulseStartedMs < kPulseDurationMs
            && absoluteCount(left) < kMaximumDiagnosticCount
            && absoluteCount(right) < kMaximumDiagnosticCount)
        {
            return;
        }

        forceSafeOutputs();
        outputsStoppedMs = nowMs;
        state = State::COASTING;
    }

    void reportResult(uint32_t nowMs)
    {
        if (resultReported)
            return;
        if (state == State::COASTING
            && nowMs - outputsStoppedMs < kCoastCaptureMs)
        {
            return;
        }
        if (state != State::COASTING && state != State::OUTPUT_LOCKED)
            return;

        forceSafeOutputs();
        int32_t left = 0;
        int32_t right = 0;
        readEncoderCounts(left, right);
        resultReported = true;

#if AGV_CHANNEL_DIAGNOSTIC_CHANNEL == 1
        Serial.printf("[DIAG_RESULT] channel=A expectedWheel=LEFT L=%ld R=%ld\n",
                      static_cast<long>(left),
                      static_cast<long>(right));
        const int32_t expected = left;
        const int32_t other = right;
#else
        Serial.printf("[DIAG_RESULT] channel=B expectedWheel=RIGHT L=%ld R=%ld\n",
                      static_cast<long>(left),
                      static_cast<long>(right));
        const int32_t expected = right;
        const int32_t other = left;
#endif

        if (absoluteCount(expected) < kMovementThreshold
            && absoluteCount(other) < kMovementThreshold)
        {
            Serial.println("[DIAG] NO_ENCODER_ACTIVITY: report whether a physical wheel moved");
        }
        else if (absoluteCount(expected) < kMovementThreshold
                 && absoluteCount(other) >= kMovementThreshold)
        {
            Serial.println("[DIAG] CROSSED_MAPPING: the other encoder reported motion");
        }
        else if (absoluteCount(expected) >= kMovementThreshold
                 && absoluteCount(other) >= kMovementThreshold)
        {
            Serial.println("[DIAG] BOTH_ENCODERS_ACTIVE: check crossed/shared wiring");
        }
        else if (expected < 0)
        {
            Serial.println("[DIAG] EXPECTED_ENCODER_REVERSED: direction/polarity mismatch");
        }
        else
        {
            Serial.println("[DIAG] EXPECTED_MAPPING: selected output matched its encoder");
        }
        Serial.println("[SAFE] PWM=0/0 STBY=LOW; reboot required");
        state = State::COMPLETE;
    }

    void logManualEncoderChanges(uint32_t nowMs)
    {
        if (state != State::WAIT_BOOT
            || nowMs - lastManualEncoderLogMs < kManualEncoderLogIntervalMs)
        {
            return;
        }

        int32_t left = 0;
        int32_t right = 0;
        readEncoderCounts(left, right);
        if (left == lastManualLeft && right == lastManualRight)
            return;

        lastManualEncoderLogMs = nowMs;
        lastManualLeft = left;
        lastManualRight = right;
        Serial.printf("[ENCODER_MANUAL] L=%ld R=%ld\n",
                      static_cast<long>(left),
                      static_cast<long>(right));
    }
}

void setup()
{
    // Safe outputs are established before Serial or user interaction.
    initializeHardware();
    initializeBootButton();

    Serial.begin(kSerialBaud);
    delay(300);
    Serial.println();
    Serial.println("================================");
    Serial.println("MOTOR/ENCODER CHANNEL DIAGNOSTIC");
#if AGV_CHANNEL_DIAGNOSTIC_CHANNEL == 1
    Serial.println("SELECTED OUTPUT: A (AO1/AO2, EXPECT LEFT)");
#else
    Serial.println("SELECTED OUTPUT: B (BO1/BO2, EXPECT RIGHT)");
#endif
#if AGV_CHANNEL_DIAGNOSTIC_MOTOR_ENABLED
    Serial.println("MOTOR OUTPUTS: ONE 300 MS PULSE AFTER BOOT + 5 SECONDS");
    Serial.println("WARNING: BOTH WHEELS MUST REMAIN OFF THE FLOOR");
#else
    Serial.println("MOTOR OUTPUTS: COMPILE-LOCKED OFF");
#endif
    Serial.println("NETWORK: DISABLED");
    Serial.println("MANUAL CHECK: TURN ONE PHYSICAL WHEEL; WATCH L/R");
    Serial.println("BOOT: START/CANCEL; DURING PULSE: LATCHED E-STOP");
    Serial.println("================================");
    Serial.println("[SAFE] Press BOOT once only when ready");
}

void loop()
{
    const uint32_t nowMs = millis();

    if (terminalState())
        forceSafeOutputs();

    if (bootButtonPressed(nowMs))
        handleBootPress(nowMs);

    updateCountdown(nowMs);
    updatePulse(nowMs);
    reportResult(nowMs);
    logManualEncoderChanges(nowMs);

    if (terminalState())
        forceSafeOutputs();
    if (state != State::PULSING)
        delay(2);
}

#endif // AGV_CHANNEL_DIAGNOSTIC_ENABLED
