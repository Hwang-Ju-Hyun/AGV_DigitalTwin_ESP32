#include "MotionController.hpp"

#include "Config.hpp"

#include <climits>

namespace
{
    // Values preserved from AGV_Project_Record/final_l_route_main.cpp.
    constexpr int kForwardLeftStartPwm = 50;
    constexpr int kForwardRightStartPwm = 55;
    constexpr int kForwardLeftCruisePwm = 80;
    constexpr int kForwardRightCruisePwm = 85;
    constexpr int32_t kForwardAccelCounts = 100;
    constexpr int32_t kForwardDecelCounts = 160;
    constexpr int kForwardMinPwm = 42;
    constexpr int kForwardMaxPwm = 100;
    constexpr float kForwardSyncKp = 0.25f;
    constexpr int kSyncCorrectionLimit = 15;

    constexpr int32_t kWrongDirectionLimit = -10;
    constexpr int32_t kCountOverrunAllowance = 100;
    constexpr int32_t kWheelMismatchLimit = 80;
    constexpr uint32_t kMotionTimeoutMs = 10000;
    constexpr uint32_t kProgressCheckMs = 300;
    constexpr int32_t kMinimumWindowAdvance = 2;
    constexpr uint8_t kMaximumNoProgressWindows = 4;

    int clampInt(int value, int minimum, int maximum)
    {
        if (value < minimum)
            return minimum;
        if (value > maximum)
            return maximum;
        return value;
    }

    float clampUnit(float value)
    {
        if (value < 0.0f)
            return 0.0f;
        if (value > 1.0f)
            return 1.0f;
        return value;
    }
}

volatile int32_t MotionController::s_LeftCount = 0;
volatile int32_t MotionController::s_RightCount = 0;
portMUX_TYPE MotionController::s_EncoderMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR MotionController::leftEncoderISR()
{
    const int encoderB = digitalRead(AppConfig::kLeftEncoderBPin);
    portENTER_CRITICAL_ISR(&s_EncoderMux);
    if (encoderB == HIGH)
        --s_LeftCount;
    else
        ++s_LeftCount;
    portEXIT_CRITICAL_ISR(&s_EncoderMux);
}

void IRAM_ATTR MotionController::rightEncoderISR()
{
    const int encoderB = digitalRead(AppConfig::kRightEncoderBPin);
    portENTER_CRITICAL_ISR(&s_EncoderMux);
    if (encoderB == HIGH)
        ++s_RightCount;
    else
        --s_RightCount;
    portEXIT_CRITICAL_ISR(&s_EncoderMux);
}

void MotionController::readEncoderCounts(int32_t& left, int32_t& right)
{
    portENTER_CRITICAL(&s_EncoderMux);
    left = s_LeftCount;
    right = s_RightCount;
    portEXIT_CRITICAL(&s_EncoderMux);
}

void MotionController::resetEncoderCounts()
{
    portENTER_CRITICAL(&s_EncoderMux);
    s_LeftCount = 0;
    s_RightCount = 0;
    portEXIT_CRITICAL(&s_EncoderMux);
}

void MotionController::begin()
{
    if (m_Initialized)
    {
        forceSafeOutputs();
        return;
    }

    // STBY is made safe before any other motor pin is configured.
    pinMode(AppConfig::kMotorStandbyPin, OUTPUT);
    digitalWrite(AppConfig::kMotorStandbyPin, LOW);

    pinMode(AppConfig::kLeftMotorIn1Pin, OUTPUT);
    pinMode(AppConfig::kLeftMotorIn2Pin, OUTPUT);
    pinMode(AppConfig::kRightMotorIn1Pin, OUTPUT);
    pinMode(AppConfig::kRightMotorIn2Pin, OUTPUT);
    digitalWrite(AppConfig::kLeftMotorIn1Pin, LOW);
    digitalWrite(AppConfig::kLeftMotorIn2Pin, LOW);
    digitalWrite(AppConfig::kRightMotorIn1Pin, LOW);
    digitalWrite(AppConfig::kRightMotorIn2Pin, LOW);

    ledcSetup(AppConfig::kLeftPwmChannel,
              AppConfig::kPwmFrequency,
              AppConfig::kPwmResolutionBits);
    ledcSetup(AppConfig::kRightPwmChannel,
              AppConfig::kPwmFrequency,
              AppConfig::kPwmResolutionBits);
    ledcAttachPin(AppConfig::kLeftMotorPwmPin, AppConfig::kLeftPwmChannel);
    ledcAttachPin(AppConfig::kRightMotorPwmPin, AppConfig::kRightPwmChannel);
    ledcWrite(AppConfig::kLeftPwmChannel, 0);
    ledcWrite(AppConfig::kRightPwmChannel, 0);

    pinMode(AppConfig::kLeftEncoderAPin, INPUT_PULLUP);
    pinMode(AppConfig::kLeftEncoderBPin, INPUT_PULLUP);
    pinMode(AppConfig::kRightEncoderAPin, INPUT_PULLUP);
    pinMode(AppConfig::kRightEncoderBPin, INPUT_PULLUP);
    resetEncoderCounts();
    attachInterrupt(digitalPinToInterrupt(AppConfig::kLeftEncoderAPin),
                    leftEncoderISR,
                    RISING);
    attachInterrupt(digitalPinToInterrupt(AppConfig::kRightEncoderAPin),
                    rightEncoderISR,
                    RISING);

    m_Initialized = true;
    m_State = State::IDLE;
    forceSafeOutputs();
}

MotionController::StartResult MotionController::startForward(int32_t targetCount)
{
    return startForward(targetCount, millis());
}

MotionController::StartResult MotionController::startForward(int32_t targetCount,
                                                             uint32_t nowMs)
{
    if (!m_Initialized)
        return StartResult::NOT_READY;

    if (m_State == State::FAULTED)
    {
        forceSafeOutputs();
        return StartResult::FAULT_LATCHED;
    }

    if (m_State == State::RUNNING)
        return StartResult::ALREADY_RUNNING;

    if (targetCount <= 0 || targetCount > INT32_MAX - kCountOverrunAllowance)
    {
        forceSafeOutputs();
        return StartResult::INVALID_TARGET;
    }

    // This branch is deliberately before all HIGH/non-zero output commands.
    // With the current Phase 2A compile lock, start is a safe no-op.
    if (!AppConfig::kEnableMotorOutputs)
    {
        forceSafeOutputs();
        return StartResult::OUTPUT_DISABLED;
    }

    forceSafeOutputs();
    resetEncoderCounts();
    m_TargetCount = targetCount;
    m_MotionStartedMs = nowMs;
    m_FinalElapsedMs = 0;
    m_LastProgressCheckMs = nowMs;
    m_LastVelocitySampleMs = nowMs;
    m_LastProgressLeft = 0;
    m_LastProgressRight = 0;
    m_LastVelocityAverage = 0;
    m_LeftNoProgressWindows = 0;
    m_RightNoProgressWindows = 0;
    m_VelocityCountsPerSecond = 0.0f;
    m_State = State::RUNNING;

    applyForwardOutputs(kForwardLeftStartPwm, kForwardRightStartPwm);
    return StartResult::STARTED;
}

MotionController::UpdateResult MotionController::update(uint32_t nowMs)
{
    if (m_State == State::FAULTED)
    {
        forceSafeOutputs();
        return UpdateResult::FAULTED;
    }

    if (m_State == State::COMPLETE)
    {
        forceSafeOutputs();
        return outputsSafe() ? UpdateResult::COMPLETE
                             : latchFault(Fault::OUTPUT_INVARIANT, nowMs);
    }

    if (m_State != State::RUNNING)
    {
        if (m_Initialized)
            forceSafeOutputs();
        return UpdateResult::IDLE;
    }

    if (!AppConfig::kEnableMotorOutputs)
        return latchFault(Fault::OUTPUT_INVARIANT, nowMs);

    int32_t leftCount = 0;
    int32_t rightCount = 0;
    readEncoderCounts(leftCount, rightCount);

    if (leftCount < kWrongDirectionLimit || rightCount < kWrongDirectionLimit)
        return latchFault(Fault::WRONG_DIRECTION, nowMs);

    if (leftCount > m_TargetCount + kCountOverrunAllowance
        || rightCount > m_TargetCount + kCountOverrunAllowance)
    {
        return latchFault(Fault::COUNT_OVERRUN, nowMs);
    }

    int64_t countDifference = static_cast<int64_t>(leftCount)
                            - static_cast<int64_t>(rightCount);
    if (countDifference < 0)
        countDifference = -countDifference;
    if (countDifference > kWheelMismatchLimit)
        return latchFault(Fault::WHEEL_MISMATCH, nowMs);

    updateVelocity(leftCount, rightCount, nowMs);

    if (leftCount >= m_TargetCount && rightCount >= m_TargetCount)
    {
        forceSafeOutputs();
        m_FinalElapsedMs = nowMs - m_MotionStartedMs;
        m_VelocityCountsPerSecond = 0.0f;
        m_State = State::COMPLETE;
        return outputsSafe() ? UpdateResult::COMPLETE
                             : latchFault(Fault::OUTPUT_INVARIANT, nowMs);
    }

    if (nowMs - m_MotionStartedMs >= kMotionTimeoutMs)
        return latchFault(Fault::TIMEOUT, nowMs);

    if (nowMs - m_LastProgressCheckMs >= kProgressCheckMs)
    {
        const int32_t leftAdvance = leftCount - m_LastProgressLeft;
        const int32_t rightAdvance = rightCount - m_LastProgressRight;

        if (leftCount < m_TargetCount && leftAdvance < kMinimumWindowAdvance)
            ++m_LeftNoProgressWindows;
        else
            m_LeftNoProgressWindows = 0;

        if (rightCount < m_TargetCount && rightAdvance < kMinimumWindowAdvance)
            ++m_RightNoProgressWindows;
        else
            m_RightNoProgressWindows = 0;

        m_LastProgressLeft = leftCount;
        m_LastProgressRight = rightCount;
        m_LastProgressCheckMs = nowMs;

        if (m_LeftNoProgressWindows >= kMaximumNoProgressWindows
            || m_RightNoProgressWindows >= kMaximumNoProgressWindows)
        {
            return latchFault(Fault::STALL, nowMs);
        }
    }

    int32_t slowerProgress = leftCount < rightCount ? leftCount : rightCount;
    int32_t leadingProgress = leftCount > rightCount ? leftCount : rightCount;
    if (slowerProgress < 0)
        slowerProgress = 0;

    int32_t remaining = m_TargetCount - leadingProgress;
    if (remaining < 0)
        remaining = 0;

    const float accelRatio = clampUnit(
        static_cast<float>(slowerProgress) / static_cast<float>(kForwardAccelCounts));
    const float decelRatio = clampUnit(
        static_cast<float>(remaining) / static_cast<float>(kForwardDecelCounts));
    const float motionRatio = accelRatio < decelRatio ? accelRatio : decelRatio;

    const int baseLeftPwm = kForwardLeftStartPwm
        + static_cast<int>((kForwardLeftCruisePwm - kForwardLeftStartPwm) * motionRatio);
    const int baseRightPwm = kForwardRightStartPwm
        + static_cast<int>((kForwardRightCruisePwm - kForwardRightStartPwm) * motionRatio);

    const int32_t syncError = leftCount - rightCount;
    const int correction = clampInt(static_cast<int>(syncError * kForwardSyncKp),
                                    -kSyncCorrectionLimit,
                                    kSyncCorrectionLimit);

    int leftPwm = baseLeftPwm - correction;
    int rightPwm = baseRightPwm + correction;
    leftPwm = leftCount >= m_TargetCount
        ? 0
        : clampInt(leftPwm, kForwardMinPwm, kForwardMaxPwm);
    rightPwm = rightCount >= m_TargetCount
        ? 0
        : clampInt(rightPwm, kForwardMinPwm, kForwardMaxPwm);

    applyForwardOutputs(leftPwm, rightPwm);
    return UpdateResult::RUNNING;
}

void MotionController::stopImmediately()
{
    if (m_State == State::RUNNING)
    {
        const uint32_t nowMs = millis();
        forceSafeOutputs();
        m_FinalElapsedMs = nowMs - m_MotionStartedMs;
        m_VelocityCountsPerSecond = 0.0f;
        m_State = State::IDLE;
        return;
    }

    if (m_Initialized)
        forceSafeOutputs();
}

void MotionController::emergencyStop(Fault cause)
{
    if (cause == Fault::NONE)
        cause = Fault::EXTERNAL_STOP;
    latchFault(cause, millis());
}

MotionController::Snapshot MotionController::snapshot() const
{
    Snapshot result;
    readEncoderCounts(result.leftCount, result.rightCount);
    result.targetCount = m_TargetCount;
    result.leftPwm = m_LeftPwm;
    result.rightPwm = m_RightPwm;
    result.velocityCountsPerSecond = m_VelocityCountsPerSecond;
    result.elapsedMs = m_State == State::RUNNING
        ? millis() - m_MotionStartedMs
        : m_FinalElapsedMs;
    result.running = running();
    result.completed = completed();
    result.faultLatched = faultLatched();
    result.outputsSafe = outputsSafe();
    result.fault = m_Fault;

    if (m_TargetCount > 0)
    {
        int32_t safeLeft = result.leftCount < 0 ? 0 : result.leftCount;
        int32_t safeRight = result.rightCount < 0 ? 0 : result.rightCount;
        const int32_t slowerProgress = safeLeft < safeRight ? safeLeft : safeRight;
        result.progress = clampUnit(
            static_cast<float>(slowerProgress) / static_cast<float>(m_TargetCount));
    }

    return result;
}

bool MotionController::outputsSafe() const
{
    if (!m_Initialized)
        return false;

    return m_LeftPwm == 0
        && m_RightPwm == 0
        && digitalRead(AppConfig::kMotorStandbyPin) == LOW
        && digitalRead(AppConfig::kLeftMotorIn1Pin) == LOW
        && digitalRead(AppConfig::kLeftMotorIn2Pin) == LOW
        && digitalRead(AppConfig::kRightMotorIn1Pin) == LOW
        && digitalRead(AppConfig::kRightMotorIn2Pin) == LOW;
}

bool MotionController::running() const
{
    return m_State == State::RUNNING;
}

bool MotionController::completed() const
{
    return m_State == State::COMPLETE;
}

bool MotionController::faultLatched() const
{
    return m_State == State::FAULTED;
}

MotionController::Fault MotionController::fault() const
{
    return m_Fault;
}

void MotionController::forceSafeOutputs()
{
    // Disable the bridge first; clearing duty and direction follows immediately.
    digitalWrite(AppConfig::kMotorStandbyPin, LOW);
    ledcWrite(AppConfig::kLeftPwmChannel, 0);
    ledcWrite(AppConfig::kRightPwmChannel, 0);
    m_LeftPwm = 0;
    m_RightPwm = 0;

    digitalWrite(AppConfig::kLeftMotorIn1Pin, LOW);
    digitalWrite(AppConfig::kLeftMotorIn2Pin, LOW);
    digitalWrite(AppConfig::kRightMotorIn1Pin, LOW);
    digitalWrite(AppConfig::kRightMotorIn2Pin, LOW);
}

void MotionController::applyForwardOutputs(int leftPwm, int rightPwm)
{
    // No HIGH direction/STBY writes or non-zero PWM writes are reachable while
    // the compile-time output lock is false.
    if (!AppConfig::kEnableMotorOutputs)
    {
        forceSafeOutputs();
        return;
    }

    leftPwm = clampInt(leftPwm, 0, kForwardMaxPwm);
    rightPwm = clampInt(rightPwm, 0, kForwardMaxPwm);

    digitalWrite(AppConfig::kLeftMotorIn1Pin, LOW);
    digitalWrite(AppConfig::kLeftMotorIn2Pin, HIGH);
    digitalWrite(AppConfig::kRightMotorIn1Pin, HIGH);
    digitalWrite(AppConfig::kRightMotorIn2Pin, LOW);
    ledcWrite(AppConfig::kLeftPwmChannel, leftPwm);
    ledcWrite(AppConfig::kRightPwmChannel, rightPwm);
    m_LeftPwm = leftPwm;
    m_RightPwm = rightPwm;
    // STBY is the final enable after direction and PWM are fully established.
    digitalWrite(AppConfig::kMotorStandbyPin, HIGH);
}

MotionController::UpdateResult MotionController::latchFault(Fault cause,
                                                            uint32_t nowMs)
{
    forceSafeOutputs();
    if (m_State == State::RUNNING)
        m_FinalElapsedMs = nowMs - m_MotionStartedMs;
    m_VelocityCountsPerSecond = 0.0f;
    if (m_Fault == Fault::NONE)
        m_Fault = cause == Fault::NONE ? Fault::EXTERNAL_STOP : cause;
    m_State = State::FAULTED;
    return UpdateResult::FAULTED;
}

void MotionController::updateVelocity(int32_t leftCount,
                                      int32_t rightCount,
                                      uint32_t nowMs)
{
    const uint32_t elapsedMs = nowMs - m_LastVelocitySampleMs;
    if (elapsedMs < kProgressCheckMs)
        return;

    const int32_t averageCount = static_cast<int32_t>(
        (static_cast<int64_t>(leftCount) + static_cast<int64_t>(rightCount)) / 2);
    const int32_t advance = averageCount - m_LastVelocityAverage;
    m_VelocityCountsPerSecond =
        static_cast<float>(advance) * 1000.0f / static_cast<float>(elapsedMs);
    m_LastVelocityAverage = averageCount;
    m_LastVelocitySampleMs = nowMs;
}
