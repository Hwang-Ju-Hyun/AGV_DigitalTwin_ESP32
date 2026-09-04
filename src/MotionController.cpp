#include "MotionController.hpp"

#include "Config.hpp"

#include <climits>
#include <cmath>

namespace
{
    // Values preserved from AGV_Project_Record/final_l_route_main.cpp.
    constexpr int kForwardLeftStartPwm = 50;
    constexpr int kForwardRightStartPwm = 55;
    constexpr int kForwardLeftCruisePwm = 80;
    constexpr int kForwardRightCruisePwm = 85;
    constexpr int kPhysicalFleetForwardStartPwm = 50;
    constexpr int kPhysicalFleetForwardCruisePwm = 80;
    constexpr int32_t kForwardAccelCounts = 100;
    constexpr int32_t kForwardDecelCounts = 160;
    constexpr int kForwardMinPwm = 42;
    constexpr int kForwardMaxPwm = 100;
    constexpr float kForwardSyncKp = 0.25f;
    constexpr float kForwardSyncKd = 0.50f;
    constexpr uint32_t kForwardSyncSampleMs = 50;
    constexpr int kForwardSyncCorrectionLimit = 15;

    // Correction primitives trade travel time for lower overshoot. Minimum
    // PWM stays at the existing proven floor; only start/cruise ceilings and
    // ramp distances are reduced.
    constexpr int kCorrectionForwardLeftStartPwm = 46;
    constexpr int kCorrectionForwardRightStartPwm = 46;
    constexpr int kCorrectionForwardLeftCruisePwm = 62;
    constexpr int kCorrectionForwardRightCruisePwm = 62;
    constexpr int32_t kCorrectionForwardAccelCounts = 45;
    constexpr int32_t kCorrectionForwardDecelCounts = 70;
    constexpr int kCorrectionForwardMaxPwm = 75;
    constexpr int kCorrectionForwardSyncCorrectionLimit = 10;

    // Turn PWM values are preserved from the physically verified L-route.
    // Direction pins below use the Server convention: positive heading is a
    // physical counterclockwise turn when viewed from above.
    constexpr int kTurnLeftCruisePwm = 58;
    constexpr int kTurnRightCruisePwm = 63;
    constexpr int kTurnLeftMidPwm = 50;
    constexpr int kTurnRightMidPwm = 55;
    constexpr int kTurnLeftSlowPwm = 44;
    constexpr int kTurnRightSlowPwm = 49;
    constexpr int32_t kTurnCruiseRemainingCounts = 70;
    constexpr int32_t kTurnMidRemainingCounts = 25;
    constexpr int kTurnMinPwm = 36;
    constexpr int kTurnMaxPwm = 90;
    constexpr float kTurnSyncKp = 0.20f;
    constexpr int kTurnSyncCorrectionLimit = 12;

    constexpr int kCorrectionTurnLeftCruisePwm = 52;
    constexpr int kCorrectionTurnRightCruisePwm = 57;
    constexpr int kCorrectionTurnLeftMidPwm = 48;
    constexpr int kCorrectionTurnRightMidPwm = 53;
    constexpr int kCorrectionTurnLeftSlowPwm = 44;
    constexpr int kCorrectionTurnRightSlowPwm = 49;
    constexpr int kCorrectionTurnMaxPwm = 65;
    constexpr int kCorrectionTurnSyncCorrectionLimit = 10;

    constexpr int32_t kWrongDirectionLimit = -10;
    constexpr int32_t kForwardCountOverrunAllowance = 100;
    constexpr int32_t kTurnCountOverrunAllowance = 60;
    constexpr int32_t kForwardWheelMismatchLimit = 80;
    constexpr int32_t kTurnWheelMismatchLimit = 50;
    constexpr uint32_t kForwardMotionTimeoutMs = 10000;
    constexpr uint32_t kTurnMotionTimeoutMs = 6000;
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

    bool isTurnMode(MotionController::Mode mode)
    {
        return mode == MotionController::Mode::TURN_CW
            || mode == MotionController::Mode::TURN_CCW;
    }

    bool isValidMode(MotionController::Mode mode)
    {
        return mode == MotionController::Mode::FORWARD || isTurnMode(mode);
    }

    int32_t countOverrunAllowance(MotionController::Mode mode)
    {
        return isTurnMode(mode) ? kTurnCountOverrunAllowance
                                : kForwardCountOverrunAllowance;
    }

    int32_t wheelMismatchLimit(MotionController::Mode mode)
    {
        return isTurnMode(mode) ? kTurnWheelMismatchLimit
                                : kForwardWheelMismatchLimit;
    }

    uint32_t motionTimeoutMs(MotionController::Mode mode)
    {
        return isTurnMode(mode) ? kTurnMotionTimeoutMs
                                : kForwardMotionTimeoutMs;
    }

    int maximumPwm(MotionController::Mode mode,
                   MotionController::Profile profile)
    {
        if (profile == MotionController::Profile::CORRECTION)
            return isTurnMode(mode) ? kCorrectionTurnMaxPwm
                                    : kCorrectionForwardMaxPwm;
        return isTurnMode(mode) ? kTurnMaxPwm : kForwardMaxPwm;
    }
}

volatile int32_t MotionController::s_LeftCount = 0;
volatile int32_t MotionController::s_RightCount = 0;
volatile uint32_t MotionController::s_EncoderActivitySequence = 0;
volatile uint32_t MotionController::s_EncoderResetEpoch = 0;
portMUX_TYPE MotionController::s_EncoderMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR MotionController::leftEncoderISR()
{
    const int encoderB = digitalRead(AppConfig::kLeftEncoderBPin);
    portENTER_CRITICAL_ISR(&s_EncoderMux);
    if (encoderB == HIGH)
        --s_LeftCount;
    else
        ++s_LeftCount;
    ++s_EncoderActivitySequence;
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
    ++s_EncoderActivitySequence;
    portEXIT_CRITICAL_ISR(&s_EncoderMux);
}

void MotionController::readEncoderCounts(int32_t& left, int32_t& right)
{
    portENTER_CRITICAL(&s_EncoderMux);
    left = s_LeftCount;
    right = s_RightCount;
    portEXIT_CRITICAL(&s_EncoderMux);
}

void MotionController::readEncoderState(int32_t& left,
                                        int32_t& right,
                                        uint32_t& activitySequence)
{
    portENTER_CRITICAL(&s_EncoderMux);
    left = s_LeftCount;
    right = s_RightCount;
    activitySequence = s_EncoderActivitySequence;
    portEXIT_CRITICAL(&s_EncoderMux);
}

void MotionController::readEncoderSnapshot(int32_t& left,
                                           int32_t& right,
                                           uint32_t& resetEpoch)
{
    portENTER_CRITICAL(&s_EncoderMux);
    left = s_LeftCount;
    right = s_RightCount;
    resetEpoch = s_EncoderResetEpoch;
    portEXIT_CRITICAL(&s_EncoderMux);
}

void MotionController::resetEncoderCounts()
{
    portENTER_CRITICAL(&s_EncoderMux);
    s_LeftCount = 0;
    s_RightCount = 0;
    s_EncoderActivitySequence = 0;
    ++s_EncoderResetEpoch;
    portEXIT_CRITICAL(&s_EncoderMux);
}

void MotionController::normalizeCounts(Mode mode,
                                       int32_t rawLeft,
                                       int32_t rawRight,
                                       int32_t& leftProgress,
                                       int32_t& rightProgress)
{
    switch (mode)
    {
        case Mode::FORWARD:
            leftProgress = rawLeft;
            rightProgress = rawRight;
            break;

        case Mode::TURN_CW:
            // Physical CW: left wheel forward, right wheel reverse.
            leftProgress = rawLeft;
            rightProgress = -rawRight;
            break;

        case Mode::TURN_CCW:
            // Physical CCW: left wheel reverse, right wheel forward.
            leftProgress = -rawLeft;
            rightProgress = rawRight;
            break;

        case Mode::NONE:
        default:
            leftProgress = 0;
            rightProgress = 0;
            break;
    }
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
    return startMotion(Mode::FORWARD, targetCount, millis());
}

MotionController::StartResult MotionController::startForward(int32_t targetCount,
                                                             uint32_t nowMs)
{
    return startMotion(Mode::FORWARD, targetCount, nowMs);
}

MotionController::StartResult MotionController::startMotion(Mode mode,
                                                            int32_t targetCount)
{
    return startMotion(mode, targetCount, millis(), Profile::NORMAL);
}

MotionController::StartResult MotionController::startMotion(Mode mode,
                                                            int32_t targetCount,
                                                            uint32_t nowMs)
{
    return startMotion(mode, targetCount, nowMs, Profile::NORMAL);
}

MotionController::StartResult MotionController::startMotion(
    Mode mode,
    int32_t targetCount,
    Profile profile)
{
    return startMotion(mode, targetCount, millis(), profile);
}

MotionController::StartResult MotionController::startMotion(
    Mode mode,
    int32_t targetCount,
    uint32_t nowMs,
    Profile profile)
{
    if (!m_Initialized)
        return StartResult::NOT_READY;

    if (m_State == State::FAULTED)
    {
        forceSafeOutputs();
        return StartResult::FAULT_LATCHED;
    }

    if (m_State == State::RUNNING || m_State == State::SETTLING)
        return StartResult::ALREADY_RUNNING;

    if (!isValidMode(mode))
    {
        forceSafeOutputs();
        return StartResult::INVALID_MODE;
    }

    if (targetCount <= 0
        || targetCount > INT32_MAX - countOverrunAllowance(mode))
    {
        forceSafeOutputs();
        return StartResult::INVALID_TARGET;
    }

    const bool applyFleetForwardTrim = mode == Mode::FORWARD
        && profile != Profile::NORMAL;
    const int64_t leftTargetCount = applyFleetForwardTrim
        ? static_cast<int64_t>(std::llround(
            static_cast<double>(targetCount)
            * AppConfig::kForwardLeftTargetScale))
        : targetCount;
    const int64_t rightTargetCount = applyFleetForwardTrim
        ? static_cast<int64_t>(std::llround(
            static_cast<double>(targetCount)
            * AppConfig::kForwardRightTargetScale))
        : targetCount;
    const int32_t overrunAllowance = countOverrunAllowance(mode);
    if (leftTargetCount <= 0 || rightTargetCount <= 0
        || leftTargetCount > INT32_MAX - overrunAllowance
        || rightTargetCount > INT32_MAX - overrunAllowance)
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
    m_Mode = mode;
    m_Profile = profile;
    m_TargetCount = targetCount;
    m_LeftTargetCount = static_cast<int32_t>(leftTargetCount);
    m_RightTargetCount = static_cast<int32_t>(rightTargetCount);
    m_MotionStartedMs = nowMs;
    m_FinalElapsedMs = 0;
    m_SettlingStartedMs = 0;
    m_SettlingStableSinceMs = 0;
    m_LastProgressCheckMs = nowMs;
    m_LastVelocitySampleMs = nowMs;
    m_LastSyncSampleMs = nowMs;
    m_LastProgressLeft = 0;
    m_LastProgressRight = 0;
    m_LastVelocityAverage = 0;
    m_LastSyncLeft = 0;
    m_LastSyncRight = 0;
    m_LeftIntervalDelta = 0;
    m_RightIntervalDelta = 0;
    m_CumulativeSyncError = 0;
    m_IntervalVelocityError = 0;
    m_SyncCorrection = 0;
    m_SettlingLastLeft = 0;
    m_SettlingLastRight = 0;
    m_SettlingStartLeft = 0;
    m_SettlingStartRight = 0;
    m_SettlingLastActivitySequence = 0;
    m_HasSettlingBaseline = false;
    m_LeftNoProgressWindows = 0;
    m_RightNoProgressWindows = 0;
    m_VelocityCountsPerSecond = 0.0f;
    m_State = State::RUNNING;

    if (mode == Mode::FORWARD)
    {
        if (profile == Profile::CORRECTION)
        {
            applyMotionOutputs(mode,
                               kCorrectionForwardLeftStartPwm,
                               kCorrectionForwardRightStartPwm);
        }
        else
        {
            const bool physicalFleetProfile =
                profile == Profile::PHYSICAL_FLEET;
            applyMotionOutputs(
                mode,
                physicalFleetProfile ? kPhysicalFleetForwardStartPwm
                                     : kForwardLeftStartPwm,
                physicalFleetProfile ? kPhysicalFleetForwardStartPwm
                                     : kForwardRightStartPwm);
        }
    }
    else
    {
        if (profile == Profile::CORRECTION)
        {
            if (targetCount > kTurnCruiseRemainingCounts)
            {
                applyMotionOutputs(mode,
                                   kCorrectionTurnLeftCruisePwm,
                                   kCorrectionTurnRightCruisePwm);
            }
            else if (targetCount > kTurnMidRemainingCounts)
            {
                applyMotionOutputs(mode,
                                   kCorrectionTurnLeftMidPwm,
                                   kCorrectionTurnRightMidPwm);
            }
            else
            {
                applyMotionOutputs(mode,
                                   kCorrectionTurnLeftSlowPwm,
                                   kCorrectionTurnRightSlowPwm);
            }
        }
        else
            applyMotionOutputs(mode, kTurnLeftCruisePwm, kTurnRightCruisePwm);
    }
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

    if (m_State == State::SETTLING)
        return updateSettling(nowMs);

    if (m_State != State::RUNNING)
    {
        if (m_Initialized)
            forceSafeOutputs();
        return UpdateResult::IDLE;
    }

    if (!AppConfig::kEnableMotorOutputs)
        return latchFault(Fault::OUTPUT_INVARIANT, nowMs);

    int32_t rawLeftCount = 0;
    int32_t rawRightCount = 0;
    uint32_t activitySequence = 0;
    readEncoderState(rawLeftCount, rawRightCount, activitySequence);

    int32_t leftCount = 0;
    int32_t rightCount = 0;
    normalizeCounts(m_Mode,
                    rawLeftCount,
                    rawRightCount,
                    leftCount,
                    rightCount);

    if (leftCount >= m_LeftTargetCount
        && rightCount >= m_RightTargetCount)
    {
        // Remove drive power on the same control-loop sample that observes
        // both targets. Encoder stability is verified afterwards without a
        // blocking delay.
        forceSafeOutputs();
        if (!outputsSafe())
            return latchFault(Fault::OUTPUT_INVARIANT, nowMs);

        m_VelocityCountsPerSecond = 0.0f;
        m_SettlingStartedMs = nowMs;
        m_SettlingStableSinceMs = nowMs;
        m_SettlingLastLeft = leftCount;
        m_SettlingLastRight = rightCount;
        m_SettlingStartLeft = leftCount;
        m_SettlingStartRight = rightCount;
        m_SettlingLastActivitySequence = activitySequence;
        m_HasSettlingBaseline = true;
        m_State = State::SETTLING;
        return updateSettling(nowMs);
    }

    if (leftCount < kWrongDirectionLimit || rightCount < kWrongDirectionLimit)
        return latchFault(Fault::WRONG_DIRECTION, nowMs);

    const int32_t overrunAllowance = countOverrunAllowance(m_Mode);
    if (leftCount > m_LeftTargetCount + overrunAllowance
        || rightCount > m_RightTargetCount + overrunAllowance)
    {
        return latchFault(Fault::COUNT_OVERRUN, nowMs);
    }

    int64_t countDifference = static_cast<int64_t>(leftCount)
                            - static_cast<int64_t>(rightCount);
    if (countDifference < 0)
        countDifference = -countDifference;
    if (countDifference > wheelMismatchLimit(m_Mode))
        return latchFault(Fault::WHEEL_MISMATCH, nowMs);

    updateVelocity(leftCount, rightCount, nowMs);

    if (nowMs - m_MotionStartedMs >= motionTimeoutMs(m_Mode))
        return latchFault(Fault::TIMEOUT, nowMs);

    if (nowMs - m_LastProgressCheckMs >= kProgressCheckMs)
    {
        const int32_t leftAdvance = leftCount - m_LastProgressLeft;
        const int32_t rightAdvance = rightCount - m_LastProgressRight;

        if (leftCount < m_LeftTargetCount
            && leftAdvance < kMinimumWindowAdvance)
            ++m_LeftNoProgressWindows;
        else
            m_LeftNoProgressWindows = 0;

        if (rightCount < m_RightTargetCount
            && rightAdvance < kMinimumWindowAdvance)
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

    int baseLeftPwm = 0;
    int baseRightPwm = 0;
    int minimumPwm = 0;
    int maximumPwmValue = 0;
    float syncKp = 0.0f;
    int correctionLimit = 0;

    if (m_Mode == Mode::FORWARD)
    {
        int32_t slowerProgress = leftCount < rightCount ? leftCount : rightCount;
        if (slowerProgress < 0)
            slowerProgress = 0;

        int32_t leftRemaining = m_LeftTargetCount - leftCount;
        int32_t rightRemaining = m_RightTargetCount - rightCount;
        if (leftRemaining < 0)
            leftRemaining = 0;
        if (rightRemaining < 0)
            rightRemaining = 0;
        const int32_t remaining = leftRemaining < rightRemaining
            ? leftRemaining : rightRemaining;

        const bool correctionProfile =
            m_Profile == Profile::CORRECTION;
        const int32_t accelCounts = correctionProfile
            ? kCorrectionForwardAccelCounts : kForwardAccelCounts;
        const int32_t decelCounts = correctionProfile
            ? kCorrectionForwardDecelCounts : kForwardDecelCounts;
        const int leftStartPwm = correctionProfile
            ? kCorrectionForwardLeftStartPwm
            : (m_Profile == Profile::PHYSICAL_FLEET
                ? kPhysicalFleetForwardStartPwm : kForwardLeftStartPwm);
        const int rightStartPwm = correctionProfile
            ? kCorrectionForwardRightStartPwm
            : (m_Profile == Profile::PHYSICAL_FLEET
                ? kPhysicalFleetForwardStartPwm : kForwardRightStartPwm);
        const int leftCruisePwm = correctionProfile
            ? kCorrectionForwardLeftCruisePwm
            : (m_Profile == Profile::PHYSICAL_FLEET
                ? kPhysicalFleetForwardCruisePwm : kForwardLeftCruisePwm);
        const int rightCruisePwm = correctionProfile
            ? kCorrectionForwardRightCruisePwm
            : (m_Profile == Profile::PHYSICAL_FLEET
                ? kPhysicalFleetForwardCruisePwm : kForwardRightCruisePwm);

        const float accelRatio = clampUnit(
            static_cast<float>(slowerProgress)
            / static_cast<float>(accelCounts));
        const float decelRatio = clampUnit(
            static_cast<float>(remaining)
            / static_cast<float>(decelCounts));
        const float motionRatio = accelRatio < decelRatio ? accelRatio : decelRatio;

        baseLeftPwm = leftStartPwm
            + static_cast<int>((leftCruisePwm - leftStartPwm)
                               * motionRatio);
        baseRightPwm = rightStartPwm
            + static_cast<int>((rightCruisePwm - rightStartPwm)
                               * motionRatio);
        minimumPwm = kForwardMinPwm;
        maximumPwmValue = correctionProfile
            ? kCorrectionForwardMaxPwm : kForwardMaxPwm;
        syncKp = kForwardSyncKp;
        correctionLimit = correctionProfile
            ? kCorrectionForwardSyncCorrectionLimit
            : kForwardSyncCorrectionLimit;
    }
    else
    {
        const int32_t averageProgress = static_cast<int32_t>(
            (static_cast<int64_t>(leftCount) + static_cast<int64_t>(rightCount)) / 2);
        const int32_t remaining = m_TargetCount - averageProgress;

        const bool correctionProfile =
            m_Profile == Profile::CORRECTION;
        if (remaining > kTurnCruiseRemainingCounts)
        {
            baseLeftPwm = correctionProfile
                ? kCorrectionTurnLeftCruisePwm : kTurnLeftCruisePwm;
            baseRightPwm = correctionProfile
                ? kCorrectionTurnRightCruisePwm : kTurnRightCruisePwm;
        }
        else if (remaining > kTurnMidRemainingCounts)
        {
            baseLeftPwm = correctionProfile
                ? kCorrectionTurnLeftMidPwm : kTurnLeftMidPwm;
            baseRightPwm = correctionProfile
                ? kCorrectionTurnRightMidPwm : kTurnRightMidPwm;
        }
        else
        {
            baseLeftPwm = correctionProfile
                ? kCorrectionTurnLeftSlowPwm : kTurnLeftSlowPwm;
            baseRightPwm = correctionProfile
                ? kCorrectionTurnRightSlowPwm : kTurnRightSlowPwm;
        }

        minimumPwm = kTurnMinPwm;
        maximumPwmValue = correctionProfile
            ? kCorrectionTurnMaxPwm : kTurnMaxPwm;
        syncKp = kTurnSyncKp;
        correctionLimit = correctionProfile
            ? kCorrectionTurnSyncCorrectionLimit
            : kTurnSyncCorrectionLimit;
    }

    const int32_t syncError = leftCount - rightCount;
    m_CumulativeSyncError = syncError;
    if (m_Mode == Mode::FORWARD)
    {
        const uint32_t syncElapsedMs = nowMs - m_LastSyncSampleMs;
        if (syncElapsedMs >= kForwardSyncSampleMs)
        {
            m_LeftIntervalDelta = leftCount - m_LastSyncLeft;
            m_RightIntervalDelta = rightCount - m_LastSyncRight;
            const int64_t intervalDifference =
                static_cast<int64_t>(m_LeftIntervalDelta)
                - static_cast<int64_t>(m_RightIntervalDelta);
            m_IntervalVelocityError = static_cast<int32_t>(std::lround(
                static_cast<double>(intervalDifference)
                * static_cast<double>(kForwardSyncSampleMs)
                / static_cast<double>(syncElapsedMs)));
            m_LastSyncLeft = leftCount;
            m_LastSyncRight = rightCount;
            m_LastSyncSampleMs = nowMs;
        }
    }
    else
    {
        m_LeftIntervalDelta = 0;
        m_RightIntervalDelta = 0;
        m_IntervalVelocityError = 0;
    }

    // PD only: cumulative position error plus the recent wheel-rate error.
    // There is deliberately no integral state that could wind up while PWM
    // is clamped during acceleration, deceleration, or per-wheel shutdown.
    float correctionRequest = syncError * syncKp;
    if (m_Mode == Mode::FORWARD)
        correctionRequest += m_IntervalVelocityError * kForwardSyncKd;
    const int correction = clampInt(
        static_cast<int>(std::lround(correctionRequest)),
        -correctionLimit,
        correctionLimit);
    m_SyncCorrection = correction;

    int leftPwm = baseLeftPwm - correction;
    int rightPwm = baseRightPwm + correction;
    leftPwm = leftCount >= m_LeftTargetCount
        ? 0
        : clampInt(leftPwm, minimumPwm, maximumPwmValue);
    rightPwm = rightCount >= m_RightTargetCount
        ? 0
        : clampInt(rightPwm, minimumPwm, maximumPwmValue);

    applyMotionOutputs(m_Mode, leftPwm, rightPwm);
    return UpdateResult::RUNNING;
}

MotionController::UpdateResult MotionController::updateSettling(uint32_t nowMs)
{
    // Settling is always de-energized. Reassert this invariant before reading
    // encoders or evaluating any completion condition.
    forceSafeOutputs();
    if (!outputsSafe())
        return latchFault(Fault::OUTPUT_INVARIANT, nowMs);

    int32_t rawLeftCount = 0;
    int32_t rawRightCount = 0;
    uint32_t activitySequence = 0;
    readEncoderState(rawLeftCount, rawRightCount, activitySequence);

    int32_t leftCount = 0;
    int32_t rightCount = 0;
    normalizeCounts(m_Mode,
                    rawLeftCount,
                    rawRightCount,
                    leftCount,
                    rightCount);

    if (leftCount < kWrongDirectionLimit || rightCount < kWrongDirectionLimit)
        return latchFault(Fault::WRONG_DIRECTION, nowMs);

    const int32_t overrunAllowance = countOverrunAllowance(m_Mode);
    if (leftCount > m_LeftTargetCount + overrunAllowance
        || rightCount > m_RightTargetCount + overrunAllowance)
    {
        return latchFault(Fault::COUNT_OVERRUN, nowMs);
    }

    int64_t countDifference = static_cast<int64_t>(leftCount)
                            - static_cast<int64_t>(rightCount);
    if (countDifference < 0)
        countDifference = -countDifference;
    if (countDifference > wheelMismatchLimit(m_Mode))
        return latchFault(Fault::WHEEL_MISMATCH, nowMs);

    const bool encoderChanged = activitySequence
                                    != m_SettlingLastActivitySequence
                             || leftCount != m_SettlingLastLeft
                             || rightCount != m_SettlingLastRight;
    if (encoderChanged)
    {
        m_SettlingLastLeft = leftCount;
        m_SettlingLastRight = rightCount;
        m_SettlingLastActivitySequence = activitySequence;
        m_SettlingStableSinceMs = nowMs;
    }

    // Timeout wins if a delayed update observes both timeout and stability.
    if (nowMs - m_SettlingStartedMs >= AppConfig::kEncoderSettleTimeoutMs)
        return latchFault(Fault::SETTLING_TIMEOUT, nowMs);

    if (leftCount >= m_LeftTargetCount
        && rightCount >= m_RightTargetCount
        && nowMs - m_SettlingStableSinceMs
               >= AppConfig::kEncoderSettleStableMs)
    {
        m_FinalElapsedMs = nowMs - m_MotionStartedMs;
        m_VelocityCountsPerSecond = 0.0f;
        m_State = State::COMPLETE;
        return UpdateResult::COMPLETE;
    }

    return UpdateResult::SETTLING;
}

void MotionController::stopImmediately()
{
    if (m_State == State::RUNNING || m_State == State::SETTLING)
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
    readEncoderSnapshot(result.rawLeftCount,
                        result.rawRightCount,
                        result.encoderResetEpoch);
    result.leftCount = result.rawLeftCount;
    result.rightCount = result.rawRightCount;
    result.mode = m_Mode;
    normalizeCounts(m_Mode,
                    result.rawLeftCount,
                    result.rawRightCount,
                    result.leftProgress,
                    result.rightProgress);
    result.targetCount = m_TargetCount;
    result.leftTargetCount = m_LeftTargetCount;
    result.rightTargetCount = m_RightTargetCount;
    result.leftIntervalDelta = m_LeftIntervalDelta;
    result.rightIntervalDelta = m_RightIntervalDelta;
    result.cumulativeSyncError = m_CumulativeSyncError;
    result.intervalVelocityError = m_IntervalVelocityError;
    if (m_HasSettlingBaseline)
    {
        result.leftCoastCount = result.leftProgress - m_SettlingStartLeft;
        result.rightCoastCount = result.rightProgress - m_SettlingStartRight;
    }
    result.syncCorrection = m_SyncCorrection;
    result.leftPwm = m_LeftPwm;
    result.rightPwm = m_RightPwm;
    result.velocityCountsPerSecond = m_VelocityCountsPerSecond;
    result.elapsedMs = (m_State == State::RUNNING
                        || m_State == State::SETTLING)
        ? millis() - m_MotionStartedMs
        : m_FinalElapsedMs;
    result.running = running();
    result.settling = settling();
    result.completed = completed();
    result.faultLatched = faultLatched();
    result.outputsSafe = outputsSafe();
    result.profile = m_Profile;
    result.fault = m_Fault;

    if (m_LeftTargetCount > 0 && m_RightTargetCount > 0)
    {
        int32_t safeLeft = result.leftProgress < 0 ? 0 : result.leftProgress;
        int32_t safeRight = result.rightProgress < 0 ? 0 : result.rightProgress;
        const float leftRatio = static_cast<float>(safeLeft)
                              / static_cast<float>(m_LeftTargetCount);
        const float rightRatio = static_cast<float>(safeRight)
                               / static_cast<float>(m_RightTargetCount);
        result.progress = clampUnit(leftRatio < rightRatio
                                        ? leftRatio : rightRatio);
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

bool MotionController::settling() const
{
    return m_State == State::SETTLING;
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

void MotionController::applyMotionOutputs(Mode mode, int leftPwm, int rightPwm)
{
    // No HIGH direction/STBY writes or non-zero PWM writes are reachable while
    // the compile-time output lock is false.
    if (!AppConfig::kEnableMotorOutputs)
    {
        forceSafeOutputs();
        return;
    }

    if (!isValidMode(mode))
    {
        forceSafeOutputs();
        return;
    }

    const int pwmLimit = maximumPwm(mode, m_Profile);
    leftPwm = clampInt(leftPwm, 0, pwmLimit);
    rightPwm = clampInt(rightPwm, 0, pwmLimit);

    switch (mode)
    {
        case Mode::FORWARD:
            digitalWrite(AppConfig::kLeftMotorIn1Pin, LOW);
            digitalWrite(AppConfig::kLeftMotorIn2Pin, HIGH);
            digitalWrite(AppConfig::kRightMotorIn1Pin, HIGH);
            digitalWrite(AppConfig::kRightMotorIn2Pin, LOW);
            break;

        case Mode::TURN_CW:
            // Left forward, right reverse rotates the chassis clockwise.
            digitalWrite(AppConfig::kLeftMotorIn1Pin, LOW);
            digitalWrite(AppConfig::kLeftMotorIn2Pin, HIGH);
            digitalWrite(AppConfig::kRightMotorIn1Pin, LOW);
            digitalWrite(AppConfig::kRightMotorIn2Pin, HIGH);
            break;

        case Mode::TURN_CCW:
            // Left reverse, right forward rotates the chassis counterclockwise.
            digitalWrite(AppConfig::kLeftMotorIn1Pin, HIGH);
            digitalWrite(AppConfig::kLeftMotorIn2Pin, LOW);
            digitalWrite(AppConfig::kRightMotorIn1Pin, HIGH);
            digitalWrite(AppConfig::kRightMotorIn2Pin, LOW);
            break;

        case Mode::NONE:
        default:
            forceSafeOutputs();
            return;
    }

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
    if (m_State == State::RUNNING || m_State == State::SETTLING)
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
