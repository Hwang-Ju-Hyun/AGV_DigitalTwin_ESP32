#include "MotionController.hpp"
#include "Config.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iterator>

namespace
{
    void resetArduino()
    {
        std::fill(std::begin(g_ArduinoPinState),
                  std::end(g_ArduinoPinState),
                  LOW);
        std::fill(std::begin(g_ArduinoPwmDuty),
                  std::end(g_ArduinoPwmDuty),
                  0);
        std::fill(std::begin(g_ArduinoInterruptHandler),
                  std::end(g_ArduinoInterruptHandler),
                  nullptr);
        g_ArduinoMillis = 0;
    }

    void pulseForward(int leftPulses, int rightPulses)
    {
        digitalWrite(AppConfig::kLeftEncoderBPin, LOW);
        digitalWrite(AppConfig::kRightEncoderBPin, HIGH);
        for (int i = 0; i < leftPulses; ++i)
            g_ArduinoInterruptHandler[AppConfig::kLeftEncoderAPin]();
        for (int i = 0; i < rightPulses; ++i)
            g_ArduinoInterruptHandler[AppConfig::kRightEncoderAPin]();
    }

    void testPhysicalFleetForwardTargetsRemainEqual()
    {
        // round(350 mm * 520 counts / 300 mm)
        constexpr int32_t kNominal350MmTarget = 607;
        resetArduino();
        MotionController motion;
        motion.begin();
        assert(motion.outputsSafe());
        assert(motion.startMotion(
                   MotionController::Mode::FORWARD,
                   kNominal350MmTarget,
                   10,
                   MotionController::Profile::PHYSICAL_FLEET)
               == MotionController::StartResult::STARTED);

        auto snapshot = motion.snapshot();
        assert(snapshot.leftTargetCount == kNominal350MmTarget);
        assert(snapshot.rightTargetCount == kNominal350MmTarget);
        assert(snapshot.profile
               == MotionController::Profile::PHYSICAL_FLEET);
        assert(snapshot.leftPwm == 50);
        assert(snapshot.rightPwm == 55);

        pulseForward(602, 602);
        assert(motion.update(20) == MotionController::UpdateResult::RUNNING);
        snapshot = motion.snapshot();
        assert(snapshot.leftPwm > 0);
        assert(snapshot.rightPwm > 0);

        pulseForward(5, 5);
        assert(motion.update(30) == MotionController::UpdateResult::SETTLING);
        assert(motion.outputsSafe());
    }

    void testLegacyForwardTargetRemainsUnchanged()
    {
        resetArduino();
        MotionController motion;
        motion.begin();
        assert(motion.startForward(100, 10)
               == MotionController::StartResult::STARTED);
        const auto snapshot = motion.snapshot();
        assert(snapshot.leftTargetCount == 100);
        assert(snapshot.rightTargetCount == 100);
        assert(snapshot.profile == MotionController::Profile::NORMAL);
        motion.stopImmediately();
        assert(motion.outputsSafe());
    }

    void testCorrectionProfilesStartSlower()
    {
        resetArduino();
        MotionController drive;
        drive.begin();
        assert(drive.startMotion(MotionController::Mode::FORWARD,
                                 100,
                                 10,
                                 MotionController::Profile::CORRECTION)
               == MotionController::StartResult::STARTED);
        auto snapshot = drive.snapshot();
        assert(snapshot.profile == MotionController::Profile::CORRECTION);
        assert(snapshot.leftTargetCount == 100);
        assert(snapshot.rightTargetCount == 100);
        assert(snapshot.leftPwm == 46);
        assert(snapshot.rightPwm == 51);
        drive.stopImmediately();
        assert(drive.outputsSafe());

        resetArduino();
        MotionController turn;
        turn.begin();
        assert(turn.startMotion(MotionController::Mode::TURN_CW,
                                20,
                                10,
                                MotionController::Profile::CORRECTION)
               == MotionController::StartResult::STARTED);
        snapshot = turn.snapshot();
        assert(snapshot.leftTargetCount == 20);
        assert(snapshot.rightTargetCount == 20);
        assert(snapshot.leftPwm == 44);
        assert(snapshot.rightPwm == 49);
        turn.stopImmediately();
        assert(turn.outputsSafe());
    }

    void testWheelMismatchStillLatchesSafeOutputs()
    {
        resetArduino();
        MotionController motion;
        motion.begin();
        assert(motion.startMotion(MotionController::Mode::FORWARD,
                                  200,
                                  10,
                                  MotionController::Profile::CORRECTION)
               == MotionController::StartResult::STARTED);
        pulseForward(81, 0);
        assert(motion.update(20) == MotionController::UpdateResult::FAULTED);
        assert(motion.fault() == MotionController::Fault::WHEEL_MISMATCH);
        assert(motion.outputsSafe());
    }

    void testForwardSyncUsesBoundedCumulativeAndIntervalError()
    {
        resetArduino();
        MotionController motion;
        motion.begin();
        assert(motion.startMotion(MotionController::Mode::FORWARD,
                                  300,
                                  10,
                                  MotionController::Profile::PHYSICAL_FLEET)
               == MotionController::StartResult::STARTED);

        // Right is faster during the first 50 ms window. The controller must
        // increase left effort and reduce right effort before the final count
        // target, using both cumulative and interval error.
        pulseForward(10, 20);
        assert(motion.update(60) == MotionController::UpdateResult::RUNNING);
        auto snapshot = motion.snapshot();
        assert(snapshot.leftIntervalDelta == 10);
        assert(snapshot.rightIntervalDelta == 20);
        assert(snapshot.cumulativeSyncError == -10);
        assert(snapshot.intervalVelocityError == -10);
        assert(snapshot.syncCorrection == -8);
        assert(snapshot.leftPwm > snapshot.rightPwm);

        // A large but still non-faulting difference is bounded by the
        // existing physical-fleet correction limit; outputs remain forward.
        pulseForward(50, 0);
        assert(motion.update(110) == MotionController::UpdateResult::RUNNING);
        snapshot = motion.snapshot();
        assert(snapshot.syncCorrection == 15);
        assert(snapshot.leftPwm >= 42);
        assert(snapshot.rightPwm >= 42);
        assert(snapshot.leftPwm <= 100);
        assert(snapshot.rightPwm <= 100);
        assert(!motion.faultLatched());

        // No correction memory may leak into the next primitive.
        motion.stopImmediately();
        assert(motion.startMotion(MotionController::Mode::FORWARD,
                                  100,
                                  120,
                                  MotionController::Profile::CORRECTION)
               == MotionController::StartResult::STARTED);
        snapshot = motion.snapshot();
        assert(snapshot.leftIntervalDelta == 0);
        assert(snapshot.rightIntervalDelta == 0);
        assert(snapshot.cumulativeSyncError == 0);
        assert(snapshot.intervalVelocityError == 0);
        assert(snapshot.syncCorrection == 0);

        pulseForward(40, 0);
        assert(motion.update(170) == MotionController::UpdateResult::RUNNING);
        snapshot = motion.snapshot();
        assert(snapshot.syncCorrection == 10);
        assert(snapshot.leftPwm >= 42);
        assert(snapshot.rightPwm >= 42);
        assert(snapshot.leftPwm <= 75);
        assert(snapshot.rightPwm <= 75);
    }
}

int main()
{
    static_assert(AppConfig::kTurn90CwCount == 163,
                  "CW calibration contract changed");
    static_assert(AppConfig::kTurn90CcwCount == 159,
                  "CCW calibration contract changed");
    static_assert(AppConfig::kForwardLeftTargetScale == 1.0f,
                  "Forward left target must remain nominal");
    static_assert(AppConfig::kForwardRightTargetScale == 1.0f,
                  "Forward right target must remain nominal");

    testPhysicalFleetForwardTargetsRemainEqual();
    testLegacyForwardTargetRemainsUnchanged();
    testCorrectionProfilesStartSlower();
    testWheelMismatchStillLatchesSafeOutputs();
    testForwardSyncUsesBoundedCumulativeAndIntervalError();
    return 0;
}
