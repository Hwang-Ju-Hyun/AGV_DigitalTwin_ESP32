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

    void testForwardRightTargetEndsFivePercentEarly()
    {
        resetArduino();
        MotionController motion;
        motion.begin();
        assert(motion.outputsSafe());
        assert(motion.startMotion(
                   MotionController::Mode::FORWARD,
                   200,
                   10,
                   MotionController::Profile::PHYSICAL_FLEET)
               == MotionController::StartResult::STARTED);

        auto snapshot = motion.snapshot();
        assert(snapshot.leftTargetCount == 205);
        assert(snapshot.rightTargetCount == 195);
        assert(snapshot.profile
               == MotionController::Profile::PHYSICAL_FLEET);
        assert(snapshot.leftPwm == 50);
        assert(snapshot.rightPwm == 55);

        pulseForward(195, 195);
        assert(motion.update(20) == MotionController::UpdateResult::RUNNING);
        snapshot = motion.snapshot();
        assert(snapshot.leftPwm > 0);
        assert(snapshot.rightPwm == 0);

        pulseForward(10, 0);
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
}

int main()
{
    static_assert(AppConfig::kTurn90CwCount == 163,
                  "CW calibration contract changed");
    static_assert(AppConfig::kTurn90CcwCount == 159,
                  "CCW calibration contract changed");
    static_assert(AppConfig::kForwardLeftTargetScale == 1.025f,
                  "Forward left trim contract changed");
    static_assert(AppConfig::kForwardRightTargetScale == 0.975f,
                  "Forward right trim contract changed");

    testForwardRightTargetEndsFivePercentEarly();
    testLegacyForwardTargetRemainsUnchanged();
    testCorrectionProfilesStartSlower();
    testWheelMismatchStillLatchesSafeOutputs();
    return 0;
}
