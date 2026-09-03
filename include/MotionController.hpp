#pragma once

#include <Arduino.h>
#include <cstdint>

class MotionController
{
public:
    enum class Mode : uint8_t
    {
        NONE,
        FORWARD,
        TURN_CW,
        TURN_CCW
    };

    enum class Profile : uint8_t
    {
        NORMAL,
        PHYSICAL_FLEET,
        CORRECTION
    };

    enum class StartResult : uint8_t
    {
        STARTED,
        OUTPUT_DISABLED,
        INVALID_TARGET,
        INVALID_MODE,
        NOT_READY,
        ALREADY_RUNNING,
        FAULT_LATCHED
    };

    enum class UpdateResult : uint8_t
    {
        IDLE,
        RUNNING,
        SETTLING,
        COMPLETE,
        FAULTED
    };

    enum class Fault : uint8_t
    {
        NONE,
        WRONG_DIRECTION,
        COUNT_OVERRUN,
        WHEEL_MISMATCH,
        TIMEOUT,
        STALL,
        EXTERNAL_STOP,
        OUTPUT_INVARIANT,
        SETTLING_TIMEOUT
    };

    struct Snapshot
    {
        // leftCount/rightCount remain raw-count aliases for existing odometry
        // callers. New motion code should use the explicitly named fields.
        int32_t leftCount = 0;
        int32_t rightCount = 0;
        int32_t rawLeftCount = 0;
        int32_t rawRightCount = 0;
        int32_t leftProgress = 0;
        int32_t rightProgress = 0;
        int32_t targetCount = 0;
        int32_t leftTargetCount = 0;
        int32_t rightTargetCount = 0;
        int32_t leftIntervalDelta = 0;
        int32_t rightIntervalDelta = 0;
        int32_t cumulativeSyncError = 0;
        int32_t intervalVelocityError = 0;
        int syncCorrection = 0;
        int leftPwm = 0;
        int rightPwm = 0;
        float progress = 0.0f;
        float velocityCountsPerSecond = 0.0f;
        uint32_t elapsedMs = 0;
        uint32_t encoderResetEpoch = 0;
        bool running = false;
        bool settling = false;
        bool completed = false;
        bool faultLatched = false;
        bool outputsSafe = false;
        Mode mode = Mode::NONE;
        Profile profile = Profile::NORMAL;
        Fault fault = Fault::NONE;
    };

    // Configures the verified TB6612 and encoder pins, then forces every motor
    // output low. Calling begin() again does not clear a latched fault.
    void begin();

    StartResult startForward(int32_t targetCount);
    StartResult startForward(int32_t targetCount, uint32_t nowMs);
    StartResult startMotion(Mode mode, int32_t targetCount);
    StartResult startMotion(Mode mode, int32_t targetCount, uint32_t nowMs);
    StartResult startMotion(Mode mode,
                            int32_t targetCount,
                            Profile profile);
    StartResult startMotion(Mode mode,
                            int32_t targetCount,
                            uint32_t nowMs,
                            Profile profile);
    UpdateResult update(uint32_t nowMs);

    // Intentional local/server cancellation removes power immediately without
    // converting that request into a fault. Detected faults and emergencyStop()
    // remain latched until reboot.
    void stopImmediately();
    void emergencyStop(Fault cause = Fault::EXTERNAL_STOP);

    Snapshot snapshot() const;
    bool outputsSafe() const;
    bool running() const;
    bool settling() const;
    bool completed() const;
    bool faultLatched() const;
    Fault fault() const;

private:
    enum class State : uint8_t
    {
        NOT_READY,
        IDLE,
        RUNNING,
        SETTLING,
        COMPLETE,
        FAULTED
    };

    static void IRAM_ATTR leftEncoderISR();
    static void IRAM_ATTR rightEncoderISR();
    static void readEncoderCounts(int32_t& left, int32_t& right);
    static void readEncoderState(int32_t& left,
                                 int32_t& right,
                                 uint32_t& activitySequence);
    static void readEncoderSnapshot(int32_t& left,
                                    int32_t& right,
                                    uint32_t& resetEpoch);
    static void resetEncoderCounts();
    static void normalizeCounts(Mode mode,
                                int32_t rawLeft,
                                int32_t rawRight,
                                int32_t& leftProgress,
                                int32_t& rightProgress);

    void forceSafeOutputs();
    void applyMotionOutputs(Mode mode, int leftPwm, int rightPwm);
    UpdateResult updateSettling(uint32_t nowMs);
    UpdateResult latchFault(Fault cause, uint32_t nowMs);
    void updateVelocity(int32_t leftCount, int32_t rightCount, uint32_t nowMs);

    static volatile int32_t s_LeftCount;
    static volatile int32_t s_RightCount;
    static volatile uint32_t s_EncoderActivitySequence;
    static volatile uint32_t s_EncoderResetEpoch;
    static portMUX_TYPE s_EncoderMux;

    State m_State = State::NOT_READY;
    Fault m_Fault = Fault::NONE;
    Mode m_Mode = Mode::NONE;
    Profile m_Profile = Profile::NORMAL;
    bool m_Initialized = false;
    int32_t m_TargetCount = 0;
    int32_t m_LeftTargetCount = 0;
    int32_t m_RightTargetCount = 0;
    int m_LeftPwm = 0;
    int m_RightPwm = 0;
    uint32_t m_MotionStartedMs = 0;
    uint32_t m_FinalElapsedMs = 0;
    uint32_t m_SettlingStartedMs = 0;
    uint32_t m_SettlingStableSinceMs = 0;
    uint32_t m_LastProgressCheckMs = 0;
    uint32_t m_LastVelocitySampleMs = 0;
    uint32_t m_LastSyncSampleMs = 0;
    int32_t m_LastProgressLeft = 0;
    int32_t m_LastProgressRight = 0;
    int32_t m_LastVelocityAverage = 0;
    int32_t m_LastSyncLeft = 0;
    int32_t m_LastSyncRight = 0;
    int32_t m_LeftIntervalDelta = 0;
    int32_t m_RightIntervalDelta = 0;
    int32_t m_CumulativeSyncError = 0;
    int32_t m_IntervalVelocityError = 0;
    int m_SyncCorrection = 0;
    int32_t m_SettlingLastLeft = 0;
    int32_t m_SettlingLastRight = 0;
    uint32_t m_SettlingLastActivitySequence = 0;
    uint8_t m_LeftNoProgressWindows = 0;
    uint8_t m_RightNoProgressWindows = 0;
    float m_VelocityCountsPerSecond = 0.0f;
};
