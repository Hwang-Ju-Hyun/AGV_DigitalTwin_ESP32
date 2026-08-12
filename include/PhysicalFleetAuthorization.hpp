#pragma once

#include <cstdint>

class PhysicalFleetAuthorization
{
public:
    enum class State : uint8_t
    {
        WAIT_BOOT,
        COUNTDOWN,
        ARMED,
        ESTOP_LATCHED
    };

    enum class BootResult : uint8_t
    {
        COUNTDOWN_STARTED,
        COUNTDOWN_CANCELLED,
        ESTOP_LATCHED,
        IGNORED
    };

    void begin();
    BootResult onBootPress(uint32_t nowMs);
    bool update(uint32_t nowMs, uint32_t countdownMs);

    State state() const { return m_State; }
    bool networkAllowed() const { return m_State == State::ARMED; }
    uint32_t countdownRemainingMs(uint32_t nowMs,
                                  uint32_t countdownMs) const;

    static const char* stateName(State state);
    static const char* bootResultName(BootResult result);

private:
    State m_State = State::WAIT_BOOT;
    uint32_t m_CountdownStartedMs = 0;
};
