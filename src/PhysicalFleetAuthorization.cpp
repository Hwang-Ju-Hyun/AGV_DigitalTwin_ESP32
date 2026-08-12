#include "PhysicalFleetAuthorization.hpp"

void PhysicalFleetAuthorization::begin()
{
    m_State = State::WAIT_BOOT;
    m_CountdownStartedMs = 0;
}

PhysicalFleetAuthorization::BootResult
PhysicalFleetAuthorization::onBootPress(uint32_t nowMs)
{
    switch (m_State)
    {
    case State::WAIT_BOOT:
        m_CountdownStartedMs = nowMs;
        m_State = State::COUNTDOWN;
        return BootResult::COUNTDOWN_STARTED;

    case State::COUNTDOWN:
        m_CountdownStartedMs = 0;
        m_State = State::WAIT_BOOT;
        return BootResult::COUNTDOWN_CANCELLED;

    case State::ARMED:
        m_State = State::ESTOP_LATCHED;
        return BootResult::ESTOP_LATCHED;

    case State::ESTOP_LATCHED:
    default:
        return BootResult::IGNORED;
    }
}

bool PhysicalFleetAuthorization::update(uint32_t nowMs,
                                        uint32_t countdownMs)
{
    if (m_State != State::COUNTDOWN)
        return false;

    // Unsigned subtraction remains correct across millis() wraparound.
    if (nowMs - m_CountdownStartedMs < countdownMs)
        return false;

    m_CountdownStartedMs = 0;
    m_State = State::ARMED;
    return true;
}

uint32_t PhysicalFleetAuthorization::countdownRemainingMs(
    uint32_t nowMs,
    uint32_t countdownMs) const
{
    if (m_State != State::COUNTDOWN)
        return 0;

    const uint32_t elapsed = nowMs - m_CountdownStartedMs;
    return elapsed >= countdownMs ? 0 : countdownMs - elapsed;
}

const char* PhysicalFleetAuthorization::stateName(State state)
{
    switch (state)
    {
    case State::WAIT_BOOT:     return "WAIT_BOOT";
    case State::COUNTDOWN:     return "COUNTDOWN";
    case State::ARMED:         return "ARMED";
    case State::ESTOP_LATCHED: return "ESTOP_LATCHED";
    default:                   return "UNKNOWN";
    }
}

const char* PhysicalFleetAuthorization::bootResultName(BootResult result)
{
    switch (result)
    {
    case BootResult::COUNTDOWN_STARTED:   return "COUNTDOWN_STARTED";
    case BootResult::COUNTDOWN_CANCELLED: return "COUNTDOWN_CANCELLED";
    case BootResult::ESTOP_LATCHED:       return "ESTOP_LATCHED";
    case BootResult::IGNORED:             return "IGNORED";
    default:                              return "UNKNOWN";
    }
}
