#include "PhysicalFleetAuthorization.hpp"

#include <cassert>
#include <cstdint>

int main()
{
    constexpr uint32_t countdownMs = 5000;
    PhysicalFleetAuthorization gate;
    gate.begin();
    assert(!gate.networkAllowed());

    assert(gate.onBootPress(100) ==
           PhysicalFleetAuthorization::BootResult::COUNTDOWN_STARTED);
    assert(!gate.update(5099, countdownMs));
    assert(gate.update(5100, countdownMs));
    assert(gate.networkAllowed());
    assert(gate.onBootPress(5200) ==
           PhysicalFleetAuthorization::BootResult::ESTOP_LATCHED);
    assert(!gate.networkAllowed());

    gate.begin();
    assert(gate.onBootPress(10) ==
           PhysicalFleetAuthorization::BootResult::COUNTDOWN_STARTED);
    assert(gate.onBootPress(20) ==
           PhysicalFleetAuthorization::BootResult::COUNTDOWN_CANCELLED);
    assert(gate.state() == PhysicalFleetAuthorization::State::WAIT_BOOT);

    gate.begin();
    const uint32_t nearWrap = 0xFFFFFF00u;
    assert(gate.onBootPress(nearWrap) ==
           PhysicalFleetAuthorization::BootResult::COUNTDOWN_STARTED);
    assert(!gate.update(nearWrap + 4999u, countdownMs));
    assert(gate.update(nearWrap + 5000u, countdownMs));
    return 0;
}
