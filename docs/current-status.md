# ESP32Test Current Status

Last reviewed: 2026-08-08

## Repository baseline

- This directory is the existing PlatformIO project for the physical ESP32 AGV.
- The M4 safety-hardening work started from commit `1b3b80f924ce99e1fe3f9d9de19884e92be7e0fb` on branch `agent/phase2b-motor-locked-integration`.
- Phase 2C raised-wheel preparation starts from commit `ef2d4f2d21e2768b06a63013c079de4e3a5dafb8` on branch `agent/phase2c-raised-wheel-test`.
- This status records the previously hardware-checked motor-disabled Phase 2B path plus the build-only M4 and Phase 2C changes.
- PlatformIO output, temporary build directories, IDE-local files, and the real credential file remain excluded by `.gitignore`.
- `include/Secrets.hpp` is local-only and must never be displayed, staged, committed, or pushed.

## Confirmed Phase 2A result

The user physically verified the motor-disabled Phase 2A path before this code-integration task:

- Wi-Fi/TCP and `HELLO_ACK` succeeded.
- The Server's exact `[1 -> 2]` route was received and stored.
- Local BOOT approval and the five-second countdown succeeded.
- PWM stayed zero, TB6612 `STBY` stayed low, and `ARRIVED` stayed blocked.

This confirms the communication/approval dry run only. It is not evidence of Server-commanded physical motion.

## Confirmed Phase 2B motor-locked result

The user uploaded the integrated binary with motor power isolated and repeated the exact-route approval test:

- The accepted route reached `WAIT_BOOT` with both encoder counts at zero, PWM `0/0`, and `STBY=LOW`.
- One BOOT press entered `COUNTDOWN`, and the five-second countdown completed normally.
- The executor transitioned to `OUTPUT_LOCKED` with `safe=1`, encoder counts still zero, PWM `0/0`, and `STBY=LOW`.
- `STATUS` remained at node 1 with progress `0.000`, no wheel movement occurred, and `ARRIVED` remained blocked.

This verifies the integrated state machine and compile-locked output path on the ESP32. It still does not verify powered motion, encoder progress in motion, or completion reporting.

## M4 pre-activation hardening (build-only)

The active source now contains these additional safety gates, but this M4 build has not been uploaded or physically tested:

- BOOT in `WAIT_BOOT` starts the existing five-second countdown.
- A second BOOT press during `COUNTDOWN` forces safe outputs, returns to `WAIT_BOOT`, and retains the same stored route for later approval.
- BOOT in `RUNNING`, `SETTLING`, or `ARRIVAL_PENDING` forces PWM zero and `STBY=LOW` before latching `ESTOP_LATCHED`; these states no longer ignore the button.
- On the control-loop sample that first observes both encoders at or above 520 counts, drive power is removed and the executor enters a non-blocking `SETTLING` state.
- Every counted A-channel rising event increments an activity sequence, so even net-zero count motion restarts the stability timer.
- Completion requires both counts to remain at or above target with no encoder activity for at least 150 ms. A 2,000 ms settling limit, wrong direction, count overrun, wheel mismatch, or unsafe output invariant latches a fault.
- `STATUS` remains on node 1 during `SETTLING`; node 2 and `ARRIVED` remain blocked until settling completes and safe outputs are verified.

## Phase 2C raised-wheel build preparation (build-only)

PlatformIO now exposes two explicit compile-time safety profiles:

- `esp32dev` is the default environment. Both the raised-wheel flag and motor-output flag are `0`, so motor output remains compile-locked off.
- `esp32dev-raised-wheel` is the only environment where both flags are `1`, allowing the existing guarded motion path to energize the TB6612 after route acceptance, local BOOT approval, and the five-second countdown.
- Missing, non-binary, or mismatched profile flags stop compilation. Profile-specific `static_assert` checks independently verify the locked and raised-wheel builds.
- The raised-wheel boot banner explicitly reports `MOTOR OUTPUTS: ENABLED` and warns that the wheels must remain off the floor. The default banner explicitly reports the motor lock.
- MotionController, RouteExecutor, RobotProtocol, GPIO, encoder polarity, 520-count target, PWM profile, BOOT behavior, fault gates, settling, and ARRIVED gating are unchanged from `ef2d4f2`.

Both profiles have only been compiled. Neither Phase 2C binary has been uploaded or run on hardware.

## Firmware currently built

The active PlatformIO firmware is now split into these responsibilities:

- `src/RobotProtocol.cpp`: RobotProtocol v1 field-by-field serialization.
- `src/RobotClient.cpp`: Wi-Fi/TCP session, HELLO/ACK, packet handling, STATUS, ARRIVED, ERROR, and PING/PONG.
- `src/MotionController.cpp`: encoder interrupts, the verified 30 cm forward profile, synchronization, immediate motion safety, and encoder-stability settling.
- `src/RouteExecutor.cpp`: exact-route validation, BOOT/countdown/E-stop gating, running/settling state, fault latch, progress STATUS, and ARRIVED gating.
- `src/main.cpp`: callback wiring and a non-blocking safety/network loop; it does not duplicate the verified motor algorithm.
- `platformio.ini`: selects the default motor-locked build or the explicit raised-wheel-only build profile.

The only accepted physical-demo interpretation remains:

```text
Server route [node 1 -> node 2]
    -> one local forward segment
    -> 520 encoder counts per wheel
    -> approximately 30 cm on the verified chassis
```

This temporary mapping is not a general conversion from arbitrary Server map routes to physical distance.

## Safety state and invariants

- The default `esp32dev` environment compiles `AppConfig::kEnableMotorOutputs=false` and remains the safe target for a bare `platformio run`.
- Only `esp32dev-raised-wheel` compiles `AppConfig::kEnableMotorOutputs=true`; it is clearly marked as raised-wheel-only and is not a floor-driving profile.
- `src/main.cpp` retains compile-time assertions for both profiles, and `Config.hpp` rejects missing, invalid, or mismatched profile flags.
- In the default environment, the start path returns `OUTPUT_DISABLED` before any HIGH direction/STBY command or nonzero PWM command is reachable.
- Motor hardware is initialized with `STBY=LOW`, PWM zero, and all direction pins low before Serial or Wi-Fi starts.
- The default build reaches `OUTPUT_LOCKED` after a valid route, BOOT approval, and countdown. It does not move and does not send `ARRIVED`.
- The raised-wheel build can enter motion only after an accepted exact route, a live accepted session, one local BOOT press, and the five-second countdown.
- Countdown cancellation preserves the accepted route and returns to `WAIT_BOOT`; Server `CANCEL_ROUTE` remains the operation that removes the stored route.
- BOOT during `RUNNING`, `SETTLING`, or `ARRIVAL_PENDING` is a local emergency stop and latches until reboot.
- This BOOT action is a debounced firmware stop, not a hardware-rated emergency-stop circuit; it does not replace a physical power-cut switch.
- Wrong-direction counts, count overrun, left/right mismatch, timeout, stall, output-invariant failure, and emergency stop force immediate safe outputs and latch a fault until reboot.
- A detected TCP loss invokes the RouteExecutor stop path before the RobotClient closes or clears the socket.
- The first control-loop observation of both 520-count targets removes drive power. Completion then requires at least 150 ms of encoder stability, with a 2,000 ms maximum settling window.
- `ARRIVED` is eligible only after settling completes and PWM zero plus `STBY=LOW` have been verified.

## Network robustness

- A newly connected socket explicitly enters `WAIT_HELLO_ACK`.
- Missing `HELLO_ACK` for 3,000 ms closes the stale TCP socket and starts a fresh connection through the existing reconnect interval.
- A failed or partial packet write also closes the stream rather than retrying a partial frame.
- Each application loop reads at most 512 TCP bytes and processes at most four complete frames so a burst cannot indefinitely starve encoder and safety checks.
- The SSID is no longer printed in the Serial connection log.
- The exact `[1 -> 2]` whitelist and duplicate-route protection remain in force.
- M4 deliberately adds no outbound periodic PING, PONG lease, heartbeat timeout, or Server-silence timeout. A quiet accepted Server session alone does not stop motion.
- Accepted-but-half-open TCP detection remains a future design item. Clear Wi-Fi/TCP disconnects still use the existing immediate-stop path.

## Encoder progress STATUS

- During a future enabled run, `STATUS.progress` is derived from the slower wheel's count divided by 520 and is clamped to `0.0` through `1.0`.
- While moving, `currentNodeID` remains node 1 and `currentLinkID` carries target node 2, matching Server reference commit `ee3244f39253da23b9d480f775d398316fb46696`.
- During non-blocking settling, `currentNodeID` remains node 1, target node 2 remains identified, and encoder progress may remain `1.0` while outputs are already safe.
- Only after the 150 ms stability gate completes does the prepared final STATUS report node 2 and progress `1.0` before `ARRIVED` is attempted.
- Metric velocity, measured battery percentage, full `(x,z,heading)` odometry, turns, and arbitrary route expansion are not implemented in this phase.

## Preserved physical-vehicle reference

- `AGV_Project_Record/final_l_route_main.cpp` remains unchanged and outside the active `src/` build.
- Its physically verified wiring is left motor `25/26/PWM27`, right motor `33/32/PWM14`, `STBY=13`, left encoder `19/18`, right encoder `17/16`, and BOOT `0`.
- Its verified values include 260 counts per wheel revolution, 520 counts for approximately 30 cm, and 176 counts per wheel for the calibrated 90-degree turn.
- The forward ISR polarity, acceleration/deceleration values, PWM baseline, and count synchronization were transferred into `MotionController`; the reference file was not moved or rewritten.

## Build validation

- `platformio run -e esp32dev` succeeded on 2026-08-08.
  Reported use was 46,360 bytes of RAM (14.1%) and 774,445 bytes of flash (59.1%).
- `platformio run -e esp32dev-raised-wheel` succeeded on 2026-08-08.
  Reported use was 46,360 bytes of RAM (14.1%) and 775,345 bytes of flash (59.2%).
- Phase 2C validation was build-only. No upload, Serial monitor, battery connection, motor activation, or powered movement was performed.
- The earlier motor-locked hardware check described above remains the latest physical firmware verification.

## Explicitly not yet verified

- The 3-second stale-HELLO reconnect behavior on the live Windows/WSL path
- Explicit socket cleanup after a failed `WiFiClient.connect()` attempt; this path is unchanged in the reduced M4 scope
- M4 countdown-retain, BOOT E-stop, settling stability, and settling-timeout behavior on actual ESP32 hardware
- The `esp32dev-raised-wheel` boot banner and actual 520-count raised-wheel motion path on hardware
- Encoder progress STATUS received and visualized by Server/Unity
- Powered 30 cm motion from a Server `[1 -> 2]` command
- Every fault input on raised wheels, including stall, reversed count, mismatch, timeout, TCP loss, and E-stop
- The future policy for detecting an accepted but half-open TCP session; M4 intentionally does not treat Server silence as a fault
- Exactly-once Server processing of the safe-completion `ARRIVED`
- Metric velocity, battery sensing, odometry, turns, or general multi-node execution

## Next safe step

Keep using the default motor-locked environment unless a raised-wheel test is explicitly approved. Before uploading `esp32dev-raised-wheel`, confirm the wheels are mechanically clear of the floor, the vehicle cannot fall or drive away, the power wiring is secure, and a physical power-disconnect is immediately reachable. Upload and powered testing remain separate user-controlled steps. Half-open session policy remains a later decision.
