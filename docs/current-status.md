# ESP32Test Current Status

Last reviewed: 2026-08-07

## Repository baseline

- This directory is the existing PlatformIO project for the physical ESP32 AGV.
- The tracked baseline at the start of this integration task was commit `640891d990a9e786f162932597b50c439cc54335` on `master`.
- This status records the motor-disabled Phase 2B integration built and hardware-checked on 2026-08-07.
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

## Firmware currently built

The active PlatformIO firmware is now split into these responsibilities:

- `src/RobotProtocol.cpp`: RobotProtocol v1 field-by-field serialization.
- `src/RobotClient.cpp`: Wi-Fi/TCP session, HELLO/ACK, packet handling, STATUS, ARRIVED, ERROR, and PING/PONG.
- `src/MotionController.cpp`: encoder interrupts, the verified 30 cm forward profile, synchronization, and immediate motion safety.
- `src/RouteExecutor.cpp`: exact-route validation, BOOT/countdown gating, motion state, fault latch, progress STATUS, and ARRIVED gating.
- `src/main.cpp`: callback wiring and a non-blocking safety/network loop; it does not duplicate the verified motor algorithm.

The only accepted physical-demo interpretation remains:

```text
Server route [node 1 -> node 2]
    -> one local forward segment
    -> 520 encoder counts per wheel
    -> approximately 30 cm on the verified chassis
```

This temporary mapping is not a general conversion from arbitrary Server map routes to physical distance.

## Safety state and invariants

- `AppConfig::kEnableMotorOutputs` remains `false`.
- `src/main.cpp` retains a `static_assert` that intentionally fails the build if that lock is changed without revisiting the integration build.
- With the lock false, the start path returns `OUTPUT_DISABLED` before any HIGH direction/STBY command or nonzero PWM command is reachable.
- Motor hardware is initialized with `STBY=LOW`, PWM zero, and all direction pins low before Serial or Wi-Fi starts.
- The current build reaches `OUTPUT_LOCKED` after a valid route, BOOT approval, and countdown. It does not move and does not send `ARRIVED`.
- A future enabled run can enter motion only after an accepted exact route, a live accepted session, one local BOOT press, and the five-second countdown.
- Wrong-direction counts, count overrun, left/right mismatch, timeout, stall, output-invariant failure, and emergency stop force immediate safe outputs and latch a fault until reboot.
- A detected TCP loss invokes the RouteExecutor stop path before the RobotClient closes or clears the socket.
- `ARRIVED` is eligible only after both wheels reach the target and PWM zero plus `STBY=LOW` have been verified.

## Network robustness

- A newly connected socket explicitly enters `WAIT_HELLO_ACK`.
- Missing `HELLO_ACK` for 3,000 ms closes the stale TCP socket and starts a fresh connection through the existing reconnect interval.
- A failed or partial packet write also closes the stream rather than retrying a partial frame.
- Each application loop reads at most 512 TCP bytes and processes at most four complete frames so a burst cannot indefinitely starve encoder and safety checks.
- The SSID is no longer printed in the Serial connection log.
- The exact `[1 -> 2]` whitelist and duplicate-route protection remain in force.

## Encoder progress STATUS

- During a future enabled run, `STATUS.progress` is derived from the slower wheel's count divided by 520 and is clamped to `0.0` through `1.0`.
- While moving, `currentNodeID` remains node 1 and `currentLinkID` carries target node 2, matching Server reference commit `ee3244f39253da23b9d480f775d398316fb46696`.
- After encoder completion, the prepared final STATUS reports node 2 and progress `1.0` before `ARRIVED` is attempted.
- Metric velocity, measured battery percentage, full `(x,z,heading)` odometry, turns, and arbitrary route expansion are not implemented in this phase.

## Preserved physical-vehicle reference

- `AGV_Project_Record/final_l_route_main.cpp` remains unchanged and outside the active `src/` build.
- Its physically verified wiring is left motor `25/26/PWM27`, right motor `33/32/PWM14`, `STBY=13`, left encoder `19/18`, right encoder `17/16`, and BOOT `0`.
- Its verified values include 260 counts per wheel revolution, 520 counts for approximately 30 cm, and 176 counts per wheel for the calibrated 90-degree turn.
- The forward ISR polarity, acceleration/deceleration values, PWM baseline, and count synchronization were transferred into `MotionController`; the reference file was not moved or rewritten.

## Build validation

- `platformio run` for `esp32dev` succeeded on 2026-08-07.
- Reported use was 46,336 bytes of RAM (14.1%) and 773,525 bytes of flash (59.0%).
- The assistant performed build-only validation; the user subsequently uploaded and monitored the motor-locked binary with motor power isolated.
- No motor output activation or powered movement was performed.

## Explicitly not yet verified

- The 3-second stale-HELLO reconnect behavior on the live Windows/WSL path
- Encoder progress STATUS received and visualized by Server/Unity
- Powered 30 cm motion from a Server `[1 -> 2]` command
- Every fault input on raised wheels, including stall, reversed count, mismatch, timeout, TCP loss, and E-stop
- The final policy for detecting an accepted but half-open TCP session; the Server currently responds to PING but does not provide a periodic Server-silence guarantee
- Exactly-once Server processing of the safe-completion `ARRIVED`
- Metric velocity, battery sensing, odometry, turns, or general multi-node execution

## Next safe step

Keep motor outputs locked. Before any physical activation, define the accepted-session half-open TCP policy and prepare a separately reviewed raised-wheel test for disconnect, E-stop, stall, reversed count, mismatch, timeout, safe completion, and exactly-once `ARRIVED`. Changing the compile lock remains a later, explicit approval step.
