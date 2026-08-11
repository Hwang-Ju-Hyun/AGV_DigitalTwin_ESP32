# ESP32Test Current Status

Last reviewed: 2026-08-11

## Repository baseline

- This directory is the existing PlatformIO project for the physical ESP32 AGV.
- The M4 safety-hardening work started from commit `1b3b80f924ce99e1fe3f9d9de19884e92be7e0fb` on branch `agent/phase2b-motor-locked-integration`.
- Phase 2C raised-wheel preparation starts from commit `ef2d4f2d21e2768b06a63013c079de4e3a5dafb8` on branch `agent/phase2c-raised-wheel-test`.
- Phase 2D trajectory/odometry preview work starts from commit `efc9e191d9a8ed3bdeba502ee6d8ba8cacce7a01` on branch `agent/phase2d-trajectory-odometry`.
- This status records the hardware-checked motor-disabled Phase 2B path, the M4 safety gates, and the first successful Phase 2C raised-wheel Server-to-physical-AGV-to-Unity run.
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

That earlier Phase 2B check verified the integrated state machine and compile-locked output path on the ESP32. Powered motion, encoder progress, and normal completion were subsequently exercised only in the controlled Phase 2C raised-wheel run documented below.

## M4 pre-activation hardening

The active source contains these additional safety gates. The normal target/settling/completion path was exercised during the Phase 2C raised-wheel run documented below. Countdown cancellation, BOOT E-stop, and injected fault paths remain separately unverified:

- BOOT in `WAIT_BOOT` starts the existing five-second countdown.
- A second BOOT press during `COUNTDOWN` forces safe outputs, returns to `WAIT_BOOT`, and retains the same stored route for later approval.
- BOOT in `RUNNING`, `SETTLING`, or `ARRIVAL_PENDING` forces PWM zero and `STBY=LOW` before latching `ESTOP_LATCHED`; these states no longer ignore the button.
- On the control-loop sample that first observes both encoders at or above 520 counts, drive power is removed and the executor enters a non-blocking `SETTLING` state.
- Every counted A-channel rising event increments an activity sequence, so even net-zero count motion restarts the stability timer.
- Completion requires both counts to remain at or above target with no encoder activity for at least 150 ms. A 2,000 ms settling limit, wrong direction, count overrun, wheel mismatch, or unsafe output invariant latches a fault.
- `STATUS` remains on node 1 during `SETTLING`; node 2 and `ARRIVED` remain blocked until settling completes and safe outputs are verified.

## Phase 2C/2D/2E build profiles

PlatformIO exposes four explicit compile-time safety profiles:

- `esp32dev` is the default environment. Both the raised-wheel flag and motor-output flag are `0`, so motor output remains compile-locked off.
- `esp32dev-trajectory-preview` also has both motor flags at `0`. It advertises the preview capability and can parse, validate, and retain a versioned trajectory without exposing an execution path.
- `esp32dev-trajectory-trace` keeps the same motor lock and preview capability, then performs bounded Pure Pursuit geometry analysis only. It cannot command speed, PWM, pose, STATUS, ARRIVED, or BOOT execution.
- `esp32dev-raised-wheel` is the only environment where both flags are `1`, allowing the existing guarded motion path to energize the TB6612 after route acceptance, local BOOT approval, and the five-second countdown.
- Missing, non-binary, or mismatched profile flags stop compilation. Profile-specific `static_assert` checks independently verify the locked and raised-wheel builds.
- The raised-wheel boot banner explicitly reports `MOTOR OUTPUTS: ENABLED` and warns that the wheels must remain off the floor. The default banner explicitly reports the motor lock.
- MotionController, RouteExecutor, RobotProtocol, GPIO, encoder polarity, 520-count target, PWM profile, BOOT behavior, fault gates, settling, and ARRIVED gating are unchanged from `ef2d4f2`.

Both profiles have been compiled and uploaded to the physical ESP32. The default locked profile was reconfirmed with motor power isolated, and the raised-wheel profile completed the guarded 520-count path described below.

## Confirmed Phase 2C raised-wheel end-to-end result

On 2026-08-10, the user completed one Server-commanded raised-wheel run using:

- ESP32 firmware branch `agent/phase2c-raised-wheel-test` at baseline commit `97c310c2ef77ff60337cdfbb2caee469827d25e3` plus the local Server-host address adjustment required by the current Windows/WSL network.
- Server reference commit `ee3244f39253da23b9d480f775d398316fb46696` in `--physical-demo` mode.
- Unity viewer branch `agent/u1-physical-demo-viewer` at commit `b821d0c2c90e6280310faddbe2555a2f447a9305`.
- The chassis mechanically supported with both wheels clear of the floor.
- ESP32 logic powered by USB while buck `VOUT+ -> ESP32 5V` remained disconnected; the battery powered the TB6612 motor supply through the existing common-ground wiring.
- No FakeRobot session using AGV ID 1.

The observed sequence was:

1. The default `esp32dev` build connected through the corrected Windows-to-WSL forwarding path, received `HELLO_ACK accepted=1`, stored the exact two-node route, and remained at `WAIT_BOOT` with PWM `0/0` and `STBY=LOW`.
2. Unity received the map and displayed the single physical-demo AGV at node 1 while the locked firmware remained stationary.
3. The `esp32dev-raised-wheel` banner explicitly reported enabled motor output and raised-wheel-only operation. With motor power applied, the firmware still remained in `WAIT_BOOT`, PWM `0/0`, and `STBY=LOW`.
4. One BOOT press started the five-second countdown. Both raised wheels then rotated in the intended forward direction, and encoder-based `STATUS.progress` moved Unity AGV 1 from node 1 toward node 2.
5. The motors stopped automatically, settling completed, and the firmware repeatedly reported:

```text
[STATUS] node=2 target=0 progress=1.000 state=ARRIVAL_REPORTED L=538 R=543 PWM=0/0 STBY=LOW
```

The final left/right difference was 5 counts. Relative to the 520-count target, the observed stopping counts were +18 and +23; both remained within the configured `target + 100` overrun fault boundary. No fault was observed during this normal completion run. A later BOOT press in the terminal `ARRIVAL_REPORTED` state was logged as ignored, as expected for the latched completed route.

This proves the first complete normal-path integration loop:

```text
Server exact route [1 -> 2]
    -> ESP32 local approval and encoder-controlled raised-wheel motion
    -> progress/final STATUS
    -> Unity AGV movement
    -> safe local stop and ARRIVAL_REPORTED
```

After the raised-wheel test, the user also reported one low-speed floor run of the same exact `[1 -> 2]` path completing without an observed problem. This confirms the basic single straight floor path, but it does not establish repeatable or calibrated 30 cm accuracy.

## Phase 2D trajectory/odometry preview

On 2026-08-11, the motor-locked preview firmware received Server `07afac4` trajectory `[1 -> 4]` and stored 8 waypoints. The observed result was `STORED_PREVIEW_ONLY`, with `PWM=0`, `STBY=LOW`, and no execution path.

- `TRAJECTORY_COMMAND=102` uses format version `1` and at most 64 robot-local waypoints.
- The wire order is `routeID`, `formatVersion`, `waypointCount`, `startNodeID`, `finalNodeID`, `millimetersPerMapUnit`, followed by 21-byte waypoint records.
- `CAPABILITY_TRAJECTORY_COMMAND` means a future complete follower with STATUS/ARRIVED behavior. This firmware does not advertise it.
- `CAPABILITY_TRAJECTORY_PREVIEW` means parse, validate, store, and log only. It is advertised only by the motor-locked preview environment.
- `TrajectoryCommandStore` has no `MotionController` or `RouteExecutor` reference and exposes no execution API. Cancel, E-stop, or disconnect clears the preview only after the existing local safe-stop path runs.
- Synthetic rotate waypoints must have `nodeID=0`, no `NODE_BOUNDARY`, and zero target speed. Start/final node boundaries and the final STOP/FINAL flags are validated before storage.
- Encoder odometry uses the existing forward-normalized left/right counts, nominal wheel diameter 48 mm, nominal track width 130 mm, and 260 counts/revolution. It reports a robot-local `forward/left/heading` snapshot in mm/rad only.
- Encoder count resets carry an atomic reset epoch so a reset is not misread as reverse travel. The angle normalization is bounded and does not use a potentially unbounded loop.
- Odometry update/logging is compile-gated to `esp32dev-trajectory-preview`; it does not run in the previously verified raised-wheel control loop.
- Network `STATUS.x/z/heading` remains unchanged. Robot-local millimetres are not sent as Server world-map coordinates.

The Server preview path sends packet `102` separately from the legacy `ROUTE_COMMAND`; runtime trajectory motion remains blocked until scale, start heading, follower safety, and failure propagation are complete.

## Phase 2E geometry trace

The new trace evaluates at most one source waypoint per loop with a bounded 45 mm lookahead and the nominal 65 mm half-track. It reports curvature, wheel ratios, minimum radius, and whether an ideal wheel would need reverse motion. It retains no trajectory pointer and has no motor or Arduino API. The currently received Server preview remains 25 mm/map-unit with zero target speeds; the planned 60 mm/map-unit physical scale must be generated by the Server and is not substituted by this firmware.

## Firmware currently built

The active PlatformIO firmware is now split into these responsibilities:

- `src/RobotProtocol.cpp`: RobotProtocol v1 field-by-field serialization.
- `src/RobotClient.cpp`: Wi-Fi/TCP session, HELLO/ACK, packet handling, STATUS, ARRIVED, ERROR, and PING/PONG.
- `src/MotionController.cpp`: encoder interrupts, the verified 30 cm forward profile, synchronization, immediate motion safety, and encoder-stability settling.
- `src/RouteExecutor.cpp`: exact-route validation, BOOT/countdown/E-stop gating, running/settling state, fault latch, progress STATUS, and ARRIVED gating.
- `src/EncoderOdometry.cpp`: preview-only robot-local differential-drive odometry using an atomic encoder reset epoch.
- `src/TrajectoryCommandStore.cpp`: non-driving trajectory validation, duplicate handling, and bounded storage.
- `src/TrajectoryFollowerTrace.cpp`: motor-independent, bounded Pure Pursuit geometry analysis.
- `src/main.cpp`: callback wiring and a non-blocking safety/network loop; it does not duplicate the verified motor algorithm.
- `platformio.ini`: selects the default locked, preview, geometry-trace, or explicit raised-wheel-only profile.

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
- `esp32dev-trajectory-preview` is independently asserted to remain motor-locked; trajectory receipt cannot reach a motor-start API.
- `esp32dev-trajectory-trace` is independently asserted to require preview mode while both motor-output and raised-wheel flags remain false.
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

- During the verified raised-wheel run, `STATUS.progress` was derived from the slower wheel's count divided by 520, clamped to `0.0` through `1.0`, and visualized by Unity.
- While moving, `currentNodeID` remains node 1 and `currentLinkID` carries target node 2, matching Server reference commit `ee3244f39253da23b9d480f775d398316fb46696`.
- During non-blocking settling, `currentNodeID` remains node 1, target node 2 remains identified, and encoder progress may remain `1.0` while outputs are already safe.
- Only after the 150 ms stability gate completes does the prepared final STATUS report node 2 and progress `1.0` before `ARRIVED` is attempted.
- Metric battery sensing, Server-world `(x,z,heading)` odometry, trajectory execution, turns, and arbitrary route expansion are not implemented in this phase. Robot-local odometry exists only in the motor-locked preview log.

## Preserved physical-vehicle reference

- `AGV_Project_Record/final_l_route_main.cpp` remains unchanged and outside the active `src/` build.
- Its physically verified wiring is left motor `25/26/PWM27`, right motor `33/32/PWM14`, `STBY=13`, left encoder `19/18`, right encoder `17/16`, and BOOT `0`.
- Its verified values include 260 counts per wheel revolution, 520 counts for approximately 30 cm, and 176 counts per wheel for the calibrated 90-degree turn.
- The forward ISR polarity, acceleration/deceleration values, PWM baseline, and count synchronization were transferred into `MotionController`; the reference file was not moved or rewritten.

## Build validation

- On 2026-08-11, the Phase 2E worktree passed the host geometry assertions and all four PlatformIO builds without upload.

- On 2026-08-10, the Phase 2D worktree built all three environments without upload:
  - `esp32dev`: RAM 46,384 bytes (14.2%), flash 775,505 bytes (59.2%)
  - `esp32dev-trajectory-preview`: RAM 48,028 bytes (14.7%), flash 785,201 bytes (59.9%)
  - `esp32dev-raised-wheel`: RAM 46,384 bytes (14.2%), flash 776,401 bytes (59.2%)
- The matching WSL Server worktree passed CMake configure/build, `ctest`, and `TrajectorySmokeTest`. This was synthetic LINE/Bezier/LINE and serializer validation only; the Server did not dispatch a trajectory to hardware.

- `platformio run -e esp32dev` succeeded on 2026-08-08.
  Reported use was 46,360 bytes of RAM (14.1%) and 774,445 bytes of flash (59.1%).
- `platformio run -e esp32dev-raised-wheel` succeeded on 2026-08-08.
  Reported use was 46,360 bytes of RAM (14.1%) and 775,345 bytes of flash (59.2%).
- On 2026-08-10, both the default locked profile and the raised-wheel profile were uploaded and their profile-specific boot banners were observed on the physical ESP32.
- The raised-wheel profile completed one powered, Server-commanded 520-count normal path with final counts `L=538`, `R=543`, PWM `0/0`, `STBY=LOW`, progress `1.000`, and `ARRIVAL_REPORTED`.
- Unity displayed the physical-demo AGV moving from node 1 toward node 2 from ESP32 progress STATUS. A later low-speed floor run was user-reported successful, but calibrated distance/repeatability was not measured.

## Explicitly not yet verified

- The 3-second stale-HELLO reconnect behavior on the live Windows/WSL path
- Explicit socket cleanup after a failed `WiFiClient.connect()` attempt; this path is unchanged in the reduced M4 scope
- M4 countdown-retain and BOOT E-stop behavior during an active powered run
- Injected settling activity and the 2,000 ms settling-timeout fault path on actual hardware
- Every fault input on raised wheels, including stall, reversed count, mismatch, timeout, TCP loss, output-invariant failure, and E-stop
- Calibrated and repeatable floor distance for the nominal 520-count/30 cm segment
- The future policy for detecting an accepted but half-open TCP session; M4 intentionally does not treat Server silence as a fault
- Exactly-once Server processing of the safe-completion `ARRIVED`; the firmware's terminal `ARRIVAL_REPORTED` state was observed, but a separate Server-side exactly-once audit was not captured
- Robot-local odometry calibration, Server-world pose alignment, metric velocity STATUS, battery sensing, trajectory following, turns, or general multi-node execution
- Physical `millimetersPerMapUnit`, trusted start heading, and a drivable curve geometry
- Phase 2E serial trace against a Server-generated 60 mm/map-unit packet; only host tests and builds are complete

## Next safe step

Keep routine work motor-locked. Next, have the Server generate the same preview at 60 mm/map-unit, then inspect it with the trace profile over USB only. Powered curve control remains a separate later approval.
