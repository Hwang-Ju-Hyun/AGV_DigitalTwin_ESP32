# ESP32Test Current Status

Last reviewed: 2026-08-06

## Repository baseline

- This directory is the existing PlatformIO project for the physical ESP32 AGV.
- Git was initialized in place to prepare the first local baseline. No commit or remote has been created yet.
- PlatformIO build output, temporary build directories, IDE-local files, and the real credential file are excluded by `.gitignore`.
- `include/Secrets.hpp` is local-only and must never be opened for reporting, staged, committed, or pushed.

## Firmware currently built

- `src/main.cpp`, `src/RobotClient.cpp`, and `src/RobotProtocol.cpp` are the active build sources.
- The active firmware is the Phase 2A Server communication/BOOT-approval dry run.
- Motor output remains compile-locked off. The loop forces PWM to zero and TB6612 `STBY` low.
- The current build can connect, perform the RobotProtocol handshake, receive/store the restricted demo route, report safe placeholder status, and answer ping with pong.
- It does not execute physical movement or transmit `ARRIVED` from a completed motion.

## Preserved physical-vehicle reference

- `AGV_Project_Record/final_l_route_main.cpp` preserves the physically verified local motion implementation.
- Verified reference values include 260 encoder counts per wheel revolution, 520 counts for approximately 30 cm, and 176 counts per wheel for the calibrated 90-degree turn.
- The reference includes encoder interrupts, signed direction checks, acceleration/deceleration profiles, left/right count synchronization, stall detection, timeout, overrun, and mismatch safety checks.
- This file is reference code and is not part of the current PlatformIO `src/` build.

## Server integration boundary

- The Server is authoritative for world, tasks, routes, and reservations.
- The ESP32 is responsible for local motor control, encoder feedback, and immediate safety.
- Canonical packet definitions come from the Server's `Shared/Protocol.hpp` and `Shared/PacketSerializer.*`; Server documentation is supporting context.
- The ESP32 packet layout currently matches RobotProtocol version 1, but the active communication firmware and verified physical motion implementation have not yet been combined.
- The current ESP32 demo interprets one restricted node route as a local 30 cm test. That temporary interpretation is not a general mapping between the Server map and physical distance.

## Explicitly not yet verified

- Server command to encoder-controlled physical motion end to end
- Real route progress, velocity, battery, or odometry in `STATUS`
- Exactly-once `ARRIVED` after a physically confirmed stop
- Physical fault reporting and Server-side fault/E-stop world handling
- Communication watchdog and non-blocking reconnect behavior during motion

## Baseline validation

- `platformio run` for the existing `esp32dev` environment succeeded on 2026-08-06.
- The build used the existing active sources and completed with 45,744 bytes of RAM and 767,373 bytes of flash reported in use.
- No firmware upload, serial monitor, motor-power connection, or motor-output activation was performed as part of the Git baseline preparation.

## Next safe step

Review the initial Git commit candidates and confirm that no generated files or credentials are staged. Commit and push only after explicit user approval. Firmware integration remains a separate future task.
