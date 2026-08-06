# ESP32Test Repository Instructions

## Scope

- This directory is the existing PlatformIO ESP32 project and is the only writable project for ESP32 work.
- Do not create a replacement ESP32 project or copy the firmware to another directory.
- Treat the SmartFactory AGV Digital Twin Server repository as read-only reference unless the user explicitly expands the writable scope.
- Read `docs/current-status.md` before changing firmware behavior.

## Preserved baselines

- `src/` and `include/` contain the firmware currently built by PlatformIO.
- `AGV_Project_Record/final_l_route_main.cpp` is the preserved, physically verified motor/encoder/L-route reference.
- These two baselines are intentionally separate. Do not claim end-to-end physical Server integration until they are deliberately combined and tested.
- Do not delete, move, or silently rewrite the verified reference code.

## Safety

- Do not enable motor outputs, connect motor power, upload firmware, or start a serial monitor unless the user explicitly requests that action.
- A build request authorizes compilation only; it does not authorize upload.
- Keep the current compile-time motor lock and `STBY=LOW` safety behavior unless a separately approved physical-integration task changes it.
- Before any future upload or powered test, follow the documented hardware power-isolation procedure and confirm the current physical wiring with the user.

## Credentials and generated files

- Never open, print, stage, commit, or share `include/Secrets.hpp`.
- Use `include/Secrets.example.hpp` only as the public placeholder interface.
- Do not stage PlatformIO build output, temporary build workspaces, logs, private keys, or machine-local IDE files.
- Before any commit or push, verify both ignored files and staged files without displaying credential values.

## Server protocol

- The protocol source of truth is the Server repository's `Shared/Protocol.hpp` and `Shared/PacketSerializer.*`.
- Preserve field-by-field serialization; do not send C++ structs with raw `memcpy`.
- The Server owns world, task, route, and reservation state. The ESP32 owns local motion, encoder feedback, and immediate safety.
- Protocol or integration changes must update `docs/current-status.md` and be checked against the canonical Server files.

## Git workflow

- Show the user the exact commit candidates before committing.
- Do not commit or push without explicit user approval.
- Do not rewrite or delete unrelated user files.
