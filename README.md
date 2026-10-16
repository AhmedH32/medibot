# MediBot — Autonomous Edge-Assistive Medical Robot

Privacy-first edge-to-cloud elderly care robot powered by a Raspberry Pi 5 (8GB + NVMe SSD) and a Raspberry Pi Pico (RP2040) real-time motion controller.

## Repository Structure & Ownership
* **`main_board/` (Ahmed):** Raspberry Pi 5 host stack — Multi-process core orchestration, 50Hz UART daemon, YOLOv8n-Pose tracking/fall/medication verification, local Medical RAG (FAISS), quantized Gemma 2B voice pipeline, and Firebase FCM bridge.
* **`pico_firmware/` (Alaa):** RP2040 C/C++ firmware — Non-blocking UART FSM, 150ms PWM slew-rate limiter, PIO quadrature encoders, 200ms hardware watchdog, MPU-6050 IMU, HC-SR04 obstacle override, and stall protection.
* **`mobile_app/` (Mobile Lead):** Family companion app — Real-time FCM notifications (`INFO`, `ADVISORY`, `CRITICAL`), 5-minute heartbeat monitor, battery telemetry, and medication alarm scheduling.
* **`docs/contracts/`:** Binary UART specification and Firebase payload schemas.

## Git Workflow Rules
1. Never push directly to `main` without testing.
2. Work strictly inside your assigned directory (`main_board/`, `pico_firmware/`, or `mobile_app/`).
3. Do not commit large model binaries (`.gguf`, `.onnx`, `.pt`) or Firebase private keys.
