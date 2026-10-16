# Subsystem B: Binary UART Protocol Specification

- **Bus:** 3.3V UART (RP2040 GP0/GP1 <-> Pi 5 GPIO 14/15)[cite: 14, 19]
- **Baudrate:** 115200 bps (8N1)[cite: 11, 16]
- **Host Laptop Emulation:** TCP Socket `127.0.0.1:5555` (`socket://localhost:5555`)

---

## 1. Downstream Command Packet (Pi 5 -> Pico)
- **Transmission Rate:** 50 Hz (every 20 ms)[cite: 11, 17]
- **Frame Size:** Exactly 5 Bytes[cite: 11, 17]
- **Watchdog Rule:** The Pico cuts all motor PWM to 0 and pulls STBY LOW if no valid frame arrives within 200 ms[cite: 12, 18].

| Byte | Field | Type | Range / Units | Description |
| :--- | :--- | :--- | :--- | :--- |
| **0** | Start Sentinel | `uint8` | `0xAA` | Frame sync sentinel[cite: 11, 17] |
| **1** | $V_x$ | `int8` | -100 to +100 cm/s | Forward linear velocity demand[cite: 11, 12, 17, 18] |
| **2** | $W_z$ | `int8` | -128 to +127 deg/s | Rotational velocity demand (+ = CCW)[cite: 11, 12, 17, 18] |
| **3** | Control Flags | `uint8` | Bitmask | Bit 0 (`0x01`): Motor Enable (1=Active, 0=Standby)<br>Bit 1 (`0x02`): Emergency Fall State (1=Hard Lock)<br>Bit 2 (`0x04`): Audio Dialogue Active (1=Hold Velocity)[cite: 11, 12, 17, 18] |
| **4** | Checksum | `uint8` | 0x00 to 0xFF | `(uint8_t)(Vx + Wz + Flags) & 0xFF`[cite: 11, 12, 17, 18] |

---

## 2. Upstream Telemetry Packet (Pico -> Pi 5)
- **Transmission Rate:** 10 Hz (every 100 ms)[cite: 13, 18]
- **Frame Size:** Exactly 8 Bytes[cite: 13, 18]

| Byte | Field | Type | Range / Units | Description |
| :--- | :--- | :--- | :--- | :--- |
| **0** | Start Sentinel | `uint8` | `0x55` | Frame sync sentinel[cite: 13, 18] |
| **1** | Pitch Angle | `int8` | -90 to +90 deg | Pitch from MPU-6050 IMU[cite: 13, 18] |
| **2** | Roll Angle | `int8` | -90 to +90 deg | Roll from MPU-6050 IMU[cite: 13, 18] |
| **3** | Delta Ticks Left | `int8` | Signed count | Left encoder ticks accumulated over the last 100 ms[cite: 13, 18] |
| **4** | Delta Ticks Right | `int8` | Signed count | Right encoder ticks accumulated over the last 100 ms[cite: 13, 18] |
| **5** | Sonar Distance | `uint8` | 0 to 255 cm | Minimum distance from left/right HC-SR04 sensors[cite: 13, 18] |
| **6** | Battery & Stall | `uint8` | Packed Byte | **Bits 0–6 (`0x7F`):** Battery 0–100% (9.6V to 12.6V)[cite: 13, 15, 18, 21]<br>**Bit 7 (`0x80`):** Motor Stall Flag (1=Stalled, 0=OK)[cite: 15, 21] |
| **7** | Checksum | `uint8` | 0x00 to 0xFF | `(sum(Bytes[1..6])) & 0xFF`[cite: 13, 18] |

---

## 3. C Definition Reference (For RP2040 Firmware)
```c
#define CMD_PACKET_SIZE       5
#define CMD_SENTINEL          0xAA
#define FLAG_MOTOR_ENABLE     0x01
#define FLAG_EMERGENCY_FALL   0x02
#define FLAG_DIALOGUE_ACTIVE  0x04

#define TEL_PACKET_SIZE       8
#define TEL_SENTINEL          0x55
#define TEL_BATTERY_MASK      0x7F
#define TEL_STALL_FLAG_BIT    0x80
```