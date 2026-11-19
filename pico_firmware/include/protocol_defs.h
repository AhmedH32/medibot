#pragma once
// Mirrors docs/contracts/uart_protocol_contract.md. Change the contract first, then this file.
#include <stdint.h>

#define UART_BAUD             115200
#define WATCHDOG_TIMEOUT_MS   200
#define WATCHDOG_TIMEOUT_US   200000ULL

// Pi 5 -> Pico (5 bytes, 50 Hz)
#define CMD_PACKET_SIZE       5
#define CMD_SENTINEL          0xAA
#define FLAG_MOTOR_ENABLE     0x01
#define FLAG_EMERGENCY_FALL   0x02
#define FLAG_DIALOGUE_ACTIVE  0x04

// Pico -> Pi 5 (8 bytes, 10 Hz)
#define TEL_PACKET_SIZE       8
#define TEL_SENTINEL          0x55
#define TEL_BATTERY_MASK      0x7F
#define TEL_STALL_FLAG_BIT    0x80

typedef struct {
    int8_t  vx_cms;    // forward cm/s
    int8_t  wz_dps;    // deg/s, + = CCW
    uint8_t flags;
} cmd_t;
