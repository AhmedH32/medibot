#pragma once
#include <stdbool.h>
void telemetry_send(bool motor_stalled);   // 8-byte 0x55 frame, call every 100 ms
