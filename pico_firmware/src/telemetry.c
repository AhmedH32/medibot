#include <math.h>
#include "hardware/uart.h"
#include "telemetry.h"
#include "protocol_defs.h"
#include "encoders.h"
#include "imu.h"
#include "sonar.h"
#include "battery.h"

static int32_t last[2];
static int8_t clamp_i8(int32_t v) { return v > 127 ? 127 : (v < -128 ? -128 : (int8_t)v); }
static int8_t angle_i8(float a)   { return clamp_i8((int32_t)lroundf(a > 90 ? 90 : (a < -90 ? -90 : a))); }

void telemetry_send(bool motor_stalled) {
    int32_t l, r;
    encoders_total(&l, &r);
    int32_t dl = l - last[0], dr = r - last[1];       // ticks over the last 100 ms
    last[0] = l; last[1] = r;

    uint8_t f[TEL_PACKET_SIZE];
    f[0] = TEL_SENTINEL;
    f[1] = (uint8_t)angle_i8(imu_pitch_deg());
    f[2] = (uint8_t)angle_i8(imu_roll_deg());
    f[3] = (uint8_t)clamp_i8(dl);                     // NOTE: int8 saturates, see delivery notes
    f[4] = (uint8_t)clamp_i8(dr);
    f[5] = sonar_min_cm();
    f[6] = (battery_percent() & TEL_BATTERY_MASK) | (motor_stalled ? TEL_STALL_FLAG_BIT : 0);
    uint8_t sum = 0;
    for (int i = 1; i <= 6; i++) sum += f[i];
    f[7] = sum;
    uart_write_blocking(uart0, f, TEL_PACKET_SIZE);
}
