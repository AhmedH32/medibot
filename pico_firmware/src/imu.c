#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "imu.h"
#include "robot_config.h"

#define I2C_PORT  i2c0
#define MPU_ADDR  0x68
#define TO_US     3000
#define ALPHA     0.98f      // complementary filter: gyro weight

static float pitch, roll;
static bool  ok;
static uint32_t last_us;

static bool wr(uint8_t reg, uint8_t val) {
    uint8_t b[2] = {reg, val};
    return i2c_write_timeout_us(I2C_PORT, MPU_ADDR, b, 2, false, TO_US) == 2;
}

bool imu_init(void) {
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(PIN_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_I2C_SDA);
    gpio_pull_up(PIN_I2C_SCL);
    ok = wr(0x6B, 0x00) && wr(0x1A, 0x03);   // wake, DLPF ~44 Hz (defaults: +-2 g, +-250 dps)
    last_us = time_us_32();
    return ok;
}

void imu_update(void) {
    uint8_t reg = 0x3B, d[14];
    if (i2c_write_timeout_us(I2C_PORT, MPU_ADDR, &reg, 1, true, TO_US) != 1 ||
        i2c_read_timeout_us(I2C_PORT, MPU_ADDR, d, 14, false, TO_US) != 14) { ok = false; return; }
    ok = true;

    uint32_t now = time_us_32();
    float dt = (now - last_us) * 1e-6f;
    last_us = now;
    if (dt <= 0 || dt > 0.1f) return;

    #define S16(i) ((int16_t)((d[i] << 8) | d[i + 1]))
    float ax = S16(0) / 16384.0f, ay = S16(2) / 16384.0f, az = S16(4) / 16384.0f;
    float gx = S16(8) / 131.0f,   gy = S16(10) / 131.0f;   // deg/s

    // Axis mapping depends on how the board is mounted: swap/negate here (TUNE).
    float roll_acc  = atan2f(ay, az) * 57.29578f;
    float pitch_acc = atan2f(-ax, sqrtf(ay * ay + az * az)) * 57.29578f;
    roll  = ALPHA * (roll  + gx * dt) + (1 - ALPHA) * roll_acc;
    pitch = ALPHA * (pitch + gy * dt) + (1 - ALPHA) * pitch_acc;
}

float imu_pitch_deg(void) { return pitch; }
float imu_roll_deg(void)  { return roll; }
bool  imu_ok(void)        { return ok; }
