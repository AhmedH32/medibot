#include "pico/stdlib.h"
#include "hardware/watchdog.h"
#include "protocol_defs.h"
#include "robot_config.h"
#include "uart_fsm.h"
#include "motors.h"
#include "encoders.h"
#include "imu.h"
#include "sonar.h"
#include "battery.h"
#include "telemetry.h"
#include "control.h"

static inline bool due(uint64_t now, uint64_t *next, uint32_t period_ms) {
    if (now < *next) return false;
    *next = now + period_ms * 1000ULL;
    return true;
}

int main(void) {
    stdio_init_all();           // USB only (UART0 belongs to the Pi link): used for ERR_* logging
    motors_init();              // outputs safe first
    uart_fsm_init();
    encoders_init();
    imu_init();
    sonar_init();
    battery_init();
    control_init();

    watchdog_enable(250, true); // last-resort reset if this loop ever hangs (comm timeout is handled in software)

    uint64_t t = time_us_64();
    uint64_t n_wd = t, n_mot = t, n_son = t, n_pid = t, n_safe = t + 100000;

    while (true) {
        watchdog_update();
        uart_fsm_poll();
        encoders_poll();

        uint64_t now = time_us_64();
        if (due(now, &n_wd,  WATCHDOG_CHECK_MS)) control_watchdog_check(now);
        if (due(now, &n_mot, MOTOR_TICK_MS))     { imu_update(); control_tick_10ms(MOTOR_TICK_MS / 1000.0f); }
        if (due(now, &n_son, SONAR_WINDOW_MS))   sonar_tick();
        if (due(now, &n_pid, PID_PERIOD_MS))     control_pid_tick(PID_PERIOD_MS / 1000.0f);
        if (due(now, &n_safe, SAFETY_PERIOD_MS)) {
            control_stall_tick();
            battery_update();
            telemetry_send(control_motor_stalled());
        }
    }
}
