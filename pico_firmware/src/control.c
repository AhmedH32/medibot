#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "control.h"
#include "protocol_defs.h"
#include "robot_config.h"
#include "uart_fsm.h"
#include "motors.h"
#include "encoders.h"
#include "sonar.h"
#include "pid.h"

#define DEG2RAD 0.017453293f
static const float TICKS_PER_CM = ENC_TICKS_PER_REV / (3.14159265f * WHEEL_DIAMETER_CM);

static pid_ctrl_t pid[2];
static float pid_duty[2], target_v[2];
static bool  comm_lost, stalled, obstacle_active, prev_dlg;
static float last_vx, last_wz, held_vx, held_wz;
static uint8_t stall_cnt;

static void log_err(const char *m) { if (stdio_usb_connected()) printf("%s\n", m); }   // never blocks without a host

void control_init(void) {
    for (int i = 0; i < 2; i++) pid_init(&pid[i], PID_KP, PID_KI, PID_KD, PID_I_CLAMP, -1.0f, 1.0f);
}
bool control_motor_stalled(void) { return stalled; }

static void reset_loops(void) {
    pid_reset(&pid[0]); pid_reset(&pid[1]);
    pid_duty[0] = pid_duty[1] = 0; stall_cnt = 0;
}

// Spec 2.2.4: no valid frame for >= 200 ms -> STBY low, PWM 0, log ERR_WATCHDOG_TIMEOUT
void control_watchdog_check(uint64_t now_us) {
    bool expired = (now_us - last_valid_packet_us) >= WATCHDOG_TIMEOUT_US;
    if (expired && !comm_lost) {
        comm_lost = true;
        motors_estop(); reset_loops();
        log_err("ERR_WATCHDOG_TIMEOUT");
    } else if (!expired) {
        comm_lost = false;
    }
}

void control_tick_10ms(float dt) {
    if (comm_lost) return;                                    // already e-stopped

    cmd_t c;
    uart_fsm_get_cmd(&c);
    if (c.flags & FLAG_EMERGENCY_FALL)        { motors_brake(); reset_loops(); return; }   // hard lock
    if (!(c.flags & FLAG_MOTOR_ENABLE))       { motors_estop(); reset_loops(); stalled = false; return; }

    float vx = c.vx_cms, wz = c.wz_dps;

    // Audio dialogue: hold the velocity from just before the flag rose (the slew limiter keeps it smooth)
    bool dlg = c.flags & FLAG_DIALOGUE_ACTIVE;
    if (dlg) { if (!prev_dlg) { held_vx = last_vx; held_wz = last_wz; } vx = held_vx; wz = held_wz; }
    else     { last_vx = vx; last_wz = wz; }
    prev_dlg = dlg;

    // Sonar override (spec 3.3.4): either sensor < 20 cm -> Vx = 0 if moving forward, keep Wz
    bool obst = sonar_min_cm() < OBSTACLE_STOP_CM, zero_now = false;
    if (obst && vx > 0) {
        vx = 0;
        if (!obstacle_active) { obstacle_active = true; zero_now = true; }
    } else if (!obst) obstacle_active = false;

    // Stall latch: stays stopped until the Pi commands zero velocity (or clears the enable bit)
    if (stalled) {
        if (c.vx_cms == 0 && c.wz_dps == 0) { stalled = false; reset_loops(); }
        else { motors_estop(); return; }
    }

    motors_run();
    if (zero_now) { motors_zero_now(); reset_loops(); }       // forward PWM drops to 0 immediately

    // Differential drive kinematics (spec eq. 2)
    float half = wz * DEG2RAD * (WHEELBASE_CM * 0.5f);
    target_v[0] = vx - half;
    target_v[1] = vx + half;

    float duty[2];
    for (int i = 0; i < 2; i++) {
#if PHASE2_CLOSED_LOOP
        duty[i] = (target_v[i] == 0.0f) ? 0.0f : pid_duty[i];
#if PID_USE_FEEDFORWARD
        if (target_v[i] != 0.0f) duty[i] += target_v[i] / MAX_WHEEL_SPEED_CMS;
#endif
#else
        duty[i] = target_v[i] / MAX_WHEEL_SPEED_CMS;          // Phase 1: open loop
#endif
        if (duty[i] > 1.0f) duty[i] = 1.0f; else if (duty[i] < -1.0f) duty[i] = -1.0f;
    }
    motors_set_duty(duty[0], duty[1]);
    motors_update(dt);                                        // slew: max 6.67 % per 10 ms
}

void control_pid_tick(float dt) {
    int16_t dl, dr;
    encoder_get_and_reset_deltas(&dl, &dr);
#if PHASE2_CLOSED_LOOP
    int16_t meas[2] = {dl, dr};
    for (int i = 0; i < 2; i++) {
        if (comm_lost || stalled || target_v[i] == 0.0f) { pid_reset(&pid[i]); pid_duty[i] = 0; continue; }
        float target_ticks = target_v[i] * TICKS_PER_CM * dt;            // ticks per 50 ms
        pid_duty[i] = pid_update(&pid[i], target_ticks, (float)meas[i], dt);
    }
#else
    (void)dl; (void)dr; (void)dt;
#endif
}

void control_stall_tick(void) {
    static int32_t last[2];
    int32_t l, r;
    encoders_total(&l, &r);
    int32_t d[2] = {l - last[0], r - last[1]};
    last[0] = l; last[1] = r;
#if PHASE2_STALL_PROTECTION
    bool bad = false;
    for (int i = 0; i < 2; i++)
        if (fabsf(motors_get_duty(i)) > STALL_DUTY_MIN && labs(d[i]) < STALL_TICKS_MAX) bad = true;
    stall_cnt = bad ? stall_cnt + 1 : 0;
    if (stall_cnt >= STALL_CHECKS && !stalled) {
        stalled = true;
        motors_estop();                                       // STBY low immediately
        pid_reset(&pid[0]); pid_reset(&pid[1]);               // integrals to zero
        log_err("ERR_MOTOR_STALL");
    }
#else
    (void)d;
#endif
}
