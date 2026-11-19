#include <math.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "motors.h"
#include "robot_config.h"

typedef enum { M_ESTOP, M_BRAKE, M_RUN } mmode_t;
static mmode_t mode = M_ESTOP;
static float cur[2], tgt[2];
static uint16_t wrap;

static const uint8_t IN1[2] = {PIN_L_IN1, PIN_R_IN1};
static const uint8_t IN2[2] = {PIN_L_IN2, PIN_R_IN2};
static const uint8_t PWM[2] = {PIN_L_PWM, PIN_R_PWM};

static void write_pwm(int m, float duty) {
    float a = fabsf(duty);
    if (a > 1.0f) a = 1.0f;
    pwm_set_gpio_level(PWM[m], (uint16_t)(a * (wrap + 1)));
}

static void write_motor(int m, float duty) {
    if (m == 0 ? MOTOR_L_INVERT : MOTOR_R_INVERT) duty = -duty;
    gpio_put(IN1[m], duty > 0.0f);
    gpio_put(IN2[m], duty < 0.0f);
    write_pwm(m, duty);
}

void motors_init(void) {
    gpio_init(PIN_STBY); gpio_set_dir(PIN_STBY, GPIO_OUT); gpio_put(PIN_STBY, 0);
    wrap = (uint16_t)(clock_get_hz(clk_sys) / PWM_FREQ_HZ - 1);   // 125 MHz / 10 kHz -> 12499
    for (int m = 0; m < 2; m++) {
        gpio_init(IN1[m]); gpio_set_dir(IN1[m], GPIO_OUT); gpio_put(IN1[m], 0);
        gpio_init(IN2[m]); gpio_set_dir(IN2[m], GPIO_OUT); gpio_put(IN2[m], 0);
        gpio_set_function(PWM[m], GPIO_FUNC_PWM);
        pwm_config c = pwm_get_default_config();
        pwm_config_set_wrap(&c, wrap);
        pwm_init(pwm_gpio_to_slice_num(PWM[m]), &c, true);
        pwm_set_gpio_level(PWM[m], 0);
    }
    mode = M_ESTOP;
}

void motors_estop(void) {
    mode = M_ESTOP;
    cur[0] = cur[1] = tgt[0] = tgt[1] = 0;
    for (int m = 0; m < 2; m++) { write_pwm(m, 0); gpio_put(IN1[m], 0); gpio_put(IN2[m], 0); }
    gpio_put(PIN_STBY, 0);
}

void motors_brake(void) {
    mode = M_BRAKE;
    cur[0] = cur[1] = tgt[0] = tgt[1] = 0;
    gpio_put(PIN_STBY, 1);
    for (int m = 0; m < 2; m++) { gpio_put(IN1[m], 1); gpio_put(IN2[m], 1); write_pwm(m, 1.0f); }
}

void motors_run(void) {
    if (mode == M_RUN) return;
    cur[0] = cur[1] = tgt[0] = tgt[1] = 0;
    for (int m = 0; m < 2; m++) write_motor(m, 0);
    gpio_put(PIN_STBY, 1);
    mode = M_RUN;
}

void motors_zero_now(void) {
    if (mode != M_RUN) return;
    cur[0] = cur[1] = tgt[0] = tgt[1] = 0;
    for (int m = 0; m < 2; m++) write_motor(m, 0);
}

void motors_set_duty(float l, float r) {
    if (mode != M_RUN) return;
    tgt[0] = l; tgt[1] = r;
}

void motors_update(float dt_s) {
    if (mode != M_RUN) return;
    float step = dt_s / (SLEW_FULL_SCALE_MS / 1000.0f);   // 0.01/0.15 = 6.67 % per tick
    for (int m = 0; m < 2; m++) {
        float d = tgt[m] - cur[m];
        if (d >  step) d =  step;
        if (d < -step) d = -step;
        cur[m] += d;
        write_motor(m, cur[m]);
    }
}

float motors_get_duty(int side) { return cur[side]; }
