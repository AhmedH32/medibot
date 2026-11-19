#pragma once
// Discrete PID, spec 3.3.3: u = Kp e + Ki sum(e dt) + Kd (e - e_prev)/dt
typedef struct { float kp, ki, kd, i_term, prev_err, i_clamp, lo, hi; } pid_ctrl_t;

static inline void pid_init(pid_ctrl_t *p, float kp, float ki, float kd, float i_clamp, float lo, float hi) {
    p->kp = kp; p->ki = ki; p->kd = kd; p->i_clamp = i_clamp; p->lo = lo; p->hi = hi;
    p->i_term = 0; p->prev_err = 0;
}
static inline void pid_reset(pid_ctrl_t *p) { p->i_term = 0; p->prev_err = 0; }   // "clamp integrals to zero"

static inline float pid_update(pid_ctrl_t *p, float target, float measured, float dt) {
    float e = target - measured;
    p->i_term += p->ki * e * dt;
    if (p->i_term >  p->i_clamp) p->i_term =  p->i_clamp;     // anti-windup +-25 % PWM
    if (p->i_term < -p->i_clamp) p->i_term = -p->i_clamp;
    float u = p->kp * e + p->i_term + p->kd * (e - p->prev_err) / dt;
    p->prev_err = e;
    return u > p->hi ? p->hi : (u < p->lo ? p->lo : u);
}
