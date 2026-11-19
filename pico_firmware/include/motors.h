#pragma once
void  motors_init(void);
void  motors_run(void);                  // leave estop/brake, STBY high
void  motors_estop(void);                // STBY low, all PWM 0 (immediate)
void  motors_brake(void);                // hard lock: short-brake both wheels (immediate)
void  motors_zero_now(void);             // drop duty to 0 immediately but stay enabled (sonar override)
void  motors_set_duty(float l, float r); // -1..1 target, reached through the slew limiter
void  motors_update(float dt_s);         // call every 10 ms: max 6.67 % change per tick
float motors_get_duty(int side);         // current (post-slew) duty, signed
