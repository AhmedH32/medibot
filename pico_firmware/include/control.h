#pragma once
#include <stdbool.h>
#include <stdint.h>
void control_init(void);
void control_watchdog_check(uint64_t now_us);   // every 5 ms  (spec 2.2.4)
void control_tick_10ms(float dt_s);             // safety ladder, kinematics, slew, PWM
void control_pid_tick(float dt_s);              // every 50 ms (20 Hz)
void control_stall_tick(void);                  // every 100 ms
bool control_motor_stalled(void);
