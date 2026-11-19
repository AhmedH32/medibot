#pragma once
#include <stdbool.h>
bool  imu_init(void);
void  imu_update(void);        // call at ~100 Hz; computes its own dt
float imu_pitch_deg(void);
float imu_roll_deg(void);
bool  imu_ok(void);
