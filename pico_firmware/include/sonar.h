#pragma once
#include <stdint.h>
void    sonar_init(void);
void    sonar_tick(void);        // every 30 ms: one shared trigger, alternating L/R echo windows
uint8_t sonar_min_cm(void);      // min of both, 255 = nothing in range
