#pragma once
#include <stdint.h>
void     battery_init(void);
uint16_t battery_read_filtered(void);   // 64-sample average of the raw 12-bit code (blocks ~3.2 ms)
void     battery_update(void);          // call at 10 Hz
uint8_t  battery_percent(void);         // 0..100, linear between code 2470 and 3240
