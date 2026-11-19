#pragma once
#include <stdint.h>
void encoders_init(void);
void encoders_poll(void);                                          // drain both PIO FIFOs, call every loop
void encoder_get_and_reset_deltas(int16_t *delta_left, int16_t *delta_right);  // spec 2.2.3 (PID consumer)
void encoders_total(int32_t *left, int32_t *right);                // free-running totals (stall + telemetry)
