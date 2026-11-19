#pragma once
#include <stdint.h>
#include "protocol_defs.h"
extern volatile uint64_t last_valid_packet_us;   // spec 2.2.4
void     uart_fsm_init(void);
void     uart_fsm_poll(void);                    // non-blocking, call every loop
void     uart_fsm_get_cmd(cmd_t *out);
uint32_t uart_fsm_error_count(void);
