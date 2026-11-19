#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "uart_fsm.h"
#include "robot_config.h"

#define PI_UART uart0

typedef enum { SEARCH_SENTINEL, READ_VX, READ_WZ, READ_FLAGS, VERIFY_CHECKSUM } fsm_state_t;

static fsm_state_t state = SEARCH_SENTINEL;
static int8_t  vx, wz;
static uint8_t flags;
static cmd_t   cmd;
static uint32_t errors;
volatile uint64_t last_valid_packet_us;

void uart_fsm_init(void) {
    uart_init(PI_UART, UART_BAUD);
    gpio_set_function(PIN_UART_TX, GPIO_FUNC_UART);
    gpio_set_function(PIN_UART_RX, GPIO_FUNC_UART);
    uart_set_format(PI_UART, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(PI_UART, true);
}

void uart_fsm_poll(void) {
    while (uart_is_readable(PI_UART)) {
        uint8_t b = uart_getc(PI_UART);
        switch (state) {
        case SEARCH_SENTINEL:
            if (b == CMD_SENTINEL) state = READ_VX;
            break;
        case READ_VX:    vx = (int8_t)b;  state = READ_WZ;    break;
        case READ_WZ:    wz = (int8_t)b;  state = READ_FLAGS; break;
        case READ_FLAGS: flags = b;       state = VERIFY_CHECKSUM; break;
        case VERIFY_CHECKSUM:
            if (b == (uint8_t)(vx + wz + flags)) {            // CS = (Vx + Wz + Flags) & 0xFF
                cmd.vx_cms = vx; cmd.wz_dps = wz; cmd.flags = flags;
                last_valid_packet_us = time_us_64();
            } else {
                errors++;                                       // discard the whole frame
            }
            state = SEARCH_SENTINEL;
            break;
        }
    }
}

void uart_fsm_get_cmd(cmd_t *out) { *out = cmd; }
uint32_t uart_fsm_error_count(void) { return errors; }
