#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "quad.pio.h"
#include "encoders.h"
#include "robot_config.h"

// index = (old << 2) | new ; state = (B << 1) | A
static const int8_t QDELTA[16] = { 0,+1,-1, 0,  -1, 0, 0,+1,  +1, 0, 0,-1,   0,-1,+1, 0};

static PIO pios[2] = {pio0, pio1};          // left on PIO0, right on PIO1
static uint8_t prev[2];
static int32_t total[2];
static int32_t reported[2];

void encoders_init(void) {
    for (int i = 0; i < 2; i++) {
        uint off = pio_add_program(pios[i], &quad_program);
        quad_program_init(pios[i], 0, off, i == 0 ? PIN_ENC_L_A : PIN_ENC_R_A);
    }
}

void encoders_poll(void) {
    for (int i = 0; i < 2; i++) {
        while (!pio_sm_is_rx_fifo_empty(pios[i], 0)) {
            uint8_t v = pio_sm_get(pios[i], 0) & 3;
            int d = QDELTA[(prev[i] << 2) | v];
            prev[i] = v;
            if (i == 0 ? ENC_L_INVERT : ENC_R_INVERT) d = -d;
            total[i] += d;
        }
    }
}

void encoder_get_and_reset_deltas(int16_t *l, int16_t *r) {
    encoders_poll();
    int32_t dl = total[0] - reported[0], dr = total[1] - reported[1];
    reported[0] = total[0]; reported[1] = total[1];
    *l = (int16_t)dl; *r = (int16_t)dr;
}

void encoders_total(int32_t *l, int32_t *r) { *l = total[0]; *r = total[1]; }
