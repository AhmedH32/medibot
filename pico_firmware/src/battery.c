#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "battery.h"
#include "robot_config.h"

static uint8_t pct;

uint16_t battery_read_filtered(void) {
    uint32_t acc = 0;
    for (int i = 0; i < 64; i++) { acc += adc_read(); sleep_us(50); }
    return (uint16_t)(acc >> 6);
}

void battery_update(void) {
    uint16_t code = battery_read_filtered();
    if (code <= BATT_CODE_EMPTY)      pct = 0;
    else if (code >= BATT_CODE_FULL)  pct = 100;
    else pct = (uint8_t)(((uint32_t)(code - BATT_CODE_EMPTY) * 100u + (BATT_CODE_FULL - BATT_CODE_EMPTY) / 2)
                         / (BATT_CODE_FULL - BATT_CODE_EMPTY));
}

void battery_init(void) {
    adc_init();
    adc_gpio_init(PIN_BATT_ADC);
    adc_select_input(0);
    battery_update();
}

uint8_t battery_percent(void) { return pct; }
