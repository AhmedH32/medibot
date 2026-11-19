#include "pico/stdlib.h"
#include "sonar.h"
#include "robot_config.h"

static const uint8_t ECHO[2] = {PIN_SONAR_L_ECHO, PIN_SONAR_R_ECHO};
static volatile uint8_t  dist[2] = {255, 255};
static volatile bool     got[2];
static volatile uint32_t t_rise[2];
static volatile int      active;            // which echo is accepted in this 30 ms window

static void echo_cb(uint gpio, uint32_t events) {
    int i = (gpio == ECHO[0]) ? 0 : 1;
    if (i != active) return;                // the shared trigger fires both sensors; ignore the other
    if (events & GPIO_IRQ_EDGE_RISE) {
        t_rise[i] = time_us_32();
    } else if (events & GPIO_IRQ_EDGE_FALL) {
        uint32_t cm = (time_us_32() - t_rise[i]) / 58;
        dist[i] = cm > 255 ? 255 : (uint8_t)cm;
        got[i] = true;
    }
}

void sonar_init(void) {
    gpio_init(PIN_SONAR_TRIG); gpio_set_dir(PIN_SONAR_TRIG, GPIO_OUT); gpio_put(PIN_SONAR_TRIG, 0);
    for (int i = 0; i < 2; i++) { gpio_init(ECHO[i]); gpio_set_dir(ECHO[i], GPIO_IN); }
    gpio_set_irq_enabled_with_callback(ECHO[0], GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &echo_cb);
    gpio_set_irq_enabled(ECHO[1], GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
}

void sonar_tick(void) {
    if (!got[active]) dist[active] = 255;   // no echo in the last window -> clear
    active ^= 1;
    got[active] = false;
    gpio_put(PIN_SONAR_TRIG, 1);
    busy_wait_us_32(10);
    gpio_put(PIN_SONAR_TRIG, 0);
}

uint8_t sonar_min_cm(void) { return dist[0] < dist[1] ? dist[0] : dist[1]; }
