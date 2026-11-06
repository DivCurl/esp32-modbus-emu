#include "leds.h"

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_timer.h"

#include "app_config.h"

// A frame lasts a few milliseconds, too short to see. The off time keeps the LED
// blinking rather than solid while a master polls continuously.
#define LED_ON_MS  40
#define LED_OFF_MS 40

typedef struct {
    gpio_num_t pin;
    bool on;
    int64_t on_since_ms;
} led_t;

static led_t s_tx = {.pin = EMU_PIN_LED_TX, .on_since_ms = -(LED_ON_MS + LED_OFF_MS)};
static led_t s_rx = {.pin = EMU_PIN_LED_RX, .on_since_ms = -(LED_ON_MS + LED_OFF_MS)};

static int64_t now_ms(void) {
    return esp_timer_get_time() / 1000;
}

static void pulse(led_t *led) {
    int64_t now = now_ms();
    if (now - led->on_since_ms >= LED_ON_MS + LED_OFF_MS) {
        gpio_set_level(led->pin, 1);
        led->on = true;
        led->on_since_ms = now;
    }
}

static void service(led_t *led) {
    if (led->on && now_ms() - led->on_since_ms >= LED_ON_MS) {
        gpio_set_level(led->pin, 0);
        led->on = false;
    }
}

void leds_init(void) {
    gpio_config_t config = {
        .pin_bit_mask = (1ULL << EMU_PIN_LED_TX) | (1ULL << EMU_PIN_LED_RX),
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&config));
}

void leds_pulse_tx(void) {
    pulse(&s_tx);
}

void leds_pulse_rx(void) {
    pulse(&s_rx);
}

void leds_service(void) {
    service(&s_tx);
    service(&s_rx);
}
