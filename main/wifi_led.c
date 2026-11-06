#include "wifi_led.h"

#define BLINK_PERIOD_MS 2000

#define FAULT_PERIOD_MS 1500
#define FAULT_FLASH_MS  150

wifi_led_state_t wifi_led_state(bool connected, bool have_creds, bool gave_up, uint32_t down_ms) {
    if (connected) {
        return WIFI_LED_CONNECTED;
    }
    if (!have_creds || gave_up || down_ms >= WIFI_LED_FAULT_AFTER_MS) {
        return WIFI_LED_FAULT;
    }
    return WIFI_LED_CONNECTING;
}

bool wifi_led_level(wifi_led_state_t state, uint32_t now_ms) {
    switch (state) {
        case WIFI_LED_CONNECTED:
            return true;
        case WIFI_LED_CONNECTING:
            return now_ms % BLINK_PERIOD_MS < BLINK_PERIOD_MS / 2;
        case WIFI_LED_FAULT: {
            uint32_t phase = now_ms % FAULT_PERIOD_MS;
            return phase < FAULT_FLASH_MS ||
                   (phase >= 2 * FAULT_FLASH_MS && phase < 3 * FAULT_FLASH_MS);
        }
        default:
            return false;
    }
}
