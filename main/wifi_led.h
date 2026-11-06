// WiFi status LED logic. No ESP-IDF dependencies, so it can be tested on a host.
#pragma once

#include <stdbool.h>
#include <stdint.h>

// Time without a connection before it counts as a fault, even if attempts seem to be
// going well. Catches joining the network but never getting an IP address.
#define WIFI_LED_FAULT_AFTER_MS 60000

typedef enum {
    WIFI_LED_CONNECTING,  // slow blink
    WIFI_LED_CONNECTED,   // solid
    WIFI_LED_FAULT,       // double flash, then a pause
} wifi_led_state_t;

// gave_up is true once repeated attempts have failed. down_ms is the time since the
// connection was lost, or since connecting began.
wifi_led_state_t wifi_led_state(bool connected, bool have_creds, bool gave_up, uint32_t down_ms);

// True when the LED should be lit at this time
bool wifi_led_level(wifi_led_state_t state, uint32_t now_ms);
