// WiFi station with a fallback setup access point.
#pragma once

#include <stdbool.h>

#include "esp_err.h"

#include "settings.h"

typedef struct {
    bool connected;
    bool setup_ap_active;
    char ssid[33];
    char ip[16];
    int rssi;
} wifi_status_t;

void wifi_start(void);

// Saves the credentials and reconnects with them shortly after, without a reboot
esp_err_t wifi_set_credentials(const wifi_creds_t *creds);

void wifi_get_status(wifi_status_t *status);
