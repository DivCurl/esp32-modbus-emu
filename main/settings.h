// Persistent settings, stored as NVS blobs.
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

typedef struct {
    uint32_t baud;
    uint8_t parity;  // 'N', 'E' or 'O'. Data bits are always 8, as RTU requires.
    uint8_t stop_bits;
    uint8_t slave_id;
} serial_cfg_t;

typedef struct {
    char ssid[33];
    char password[65];
} wifi_creds_t;

void settings_init(void);

bool serial_cfg_valid(const serial_cfg_t *cfg);

// Loaders fall back to compile-time defaults when nothing valid is stored
void settings_load_serial(serial_cfg_t *cfg);
esp_err_t settings_save_serial(const serial_cfg_t *cfg);

// False if there are no credentials at all, stored or compiled in
bool settings_load_wifi(wifi_creds_t *creds);
esp_err_t settings_save_wifi(const wifi_creds_t *creds);

// *size is the buffer size on entry and the stored size on success. A NULL buf
// only queries the size.
esp_err_t settings_get_blob(const char *key, void *buf, size_t *size);
esp_err_t settings_set_blob(const char *key, const void *buf, size_t size);
