#include "settings.h"

#include <string.h>

#include "nvs.h"
#include "nvs_flash.h"

#include "app_config.h"

#define NVS_NAMESPACE  "emu"
#define NVS_KEY_SERIAL "serial"
#define NVS_KEY_WIFI   "wifi"

void settings_init(void) {
    esp_err_t err = nvs_flash_init();

    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

esp_err_t settings_get_blob(const char *key, void *buf, size_t *size) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);

    if (err == ESP_OK) {
        err = nvs_get_blob(handle, key, buf, size);
        nvs_close(handle);
    }
    return err;
}

esp_err_t settings_set_blob(const char *key, const void *buf, size_t size) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);

    if (err == ESP_OK) {
        err = nvs_set_blob(handle, key, buf, size);
        if (err == ESP_OK) {
            err = nvs_commit(handle);
        }
        nvs_close(handle);
    }
    return err;
}

bool serial_cfg_valid(const serial_cfg_t *cfg) {
    return cfg->baud >= 1200 && cfg->baud <= 921600 &&
           (cfg->parity == 'N' || cfg->parity == 'E' || cfg->parity == 'O') &&
           (cfg->stop_bits == 1 || cfg->stop_bits == 2) &&
           cfg->slave_id >= 1 && cfg->slave_id <= 247;
}

void settings_load_serial(serial_cfg_t *cfg) {
    size_t size = sizeof(*cfg);

    if (settings_get_blob(NVS_KEY_SERIAL, cfg, &size) != ESP_OK || size != sizeof(*cfg) ||
        !serial_cfg_valid(cfg)) {
        *cfg = (serial_cfg_t){
            .baud = EMU_DEFAULT_BAUD,
            .parity = EMU_DEFAULT_PARITY,
            .stop_bits = EMU_DEFAULT_STOP,
            .slave_id = EMU_DEFAULT_SLAVE,
        };
    }
}

esp_err_t settings_save_serial(const serial_cfg_t *cfg) {
    return settings_set_blob(NVS_KEY_SERIAL, cfg, sizeof(*cfg));
}

bool settings_load_wifi(wifi_creds_t *creds) {
    size_t size = sizeof(*creds);

    if (settings_get_blob(NVS_KEY_WIFI, creds, &size) == ESP_OK && size == sizeof(*creds) &&
        creds->ssid[0] != '\0') {
        return true;
    }

    memset(creds, 0, sizeof(*creds));
    strlcpy(creds->ssid, EMU_WIFI_SSID, sizeof(creds->ssid));
    strlcpy(creds->password, EMU_WIFI_PASSWORD, sizeof(creds->password));
    return creds->ssid[0] != '\0';
}

esp_err_t settings_save_wifi(const wifi_creds_t *creds) {
    return settings_set_blob(NVS_KEY_WIFI, creds, sizeof(*creds));
}
