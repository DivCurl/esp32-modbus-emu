#include "wifi.h"

#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"

#include "app_config.h"
#include "wifi_led.h"

#define RETRY_DELAY_MS       5000
#define RETRY_DELAY_AP_MS    30000  // scanning takes the radio away from the setup network
#define FAILS_BEFORE_AP      5
#define AP_LINGER_MS         60000  // long enough to read the new address from the setup network
#define AP_CHANNEL           1
#define AP_MAX_CONNECTIONS   2
#define RECONFIGURE_DELAY_MS 500
#define LED_TICK_MS          50

_Static_assert(sizeof(EMU_AP_PASSWORD) > 8, "EMU_AP_PASSWORD needs at least 8 characters");

static const char *TAG = "WIFI";

static esp_netif_t *s_sta_netif;
static esp_timer_handle_t s_retry_timer;
static esp_timer_handle_t s_ap_off_timer;
static esp_timer_handle_t s_led_timer;

static wifi_creds_t s_creds;
static volatile bool s_have_creds;
static volatile bool s_connected;
static volatile bool s_ap_active;
static volatile bool s_reconfigure;
static volatile int s_fails;
static volatile uint32_t s_down_since_ms;

static esp_err_t set_sta_config(void);

static uint32_t now_ms(void) {
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static void led_cb(void *arg) {
    uint32_t now = now_ms();
    wifi_led_state_t state = wifi_led_state(s_connected, s_have_creds,
                                            s_fails >= FAILS_BEFORE_AP, now - s_down_since_ms);

    gpio_set_level(EMU_PIN_LED_WIFI, wifi_led_level(state, now));
}

static void led_start(void) {
    gpio_config_t config = {
        .pin_bit_mask = 1ULL << EMU_PIN_LED_WIFI,
        .mode = GPIO_MODE_OUTPUT,
    };
    const esp_timer_create_args_t timer_args = {.callback = led_cb, .name = "wifi_led"};

    ESP_ERROR_CHECK(gpio_config(&config));
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &s_led_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(s_led_timer, LED_TICK_MS * 1000));
}

static void schedule_retry(uint32_t delay_ms) {
    // Already pending is fine: the running timer does the same job
    esp_timer_start_once(s_retry_timer, (uint64_t)delay_ms * 1000);
}

static void start_setup_ap(void) {
    if (s_ap_active) {
        return;
    }

    wifi_config_t config = {
        .ap = {
            .ssid = EMU_AP_SSID,
            .ssid_len = strlen(EMU_AP_SSID),
            .channel = AP_CHANNEL,
            .max_connection = AP_MAX_CONNECTIONS,
            .authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    strlcpy((char *)config.ap.password, EMU_AP_PASSWORD, sizeof(config.ap.password));

    // Keeps the station side, so the device rejoins by itself when the network returns
    ESP_ERROR_CHECK(esp_wifi_set_mode(s_have_creds ? WIFI_MODE_APSTA : WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &config));
    s_ap_active = true;
    ESP_LOGW(TAG, "Setup network \"%s\" active: http://192.168.4.1", EMU_AP_SSID);
}

static void apply_new_credentials(void) {
    wifi_mode_t mode = WIFI_MODE_NULL;

    esp_wifi_get_mode(&mode);
    if (mode == WIFI_MODE_AP) {
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    } else {
        esp_wifi_disconnect();
    }

    esp_err_t err = set_sta_config();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to apply credentials: %s", esp_err_to_name(err));
    }
}

static void retry_cb(void *arg) {
    if (s_reconfigure) {
        s_reconfigure = false;
        apply_new_credentials();
    }
    if (s_fails >= FAILS_BEFORE_AP) {
        start_setup_ap();
    }
    esp_wifi_connect();
}

static void ap_off_cb(void *arg) {
    if (s_connected && s_ap_active) {
        esp_wifi_set_mode(WIFI_MODE_STA);
        s_ap_active = false;
        ESP_LOGI(TAG, "Setup network stopped");
    }
}

static void event_handler(void *arg, esp_event_base_t base, int32_t id, void *data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        if (s_have_creds) {
            esp_wifi_connect();
        }
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *event = data;

        if (s_connected) {
            s_down_since_ms = now_ms();
        }
        s_connected = false;
        s_fails++;
        ESP_LOGW(TAG, "Disconnected, reason %d (attempt %d)", event->reason, s_fails);
        schedule_retry(s_ap_active ? RETRY_DELAY_AP_MS : RETRY_DELAY_MS);
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = data;

        s_connected = true;
        s_fails = 0;
        ESP_LOGI(TAG, "Web UI at http://" IPSTR, IP2STR(&event->ip_info.ip));
        if (s_ap_active) {
            esp_timer_stop(s_ap_off_timer);
            esp_timer_start_once(s_ap_off_timer, (uint64_t)AP_LINGER_MS * 1000);
        }
    }
}

static esp_err_t set_sta_config(void) {
    wifi_config_t config = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA_WPA2_PSK,
            .pmf_cfg = {.capable = true, .required = false},
            .scan_method = WIFI_ALL_CHANNEL_SCAN,
            .sort_method = WIFI_CONNECT_AP_BY_SIGNAL,
            .failure_retry_cnt = 0,
        },
    };

    strlcpy((char *)config.sta.ssid, s_creds.ssid, sizeof(config.sta.ssid));
    strlcpy((char *)config.sta.password, s_creds.password, sizeof(config.sta.password));
    return esp_wifi_set_config(WIFI_IF_STA, &config);
}

void wifi_start(void) {
    // Loaded first, so the LED doesn't show a fault for the moment before they're known
    s_have_creds = settings_load_wifi(&s_creds);
    s_down_since_ms = now_ms();
    led_start();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    s_sta_netif = esp_netif_create_default_wifi_sta();
    esp_netif_create_default_wifi_ap();
    ESP_ERROR_CHECK(esp_netif_set_hostname(s_sta_netif, EMU_HOSTNAME));

    wifi_init_config_t init_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init_config));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                        event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                        event_handler, NULL, NULL));

    const esp_timer_create_args_t retry_args = {.callback = retry_cb, .name = "wifi_retry"};
    const esp_timer_create_args_t ap_off_args = {.callback = ap_off_cb, .name = "wifi_ap_off"};
    ESP_ERROR_CHECK(esp_timer_create(&retry_args, &s_retry_timer));
    ESP_ERROR_CHECK(esp_timer_create(&ap_off_args, &s_ap_off_timer));

    if (s_have_creds) {
        ESP_LOGI(TAG, "Connecting to \"%s\"", s_creds.ssid);
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(set_sta_config());
    } else {
        ESP_LOGW(TAG, "No WiFi credentials");
        start_setup_ap();
    }

    ESP_ERROR_CHECK(esp_wifi_start());
}

esp_err_t wifi_set_credentials(const wifi_creds_t *creds) {
    esp_err_t err = settings_save_wifi(creds);
    if (err != ESP_OK) {
        return err;
    }

    s_creds = *creds;
    s_have_creds = true;
    s_fails = 0;
    s_down_since_ms = now_ms();
    s_reconfigure = true;

    // Deferred so the response to this request gets out before the link drops
    esp_timer_stop(s_retry_timer);
    schedule_retry(RECONFIGURE_DELAY_MS);
    return ESP_OK;
}

void wifi_get_status(wifi_status_t *status) {
    memset(status, 0, sizeof(*status));
    status->connected = s_connected;
    status->setup_ap_active = s_ap_active;
    strlcpy(status->ssid, s_creds.ssid, sizeof(status->ssid));

    if (status->connected) {
        esp_netif_ip_info_t info;
        wifi_ap_record_t record;

        if (esp_netif_get_ip_info(s_sta_netif, &info) == ESP_OK) {
            snprintf(status->ip, sizeof(status->ip), IPSTR, IP2STR(&info.ip));
        }
        if (esp_wifi_sta_get_ap_info(&record) == ESP_OK) {
            status->rssi = record.rssi;
        }
    }
}
