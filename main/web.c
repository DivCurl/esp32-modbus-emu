#include "web.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cJSON.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"

#include "app_config.h"
#include "modbus_slave.h"
#include "map_store.h"
#include "settings.h"
#include "wifi.h"

#define MAX_BODY_LEN        512
#define RULES_PER_CHUNK     8
#define RULE_JSON_MAX       220
#define VALUES_MAX          256
#define FRAME_JSON_MAX      (MB_CAPTURE_BYTES * 2 + 40)

static const char *TAG = "WEB";

extern const char index_html_start[] asm("_binary_index_html_start");
extern const char index_html_end[] asm("_binary_index_html_end");

static esp_err_t send_json_error(httpd_req_t *req, const char *status, const char *message) {
    char body[96];

    snprintf(body, sizeof(body), "{\"error\":\"%s\"}", message);
    httpd_resp_set_status(req, status);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, body);
}

static esp_err_t send_ok(httpd_req_t *req) {
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"ok\":true}");
}

static esp_err_t send_map_result(httpd_req_t *req, int err) {
    switch (err) {
        case MAP_OK:
            return send_ok(req);
        case MAP_ERR_INVALID:
            return send_json_error(req, "400 Bad Request", "Invalid settings");
        case MAP_ERR_OVERLAP:
            return send_json_error(req, "400 Bad Request", "That range overlaps an existing block");
        case MAP_ERR_UNMAPPED:
            return send_json_error(req, "400 Bad Request", "Part of the range isn't inside a block");
        case MAP_ERR_FULL:
            return send_json_error(req, "400 Bad Request",
                                   "Out of space: too many blocks or rules, or the pool is full");
        case MAP_ERR_NOT_FOUND:
            return send_json_error(req, "400 Bad Request", "No block at that address");
        default:
            return send_json_error(req, "500 Internal Server Error", "Could not save to flash");
    }
}

// Browsers can't add a custom header to a cross-site request without CORS approval, so
// requiring this one even with no password stops other websites posting to the device.
// The '=' prefix keeps an empty password distinct from a missing header.
static bool authorized(httpd_req_t *req) {
    char given[72];

    return httpd_req_get_hdr_value_str(req, "X-Emu-Password", given, sizeof(given)) == ESP_OK &&
           given[0] == '=' && strcmp(given + 1, EMU_WEB_PASSWORD) == 0;
}

// Checks authorization and parses the body. On NULL the error response has been sent.
static cJSON *read_json(httpd_req_t *req) {
    char body[MAX_BODY_LEN + 1];
    size_t received = 0;

    if (!authorized(req)) {
        send_json_error(req, "401 Unauthorized", "Wrong password");
        return NULL;
    }

    if (req->content_len > MAX_BODY_LEN) {
        send_json_error(req, "400 Bad Request", "Request too large");
        return NULL;
    }

    while (received < req->content_len) {
        int ret = httpd_req_recv(req, body + received, req->content_len - received);
        if (ret <= 0) {
            send_json_error(req, "400 Bad Request", "Could not read request");
            return NULL;
        }
        received += ret;
    }
    body[received] = '\0';

    cJSON *json = cJSON_Parse(body);
    if (json == NULL) {
        send_json_error(req, "400 Bad Request", "Invalid JSON");
    }
    return json;
}

static bool get_int(const cJSON *json, const char *name, int *out) {
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(json, name);

    if (!cJSON_IsNumber(item)) {
        return false;
    }
    *out = item->valueint;
    return true;
}

static int get_int_or(const cJSON *json, const char *name, int fallback) {
    int value = fallback;

    get_int(json, name, &value);
    return value;
}

static double get_double_or(const cJSON *json, const char *name, double fallback) {
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(json, name);

    return cJSON_IsNumber(item) ? item->valuedouble : fallback;
}

static esp_err_t index_get(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, index_html_start, index_html_end - index_html_start - 1);
}

static esp_err_t status_get(httpd_req_t *req) {
    static mb_stats_t stats;  // too big for the stack, and handlers run one at a time
    serial_cfg_t serial;
    wifi_status_t wifi;

    modbus_slave_get_stats(&stats);
    modbus_slave_get_config(&serial);
    wifi_get_status(&wifi);

    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    char parity[2] = {(char)serial.parity, '\0'};

    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "uptime_s", now_ms / 1000);
    cJSON_AddNumberToObject(root, "free_heap", esp_get_free_heap_size());

    cJSON *wifi_json = cJSON_AddObjectToObject(root, "wifi");
    cJSON_AddBoolToObject(wifi_json, "connected", wifi.connected);
    cJSON_AddBoolToObject(wifi_json, "setup_ap", wifi.setup_ap_active);
    cJSON_AddStringToObject(wifi_json, "ssid", wifi.ssid);
    cJSON_AddStringToObject(wifi_json, "ip", wifi.ip);
    cJSON_AddNumberToObject(wifi_json, "rssi", wifi.rssi);

    cJSON *serial_json = cJSON_AddObjectToObject(root, "serial");
    cJSON_AddNumberToObject(serial_json, "baud", serial.baud);
    cJSON_AddStringToObject(serial_json, "parity", parity);
    cJSON_AddNumberToObject(serial_json, "stop_bits", serial.stop_bits);
    cJSON_AddNumberToObject(serial_json, "slave_id", serial.slave_id);

    cJSON *stats_json = cJSON_AddObjectToObject(root, "stats");
    cJSON_AddNumberToObject(stats_json, "rx_frames", stats.rx_frames);
    cJSON_AddNumberToObject(stats_json, "tx_frames", stats.tx_frames);
    cJSON_AddNumberToObject(stats_json, "exceptions", stats.exceptions);
    cJSON_AddNumberToObject(stats_json, "crc_errors", stats.crc_errors);
    cJSON_AddNumberToObject(stats_json, "parity_errors", stats.parity_errors);
    cJSON_AddNumberToObject(stats_json, "framing_errors", stats.framing_errors);
    cJSON_AddNumberToObject(stats_json, "breaks", stats.breaks);
    cJSON_AddNumberToObject(stats_json, "overruns", stats.overruns);
    cJSON_AddNumberToObject(stats_json, "other_slave", stats.other_slave);

    cJSON *recent = cJSON_AddArrayToObject(root, "recent");
    for (int i = stats.recent_count - 1; i >= 0; i--) {
        cJSON *entry = cJSON_CreateObject();
        cJSON_AddNumberToObject(entry, "fc", stats.recent[i].request.fc);
        cJSON_AddNumberToObject(entry, "addr", stats.recent[i].request.addr);
        cJSON_AddNumberToObject(entry, "qty", stats.recent[i].request.qty);
        cJSON_AddNumberToObject(entry, "exception", stats.recent[i].request.exception);
        cJSON_AddNumberToObject(entry, "age_ms", now_ms - stats.recent[i].time_ms);
        cJSON_AddItemToArray(recent, entry);
    }

    char *body = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    if (body == NULL) {
        return send_json_error(req, "500 Internal Server Error", "Out of memory");
    }

    httpd_resp_set_type(req, "application/json");
    esp_err_t err = httpd_resp_sendstr(req, body);
    cJSON_free(body);
    return err;
}

// Entries are [time_ms, len, flags, hex], oldest first. Flags are the MB_CAP_ values.
static esp_err_t frames_get(httpd_req_t *req) {
    static mb_capture_t frames[MB_CAPTURE_MAX];
    size_t count = modbus_slave_get_capture(frames);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send_chunk(req, "[", 1);

    for (size_t i = 0; i < count; i++) {
        char line[FRAME_JSON_MAX];
        size_t kept = frames[i].len < MB_CAPTURE_BYTES ? frames[i].len : MB_CAPTURE_BYTES;
        int len = snprintf(line, sizeof(line), "%s[%lu,%u,%u,\"", i == 0 ? "" : ",",
                           (unsigned long)frames[i].time_ms, frames[i].len, frames[i].flags);

        for (size_t b = 0; b < kept; b++) {
            len += snprintf(line + len, sizeof(line) - len, "%02X", frames[i].data[b]);
        }
        len += snprintf(line + len, sizeof(line) - len, "\"]");

        if (httpd_resp_send_chunk(req, line, len) != ESP_OK) {
            return ESP_FAIL;
        }
    }

    httpd_resp_send_chunk(req, "]", 1);
    return httpd_resp_send_chunk(req, NULL, 0);
}

static esp_err_t stats_reset_post(httpd_req_t *req) {
    cJSON *json = read_json(req);
    if (json == NULL) {
        return ESP_OK;
    }
    cJSON_Delete(json);

    modbus_slave_reset_stats();
    return send_ok(req);
}

// {"strict": bool, "capacity": {...}, "blocks": [[kind, start, count, flags]], "rules": [[kind, start,
// count, type, order, mode, value, value_step, min, max, period_ms, phase_deg,
// phase_step_deg, duty_pct]]}
static esp_err_t map_get(httpd_req_t *req) {
    static map_layout_t layout;  // too big for the stack, and handlers run one at a time
    static char chunk[RULES_PER_CHUNK * RULE_JSON_MAX + 64];
    int len;

    store_get_layout(&layout);
    httpd_resp_set_type(req, "application/json");

    len = snprintf(chunk, sizeof(chunk),
                   "{\"strict\":%s,\"capacity\":{\"regs_used\":%u,\"regs_max\":%u,\"bits_used\":%u,"
                   "\"bits_max\":%u,\"blocks\":%u,\"blocks_max\":%u,\"rules\":%u,"
                   "\"rules_max\":%u},\"blocks\":[",
                   layout.strict ? "true" : "false", (unsigned)layout.regs_used, MAP_REG_POOL, (unsigned)layout.bits_used,
                   MAP_BIT_POOL, (unsigned)layout.block_count, MAP_MAX_BLOCKS,
                   (unsigned)layout.rule_count, MAP_MAX_RULES);
    httpd_resp_send_chunk(req, chunk, len);

    for (size_t i = 0; i < layout.block_count; i++) {
        const map_block_t *block = &layout.blocks[i];
        len = snprintf(chunk, sizeof(chunk), "%s[%u,%u,%lu,%u]", i == 0 ? "" : ",",
                       block->kind, block->start, (unsigned long)block->count, block->flags);
        httpd_resp_send_chunk(req, chunk, len);
    }
    httpd_resp_send_chunk(req, "],\"rules\":[", HTTPD_RESP_USE_STRLEN);

    for (size_t first = 0; first < layout.rule_count; first += RULES_PER_CHUNK) {
        len = 0;
        for (size_t i = first; i < layout.rule_count && i < first + RULES_PER_CHUNK; i++) {
            const map_rule_t *rule = &layout.rules[i];
            len += snprintf(chunk + len, sizeof(chunk) - len,
                            "%s[%u,%u,%lu,%u,%u,%u,%.17g,%.17g,%.17g,%.17g,%lu,%d,%d,%u]",
                            i == 0 ? "" : ",", rule->kind, rule->start,
                            (unsigned long)rule->count, rule->type, rule->order, rule->mode,
                            rule->value, rule->value_step, rule->min, rule->max,
                            (unsigned long)rule->period_ms, rule->phase_deg,
                            rule->phase_step_deg, rule->duty_pct);
        }
        if (httpd_resp_send_chunk(req, chunk, len) != ESP_OK) {
            return ESP_FAIL;
        }
    }

    httpd_resp_send_chunk(req, "]}", 2);
    return httpd_resp_send_chunk(req, NULL, 0);
}

static int query_int(const char *query, const char *key, int fallback) {
    char value[12];
    return httpd_query_key_value(query, key, value, sizeof(value)) == ESP_OK ? atoi(value)
                                                                             : fallback;
}

// GET /api/values?kind=3&start=3000&count=100 returns raw registers, or 0 and 1 for bits,
// with null for addresses outside every block
static esp_err_t values_get(httpd_req_t *req) {
    static uint16_t values[VALUES_MAX];
    static bool defined[VALUES_MAX];
    static char body[VALUES_MAX * 7 + 64];
    char query[64] = "";

    httpd_req_get_url_query_str(req, query, sizeof(query));
    int kind = query_int(query, "kind", -1);
    int start = query_int(query, "start", 0);
    int count = query_int(query, "count", 100);

    if (!map_kind_valid(kind) || start < 0 || start > 0xFFFF || count < 1 ||
        count > VALUES_MAX) {
        return send_json_error(req, "400 Bad Request", "Invalid kind, start or count");
    }

    store_peek(kind, start, count, values, defined);

    int len = snprintf(body, sizeof(body), "{\"kind\":%d,\"start\":%d,\"values\":[",
                       kind, start);
    for (int i = 0; i < count; i++) {
        len += defined[i] ? snprintf(body + len, sizeof(body) - len, "%s%u", i ? "," : "",
                                     values[i])
                          : snprintf(body + len, sizeof(body) - len, "%snull", i ? "," : "");
    }
    len += snprintf(body + len, sizeof(body) - len, "]}");

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, body, len);
}

static bool get_bool(const cJSON *json, const char *name) {
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(json, name);
    return cJSON_IsTrue(item) || (cJSON_IsNumber(item) && item->valueint != 0);
}

static bool in_range(int value, int low, int high) {
    return value >= low && value <= high;
}

static esp_err_t blocks_post(httpd_req_t *req) {
    cJSON *json = read_json(req);
    if (json == NULL) {
        return ESP_OK;
    }

    int kind;
    int start;
    int count;
    bool complete = get_int(json, "kind", &kind) && get_int(json, "start", &start) &&
                    get_int(json, "count", &count);
    bool read_only = get_bool(json, "read_only");
    cJSON_Delete(json);

    if (!complete || !in_range(start, 0, 0xFFFF) || !in_range(count, 1, 0x10000)) {
        return send_map_result(req, MAP_ERR_INVALID);
    }
    return send_map_result(req, store_add_block(kind, start, count,
                                                read_only ? MAP_BLOCK_READ_ONLY : 0));
}

static esp_err_t blocks_delete_post(httpd_req_t *req) {
    cJSON *json = read_json(req);
    if (json == NULL) {
        return ESP_OK;
    }

    int kind;
    int addr;
    bool complete = get_int(json, "kind", &kind) && get_int(json, "addr", &addr);
    cJSON_Delete(json);

    if (!complete || !in_range(addr, 0, 0xFFFF)) {
        return send_map_result(req, MAP_ERR_INVALID);
    }
    return send_map_result(req, store_delete_block(kind, addr));
}

static esp_err_t rules_post(httpd_req_t *req) {
    cJSON *json = read_json(req);
    if (json == NULL) {
        return ESP_OK;
    }

    int kind;
    int start;
    int count;
    int mode;
    bool complete = get_int(json, "kind", &kind) && get_int(json, "start", &start) &&
                    get_int(json, "count", &count) && get_int(json, "mode", &mode);
    int type = get_int_or(json, "type", map_kind_is_bits(kind) ? MAP_TYPE_BOOL : MAP_TYPE_UINT16);
    int order = get_int_or(json, "order", MAP_ORDER_ABCD);
    int period = get_int_or(json, "period_ms", 0);
    int phase = get_int_or(json, "phase_deg", 0);
    int phase_step = get_int_or(json, "phase_step_deg", 0);
    int duty = get_int_or(json, "duty_pct", 50);

    map_rule_t rule = {
        .value = get_double_or(json, "value", 0),
        .value_step = get_double_or(json, "value_step", 0),
        .min = get_double_or(json, "min", 0),
        .max = get_double_or(json, "max", 0),
    };
    cJSON_Delete(json);

    // Range checked here because the fields narrow when stored
    if (!complete || !in_range(kind, 0, 255) || !in_range(start, 0, 0xFFFF) ||
        !in_range(count, 1, 0x10000) || !in_range(mode, 0, 255) || !in_range(type, 0, 255) ||
        !in_range(order, 0, 255) || !in_range(period, 0, 86400000) ||
        !in_range(phase, -360, 360) || !in_range(phase_step, -360, 360) ||
        !in_range(duty, 0, 100)) {
        return send_map_result(req, MAP_ERR_INVALID);
    }

    rule.kind = kind;
    rule.start = start;
    rule.count = count;
    rule.mode = mode;
    rule.type = type;
    rule.order = order;
    rule.period_ms = period;
    rule.phase_deg = phase;
    rule.phase_step_deg = phase_step;
    rule.duty_pct = duty;
    return send_map_result(req, store_apply_rule(&rule));
}

static esp_err_t rules_clear_post(httpd_req_t *req) {
    cJSON *json = read_json(req);
    if (json == NULL) {
        return ESP_OK;
    }

    int kind;
    int start;
    int count;
    bool complete = get_int(json, "kind", &kind) && get_int(json, "start", &start) &&
                    get_int(json, "count", &count);
    cJSON_Delete(json);

    if (!complete || !in_range(kind, 0, 255) || !in_range(start, 0, 0xFFFF) ||
        !in_range(count, 1, 0x10000)) {
        return send_map_result(req, MAP_ERR_INVALID);
    }
    return send_map_result(req, store_clear_rules(kind, start, count));
}

static esp_err_t map_mode_post(httpd_req_t *req) {
    cJSON *json = read_json(req);
    if (json == NULL) {
        return ESP_OK;
    }

    const cJSON *strict = cJSON_GetObjectItemCaseSensitive(json, "strict");
    bool valid = cJSON_IsBool(strict);
    bool value = cJSON_IsTrue(strict);
    cJSON_Delete(json);

    if (!valid) {
        return send_map_result(req, MAP_ERR_INVALID);
    }
    return send_map_result(req, store_set_strict(value));
}

static esp_err_t map_reset_post(httpd_req_t *req) {
    cJSON *json = read_json(req);
    if (json == NULL) {
        return ESP_OK;
    }
    cJSON_Delete(json);

    return send_map_result(req, store_reset_defaults());
}

static esp_err_t serial_post(httpd_req_t *req) {
    cJSON *json = read_json(req);
    if (json == NULL) {
        return ESP_OK;
    }

    serial_cfg_t cfg;
    modbus_slave_get_config(&cfg);

    const cJSON *parity = cJSON_GetObjectItemCaseSensitive(json, "parity");
    if (cJSON_IsString(parity) && strlen(parity->valuestring) == 1) {
        cfg.parity = parity->valuestring[0];
    }

    int baud = get_int_or(json, "baud", cfg.baud);
    int stop_bits = get_int_or(json, "stop_bits", cfg.stop_bits);
    int slave_id = get_int_or(json, "slave_id", cfg.slave_id);
    cJSON_Delete(json);

    // Range checked here because the fields narrow when stored
    if (baud < 0 || stop_bits < 0 || stop_bits > 2 || slave_id < 0 || slave_id > 247) {
        return send_json_error(req, "400 Bad Request", "Invalid serial settings");
    }

    cfg.baud = baud;
    cfg.stop_bits = stop_bits;
    cfg.slave_id = slave_id;

    if (!serial_cfg_valid(&cfg)) {
        return send_json_error(req, "400 Bad Request", "Invalid serial settings");
    }

    if (settings_save_serial(&cfg) != ESP_OK) {
        return send_json_error(req, "500 Internal Server Error", "Could not save to flash");
    }

    modbus_slave_apply(&cfg);
    return send_ok(req);
}

static esp_err_t wifi_post(httpd_req_t *req) {
    cJSON *json = read_json(req);
    if (json == NULL) {
        return ESP_OK;
    }

    wifi_creds_t creds = {0};
    const cJSON *ssid = cJSON_GetObjectItemCaseSensitive(json, "ssid");
    const cJSON *password = cJSON_GetObjectItemCaseSensitive(json, "password");
    bool valid = cJSON_IsString(ssid) && cJSON_IsString(password) &&
                 strlen(ssid->valuestring) >= 1 &&
                 strlen(ssid->valuestring) < sizeof(creds.ssid) &&
                 strlen(password->valuestring) < sizeof(creds.password);

    if (valid) {
        strlcpy(creds.ssid, ssid->valuestring, sizeof(creds.ssid));
        strlcpy(creds.password, password->valuestring, sizeof(creds.password));
    }
    cJSON_Delete(json);

    if (!valid) {
        return send_json_error(req, "400 Bad Request", "Invalid WiFi settings");
    }

    if (wifi_set_credentials(&creds) != ESP_OK) {
        return send_json_error(req, "500 Internal Server Error", "Could not save to flash");
    }
    return send_ok(req);
}

void web_start(void) {
    static const httpd_uri_t routes[] = {
        {.uri = "/", .method = HTTP_GET, .handler = index_get},
        {.uri = "/api/status", .method = HTTP_GET, .handler = status_get},
        {.uri = "/api/frames", .method = HTTP_GET, .handler = frames_get},
        {.uri = "/api/stats/reset", .method = HTTP_POST, .handler = stats_reset_post},
        {.uri = "/api/map", .method = HTTP_GET, .handler = map_get},
        {.uri = "/api/values", .method = HTTP_GET, .handler = values_get},
        {.uri = "/api/blocks", .method = HTTP_POST, .handler = blocks_post},
        {.uri = "/api/blocks/delete", .method = HTTP_POST, .handler = blocks_delete_post},
        {.uri = "/api/rules", .method = HTTP_POST, .handler = rules_post},
        {.uri = "/api/rules/clear", .method = HTTP_POST, .handler = rules_clear_post},
        {.uri = "/api/map/reset", .method = HTTP_POST, .handler = map_reset_post},
        {.uri = "/api/map/mode", .method = HTTP_POST, .handler = map_mode_post},
        {.uri = "/api/serial", .method = HTTP_POST, .handler = serial_post},
        {.uri = "/api/wifi", .method = HTTP_POST, .handler = wifi_post},
    };

    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    config.stack_size = 8192;
    config.core_id = 0;
    config.max_uri_handlers = sizeof(routes) / sizeof(routes[0]);
    config.lru_purge_enable = true;

    ESP_ERROR_CHECK(httpd_start(&server, &config));

    for (size_t i = 0; i < sizeof(routes) / sizeof(routes[0]); i++) {
        ESP_ERROR_CHECK(httpd_register_uri_handler(server, &routes[i]));
    }

    ESP_LOGI(TAG, "Web server started");
}
