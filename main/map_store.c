#include "map_store.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "settings.h"

#define NVS_KEY_MAP       "map"
#define NVS_KEY_LEGACY    "regs"
#define NVS_KEY_STRICT    "strict"
#define MAP_STORE_VERSION 3
#define LEGACY_VERSION    2

#define EMU_TICK_MS       100
#define EMU_TASK_STACK    3072
#define EMU_TASK_PRIORITY 5
#define EMU_TASK_CORE     0  // Modbus has core 1 to itself

static const char *TAG = "MAP";

typedef struct {
    uint32_t version;
    uint32_t block_count;
    uint32_t rule_count;
} map_blob_header_t;

// Version 2 stored one entry per register. Only read, to migrate an older map.
typedef struct {
    uint16_t addr;
    uint8_t kind;
    uint8_t mode;
    uint8_t type;
    uint8_t order;
    uint8_t word;
    uint8_t reserved;
    int16_t phase_deg;
    uint32_t period_ms;
    double value;
    double min;
    double max;
} legacy_reg_t;

typedef struct {
    uint32_t version;
    uint32_t count;
    legacy_reg_t def[];
} legacy_blob_t;

static emu_map_t s_map;
static SemaphoreHandle_t s_lock;
static bool s_strict;

static void lock(void) {
    xSemaphoreTake(s_lock, portMAX_DELAY);
}

static void unlock(void) {
    xSemaphoreGive(s_lock);
}

static uint32_t now_ms(void) {
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static uint32_t random32(void) {
    return esp_random();
}

// Takes and releases the lock itself: the flash write must happen without holding it,
// or Modbus reads would stall behind it.
static int save_map(void) {
    lock();
    size_t blocks = s_map.block_count;
    size_t rules = s_map.rule_count;
    size_t size = sizeof(map_blob_header_t) + blocks * sizeof(map_block_t) +
                  rules * sizeof(map_rule_t);
    uint8_t *blob = malloc(size);
    if (blob != NULL) {
        map_blob_header_t header = {MAP_STORE_VERSION, blocks, rules};
        memcpy(blob, &header, sizeof(header));
        memcpy(blob + sizeof(header), s_map.blocks, blocks * sizeof(map_block_t));
        memcpy(blob + sizeof(header) + blocks * sizeof(map_block_t), s_map.rules,
               rules * sizeof(map_rule_t));
    }
    unlock();

    if (blob == NULL) {
        return MAP_ERR_STORAGE;
    }

    esp_err_t err = settings_set_blob(NVS_KEY_MAP, blob, size);
    free(blob);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save the map: %s", esp_err_to_name(err));
        return MAP_ERR_STORAGE;
    }
    return MAP_OK;
}

static void *read_blob(const char *key, size_t *size) {
    *size = 0;
    if (settings_get_blob(key, NULL, size) != ESP_OK || *size == 0) {
        return NULL;
    }
    void *blob = malloc(*size);
    if (blob != NULL && settings_get_blob(key, blob, size) != ESP_OK) {
        free(blob);
        blob = NULL;
    }
    return blob;
}

static bool load_map(void) {
    size_t size;
    uint8_t *blob = read_blob(NVS_KEY_MAP, &size);
    if (blob == NULL) {
        return false;
    }

    map_blob_header_t header;
    bool ok = size >= sizeof(header);
    if (ok) {
        memcpy(&header, blob, sizeof(header));
        ok = header.version == MAP_STORE_VERSION && header.block_count <= MAP_MAX_BLOCKS &&
             header.rule_count <= MAP_MAX_RULES &&
             size == sizeof(header) + header.block_count * sizeof(map_block_t) +
                     header.rule_count * sizeof(map_rule_t);
    }

    // Rebuilt through the normal calls, so a damaged blob can't break the map's invariants.
    // Strict while rebuilding, so rules use the saved blocks rather than adding more.
    map_clear(&s_map);
    s_map.strict = true;
    if (ok) {
        const map_block_t *blocks = (const map_block_t *)(blob + sizeof(header));
        const map_rule_t *rules = (const map_rule_t *)(blocks + header.block_count);

        for (uint32_t i = 0; i < header.block_count && ok; i++) {
            ok = map_add_block(&s_map, blocks[i].kind, blocks[i].start, blocks[i].count,
                               blocks[i].flags) == MAP_OK;
        }
        for (uint32_t i = 0; i < header.rule_count && ok; i++) {
            ok = map_apply_rule(&s_map, &rules[i], now_ms(), random32) == MAP_OK;
        }
    }

    free(blob);
    s_map.strict = s_strict;
    return ok;
}

static int wrap_phase(int deg) {
    deg %= 360;
    return deg < 0 ? deg + 360 : deg;
}

// Folds a run of registers into the current rule if they continue it
static bool extends(const map_rule_t *rule, const legacy_reg_t *reg) {
    if (reg->kind != rule->kind || reg->type != rule->type || reg->order != rule->order ||
        reg->mode != rule->mode || reg->addr != rule->start + map_rule_span(rule)) {
        return false;
    }

    switch (reg->mode) {
        case EMU_STATIC:
            return rule->count == 1 || reg->value == rule->value + rule->count * rule->value_step;
        case EMU_SINE:
            if (reg->min != rule->min || reg->max != rule->max ||
                reg->period_ms != rule->period_ms) {
                return false;
            }
            return rule->count == 1 ||
                   wrap_phase(reg->phase_deg) ==
                       wrap_phase(rule->phase_deg + (int)(rule->count % 360) * rule->phase_step_deg);
        default:
            return reg->min == rule->min && reg->max == rule->max &&
                   reg->period_ms == rule->period_ms;
    }
}

static bool migrate_legacy(void) {
    size_t size;
    legacy_blob_t *blob = read_blob(NVS_KEY_LEGACY, &size);
    if (blob == NULL) {
        return false;
    }

    bool ok = size >= sizeof(legacy_blob_t) && blob->version == LEGACY_VERSION &&
              size == sizeof(legacy_blob_t) + blob->count * sizeof(legacy_reg_t);

    map_clear(&s_map);
    s_map.strict = true;

    // Contiguous registers become blocks
    for (uint32_t i = 0; i < blob->count && ok;) {
        uint32_t j = i + 1;
        while (j < blob->count && blob->def[j].kind == blob->def[i].kind &&
               blob->def[j].addr == blob->def[i].addr + (j - i)) {
            j++;
        }
        ok = map_add_block(&s_map, blob->def[i].kind, blob->def[i].addr, j - i, 0) == MAP_OK;
        i = j;
    }

    // Each value's first register, folded into rules where settings continue
    map_rule_t rule;
    bool open = false;
    for (uint32_t i = 0; i <= blob->count && ok; i++) {
        const legacy_reg_t *reg = i < blob->count ? &blob->def[i] : NULL;
        if (reg != NULL && reg->word != 0) {
            continue;
        }

        if (open && reg != NULL && extends(&rule, reg)) {
            if (rule.count == 1) {
                rule.value_step = reg->value - rule.value;
                rule.phase_step_deg = wrap_phase(reg->phase_deg - rule.phase_deg);
            }
            rule.count++;
            continue;
        }

        if (open) {
            ok = map_apply_rule(&s_map, &rule, now_ms(), random32) == MAP_OK;
        }
        if (reg != NULL) {
            rule = (map_rule_t){
                .count = 1,
                .period_ms = reg->period_ms,
                .start = reg->addr,
                .kind = reg->kind,
                .type = reg->type,
                .order = reg->order,
                .mode = reg->mode,
                .duty_pct = 50,
                .phase_deg = reg->phase_deg,
                .value = reg->value,
                .min = reg->min,
                .max = reg->max,
            };
            open = true;
        }
    }

    free(blob);
    s_map.strict = s_strict;
    if (ok) {
        ESP_LOGI(TAG, "Migrated the older register map: %u blocks, %u rules",
                 (unsigned)s_map.block_count, (unsigned)s_map.rule_count);
    }
    return ok;
}

static void emu_task(void *arg) {
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        // One rule at a time, so a big map never holds Modbus reads up for long
        for (size_t i = 0;; i++) {
            lock();
            bool more = i < s_map.rule_count;
            if (more) {
                map_tick_rule(&s_map, i, now_ms(), random32);
            }
            unlock();
            if (!more) {
                break;
            }
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(EMU_TICK_MS));
    }
}

void store_init(void) {
    s_lock = xSemaphoreCreateMutex();
    configASSERT(s_lock != NULL);

    uint8_t strict = 0;
    size_t size = sizeof(strict);
    s_strict = settings_get_blob(NVS_KEY_STRICT, &strict, &size) == ESP_OK && strict;

    if (load_map()) {
        ESP_LOGI(TAG, "%s mode", s_strict ? "Strict" : "Open");
    ESP_LOGI(TAG, "Loaded %u blocks and %u rules", (unsigned)s_map.block_count,
                 (unsigned)s_map.rule_count);
    } else if (migrate_legacy()) {
        save_map();
    } else {
        map_load_defaults(&s_map);
        s_map.strict = s_strict;
        ESP_LOGI(TAG, "Using the default map");
    }

    BaseType_t created = xTaskCreatePinnedToCore(emu_task, "emu", EMU_TASK_STACK, NULL,
                                                 EMU_TASK_PRIORITY, NULL, EMU_TASK_CORE);
    configASSERT(created == pdPASS);
}

bool store_read_regs(uint8_t kind, uint16_t addr, uint16_t qty, uint16_t *out) {
    lock();
    bool ok = map_read_regs(&s_map, kind, addr, qty, out);
    unlock();
    return ok;
}

bool store_read_bits(uint8_t kind, uint16_t addr, uint16_t qty, uint8_t *out) {
    lock();
    bool ok = map_read_bits(&s_map, kind, addr, qty, out);
    unlock();
    return ok;
}

void store_get_layout(map_layout_t *out) {
    lock();
    out->strict = s_map.strict;
    memcpy(out->blocks, s_map.blocks, s_map.block_count * sizeof(map_block_t));
    out->block_count = s_map.block_count;
    memcpy(out->rules, s_map.rules, s_map.rule_count * sizeof(map_rule_t));
    out->rule_count = s_map.rule_count;
    out->regs_used = s_map.regs_used;
    out->bits_used = s_map.bits_used;
    unlock();
}

void store_peek(uint8_t kind, uint16_t start, uint32_t count, uint16_t *values, bool *defined) {
    lock();
    for (uint32_t i = 0; i < count; i++) {
        uint32_t addr = (uint32_t)start + i;
        bool bit = false;

        values[i] = 0;
        if (addr > 0xFFFF) {
            defined[i] = false;
        } else if (map_kind_is_bits(kind)) {
            defined[i] = map_peek_bit(&s_map, kind, addr, &bit);
            values[i] = bit;
        } else {
            defined[i] = map_peek_reg(&s_map, kind, addr, &values[i]);
        }
    }
    unlock();
}

static int finish(map_err_t err) {
    return err == MAP_OK ? save_map() : err;
}

int store_add_block(uint8_t kind, uint16_t start, uint32_t count, uint8_t flags) {
    lock();
    map_err_t err = map_add_block(&s_map, kind, start, count, flags);
    unlock();
    return finish(err);
}

int store_delete_block(uint8_t kind, uint16_t addr) {
    lock();
    map_err_t err = map_delete_block(&s_map, kind, addr);
    unlock();
    return finish(err);
}

int store_apply_rule(const map_rule_t *rule) {
    lock();
    map_err_t err = map_apply_rule(&s_map, rule, now_ms(), random32);
    unlock();
    return finish(err);
}

int store_clear_rules(uint8_t kind, uint16_t start, uint32_t count) {
    lock();
    map_err_t err = map_clear_rules(&s_map, kind, start, count);
    unlock();
    return finish(err);
}

int store_reset_defaults(void) {
    lock();
    map_load_defaults(&s_map);
    s_map.strict = s_strict;
    unlock();
    return save_map();
}

int store_set_strict(bool strict) {
    uint8_t value = strict;

    lock();
    s_strict = strict;
    s_map.strict = strict;
    unlock();

    if (settings_set_blob(NVS_KEY_STRICT, &value, sizeof(value)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save the address mode");
        return MAP_ERR_STORAGE;
    }
    ESP_LOGI(TAG, "%s mode", strict ? "Strict" : "Open");
    return MAP_OK;
}
