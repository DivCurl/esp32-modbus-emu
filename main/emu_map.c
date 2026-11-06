#include "emu_map.h"

#include <float.h>
#include <math.h>
#include <string.h>

#define PI_F 3.14159265f

typedef struct {
    int words;
    double min;
    double max;
} type_info_t;

static const type_info_t TYPE_INFO[MAP_TYPE_COUNT] = {
    [MAP_TYPE_UINT16] = {1, 0, 65535},
    [MAP_TYPE_INT16] = {1, -32768, 32767},
    [MAP_TYPE_UINT32] = {2, 0, 4294967295.0},
    [MAP_TYPE_INT32] = {2, -2147483648.0, 2147483647.0},
    [MAP_TYPE_FLOAT32] = {2, -FLT_MAX, FLT_MAX},
    [MAP_TYPE_FLOAT64] = {4, -DBL_MAX, DBL_MAX},
    [MAP_TYPE_BOOL] = {1, 0, 1},
};

bool map_kind_valid(uint8_t kind) {
    return kind == MAP_COILS || kind == MAP_DISCRETE || kind == MAP_INPUT || kind == MAP_HOLDING;
}

bool map_kind_is_bits(uint8_t kind) {
    return kind == MAP_COILS || kind == MAP_DISCRETE;
}

int map_type_words(uint8_t type) {
    return type < MAP_TYPE_COUNT ? TYPE_INFO[type].words : 1;
}

uint32_t map_rule_span(const map_rule_t *rule) {
    return rule->count * (uint32_t)map_type_words(rule->type);
}

static uint32_t sort_key(uint8_t kind, uint32_t addr) {
    return ((uint32_t)kind << 17) | addr;
}

// The comparisons also reject NaN and infinity
static bool in_type_range(uint8_t type, double value) {
    return value >= TYPE_INFO[type].min && value <= TYPE_INFO[type].max;
}

static double round_to_type(uint8_t type, double value) {
    value = round(value);
    if (value < TYPE_INFO[type].min) {
        return TYPE_INFO[type].min;
    }
    return value > TYPE_INFO[type].max ? TYPE_INFO[type].max : value;
}

void map_encode(uint8_t type, uint8_t order, double value, uint16_t *words) {
    int count = map_type_words(type);
    uint64_t bits = 0;

    switch (type) {
        case MAP_TYPE_UINT16:
        case MAP_TYPE_UINT32:
            bits = (uint32_t)round_to_type(type, value);
            break;
        case MAP_TYPE_INT16:
        case MAP_TYPE_INT32:
            bits = (uint32_t)(int32_t)round_to_type(type, value);
            break;
        case MAP_TYPE_FLOAT32: {
            float number = (float)value;
            uint32_t raw;
            memcpy(&raw, &number, sizeof(raw));
            bits = raw;
            break;
        }
        case MAP_TYPE_FLOAT64:
            memcpy(&bits, &value, sizeof(bits));
            break;
        case MAP_TYPE_BOOL:
            words[0] = value >= 0.5;
            return;
        default:
            break;
    }

    for (int i = 0; i < count; i++) {
        uint16_t word = (uint16_t)(bits >> (16 * (count - 1 - i)));

        if (order & 2) {
            word = (uint16_t)((word << 8) | (word >> 8));
        }
        words[(order & 1) ? count - 1 - i : i] = word;
    }
}

// ---- Blocks

// Bit blocks take whole bytes of the pool, so deleting one only needs a byte move
static uint32_t pool_size(const map_block_t *block) {
    return map_kind_is_bits(block->kind) ? (block->count + 7) / 8 * 8 : block->count;
}

static int find_block(const emu_map_t *map, uint8_t kind, uint32_t addr) {
    uint32_t key = sort_key(kind, addr);
    size_t low = 0;
    size_t high = map->block_count;

    // Last block starting at or before addr
    while (low < high) {
        size_t mid = low + (high - low) / 2;
        if (sort_key(map->blocks[mid].kind, map->blocks[mid].start) <= key) {
            low = mid + 1;
        } else {
            high = mid;
        }
    }

    if (low == 0) {
        return -1;
    }
    const map_block_t *block = &map->blocks[low - 1];
    if (block->kind != kind || addr >= (uint32_t)block->start + block->count) {
        return -1;
    }
    return (int)(low - 1);
}

static uint16_t *reg_at(const emu_map_t *map, uint8_t kind, uint32_t addr) {
    int i = find_block(map, kind, addr);
    if (i < 0) {
        return NULL;
    }
    const map_block_t *block = &map->blocks[i];
    return (uint16_t *)&map->regs[block->offset + addr - block->start];
}

static int bit_index(const emu_map_t *map, uint8_t kind, uint32_t addr) {
    int i = find_block(map, kind, addr);
    if (i < 0) {
        return -1;
    }
    const map_block_t *block = &map->blocks[i];
    return (int)(block->offset + addr - block->start);
}

static void set_bit(emu_map_t *map, uint8_t kind, uint32_t addr, bool value) {
    int index = bit_index(map, kind, addr);
    if (index >= 0) {
        if (value) {
            map->bits[index >> 3] |= 1 << (index & 7);
        } else {
            map->bits[index >> 3] &= ~(1 << (index & 7));
        }
    }
}

static bool get_bit(const emu_map_t *map, int index) {
    return (map->bits[index >> 3] >> (index & 7)) & 1;
}

void map_clear(emu_map_t *map) {
    memset(map, 0, sizeof(*map));
}

map_err_t map_add_block(emu_map_t *map, uint8_t kind, uint16_t start, uint32_t count,
                        uint8_t flags) {
    if (!map_kind_valid(kind) || count == 0 || (uint32_t)start + count > 0x10000) {
        return MAP_ERR_INVALID;
    }

    for (size_t i = 0; i < map->block_count; i++) {
        const map_block_t *other = &map->blocks[i];
        if (other->kind == kind && start < other->start + other->count &&
            other->start < start + count) {
            return MAP_ERR_OVERLAP;
        }
    }

    map_block_t block = {.count = count, .start = start, .kind = kind, .flags = flags};
    bool bits = map_kind_is_bits(kind);
    size_t used = bits ? map->bits_used : map->regs_used;
    size_t limit = bits ? MAP_BIT_POOL : MAP_REG_POOL;

    if (map->block_count == MAP_MAX_BLOCKS || used + pool_size(&block) > limit) {
        return MAP_ERR_FULL;
    }

    block.offset = used;
    if (bits) {
        map->bits_used += pool_size(&block);
    } else {
        map->regs_used += pool_size(&block);
    }

    size_t i = 0;
    while (i < map->block_count &&
           sort_key(map->blocks[i].kind, map->blocks[i].start) < sort_key(kind, start)) {
        i++;
    }
    memmove(&map->blocks[i + 1], &map->blocks[i],
            (map->block_count - i) * sizeof(map->blocks[0]));
    map->blocks[i] = block;
    map->block_count++;
    return MAP_OK;
}

map_err_t map_delete_block(emu_map_t *map, uint8_t kind, uint16_t addr) {
    int i = find_block(map, kind, addr);
    if (i < 0) {
        return MAP_ERR_NOT_FOUND;
    }

    map_block_t block = map->blocks[i];
    map_err_t err = map_clear_rules(map, kind, block.start, block.count);
    if (err != MAP_OK) {
        return err;
    }

    // Close the gap in the pool, so the space can be reused
    uint32_t size = pool_size(&block);
    if (map_kind_is_bits(kind)) {
        size_t from = (block.offset + size) / 8;
        memmove(&map->bits[block.offset / 8], &map->bits[from], map->bits_used / 8 - from);
        map->bits_used -= size;
        memset(&map->bits[map->bits_used / 8], 0, size / 8);
    } else {
        memmove(&map->regs[block.offset], &map->regs[block.offset + size],
                (map->regs_used - block.offset - size) * sizeof(map->regs[0]));
        map->regs_used -= size;
        memset(&map->regs[map->regs_used], 0, size * sizeof(map->regs[0]));
    }

    for (size_t j = 0; j < map->block_count; j++) {
        if (map_kind_is_bits(map->blocks[j].kind) == map_kind_is_bits(kind) &&
            map->blocks[j].offset > block.offset) {
            map->blocks[j].offset -= size;
        }
    }

    memmove(&map->blocks[i], &map->blocks[i + 1],
            (map->block_count - i - 1) * sizeof(map->blocks[0]));
    map->block_count--;
    return MAP_OK;
}

// ---- Rules

map_err_t map_rule_check(const emu_map_t *map, const map_rule_t *rule) {
    if (!map_kind_valid(rule->kind) || rule->type >= MAP_TYPE_COUNT ||
        rule->order > MAP_ORDER_DCBA || rule->mode >= EMU_MODE_COUNT || rule->count == 0 ||
        map_kind_is_bits(rule->kind) != (rule->type == MAP_TYPE_BOOL)) {
        return MAP_ERR_INVALID;
    }

    uint32_t span = map_rule_span(rule);
    if (rule->count > 0x10000 || (uint32_t)rule->start + span > 0x10000) {
        return MAP_ERR_INVALID;
    }

    switch (rule->mode) {
        case EMU_STATIC:
            if (!in_type_range(rule->type, rule->value) ||
                !in_type_range(rule->type, rule->value + (rule->count - 1) * rule->value_step)) {
                return MAP_ERR_INVALID;
            }
            break;
        case EMU_SINE:
            if (rule->type == MAP_TYPE_BOOL) {
                return MAP_ERR_INVALID;
            }
            // fall through
        case EMU_RANDOM:
        case EMU_SQUARE:
            if (!in_type_range(rule->type, rule->min) || !in_type_range(rule->type, rule->max) ||
                rule->min > rule->max || rule->period_ms < MAP_PERIOD_MIN_MS ||
                rule->duty_pct > 100) {
                return MAP_ERR_INVALID;
            }
            break;
        default:
            return MAP_ERR_INVALID;
    }

    // In strict mode every address must exist, though the range may run across adjacent
    // blocks. Open mode adds whatever is missing.
    if (!map->strict) {
        return MAP_OK;
    }
    uint32_t addr = rule->start;
    while (addr < rule->start + span) {
        int i = find_block(map, rule->kind, addr);
        if (i < 0) {
            return MAP_ERR_UNMAPPED;
        }
        addr = (uint32_t)map->blocks[i].start + map->blocks[i].count;
    }
    return MAP_OK;
}

static uint32_t rule_end(const map_rule_t *rule) {
    return rule->start + map_rule_span(rule);
}

int map_find_rule(const emu_map_t *map, uint8_t kind, uint16_t addr) {
    for (size_t i = 0; i < map->rule_count; i++) {
        const map_rule_t *rule = &map->rules[i];
        if (rule->kind == kind && addr >= rule->start && addr < rule_end(rule)) {
            return (int)i;
        }
    }
    return -1;
}

// The part of rule from value index first onward
static map_rule_t rule_tail(const map_rule_t *rule, uint32_t first) {
    map_rule_t tail = *rule;
    tail.start = rule->start + first * map_type_words(rule->type);
    tail.count = rule->count - first;
    tail.value = rule->value + first * rule->value_step;
    tail.phase_deg = (int16_t)((rule->phase_deg + (int32_t)(first % 360) * rule->phase_step_deg) % 360);
    return tail;
}

static void insert_rule(emu_map_t *map, const map_rule_t *rule) {
    size_t i = 0;
    while (i < map->rule_count &&
           sort_key(map->rules[i].kind, map->rules[i].start) < sort_key(rule->kind, rule->start)) {
        i++;
    }
    memmove(&map->rules[i + 1], &map->rules[i], (map->rule_count - i) * sizeof(map->rules[0]));
    memmove(&map->next_random_ms[i + 1], &map->next_random_ms[i],
            (map->rule_count - i) * sizeof(map->next_random_ms[0]));
    map->rules[i] = *rule;
    map->next_random_ms[i] = 0;
    map->rule_count++;
}

// Removes rules from [start, end), keeping the whole values of any partly covered. Fails
// without changing anything if the result, plus extra more, wouldn't fit.
static map_err_t trim_rules(emu_map_t *map, uint8_t kind, uint32_t start, uint32_t end,
                            size_t extra) {
    size_t first = 0;
    while (first < map->rule_count &&
           (map->rules[first].kind < kind ||
            (map->rules[first].kind == kind && rule_end(&map->rules[first]) <= start))) {
        first++;
    }

    size_t last = first;
    while (last < map->rule_count && map->rules[last].kind == kind &&
           map->rules[last].start < end) {
        last++;
    }

    // Only the first and last overlapped rules can stick out of the range
    map_rule_t pieces[2];
    size_t piece_count = 0;

    if (last > first) {
        const map_rule_t *head = &map->rules[first];
        const map_rule_t *tail = &map->rules[last - 1];
        uint32_t words = map_type_words(head->type);

        if (head->start < start) {
            uint32_t keep = (start - head->start) / words;
            if (keep > 0) {
                pieces[piece_count] = *head;
                pieces[piece_count].count = keep;
                piece_count++;
            }
        }

        words = map_type_words(tail->type);
        if (rule_end(tail) > end) {
            uint32_t from = (end - tail->start + words - 1) / words;
            if (from < tail->count) {
                pieces[piece_count++] = rule_tail(tail, from);
            }
        }
    }

    if (map->rule_count - (last - first) + piece_count + extra > MAP_MAX_RULES) {
        return MAP_ERR_FULL;
    }

    memmove(&map->rules[first], &map->rules[last], (map->rule_count - last) * sizeof(map->rules[0]));
    memmove(&map->next_random_ms[first], &map->next_random_ms[last],
            (map->rule_count - last) * sizeof(map->next_random_ms[0]));
    map->rule_count -= last - first;

    for (size_t i = 0; i < piece_count; i++) {
        insert_rule(map, &pieces[i]);
    }
    return MAP_OK;
}

// The end of the block before addr and the start of the one after, in addr's table
static void gap_around(const emu_map_t *map, uint8_t kind, uint32_t addr, uint32_t *low,
                       uint32_t *high) {
    *low = 0;
    *high = 0x10000;
    for (size_t i = 0; i < map->block_count; i++) {
        const map_block_t *block = &map->blocks[i];
        uint32_t end = (uint32_t)block->start + block->count;
        if (block->kind != kind) {
            continue;
        }
        if (end <= addr && end > *low) {
            *low = end;
        }
        if (block->start > addr && block->start < *high) {
            *high = block->start;
        }
    }
}

// Gives every address in [start, end) storage, in page-aligned automatic blocks fitted
// around existing ones. With apply false, only checks there is room.
static map_err_t ensure_storage(emu_map_t *map, uint8_t kind, uint32_t start, uint32_t end,
                                bool apply) {
    size_t blocks = 0;
    size_t space = 0;
    uint32_t addr = start;

    while (addr < end) {
        int found = find_block(map, kind, addr);
        if (found >= 0) {
            addr = (uint32_t)map->blocks[found].start + map->blocks[found].count;
            continue;
        }

        uint32_t low;
        uint32_t high;
        uint32_t page = addr & ~(uint32_t)(MAP_PAGE - 1);
        gap_around(map, kind, addr, &low, &high);
        low = low > page ? low : page;
        high = high < page + MAP_PAGE ? high : page + MAP_PAGE;

        if (apply) {
            map_err_t err = map_add_block(map, kind, low, high - low, MAP_BLOCK_AUTO);
            if (err != MAP_OK) {
                return err;
            }
        } else {
            map_block_t block = {.count = high - low, .kind = kind};
            blocks++;
            space += pool_size(&block);
        }
        addr = high;
    }

    if (!apply) {
        bool bits = map_kind_is_bits(kind);
        size_t used = bits ? map->bits_used : map->regs_used;
        size_t limit = bits ? MAP_BIT_POOL : MAP_REG_POOL;
        if (map->block_count + blocks > MAP_MAX_BLOCKS || used + space > limit) {
            return MAP_ERR_FULL;
        }
    }
    return MAP_OK;
}

static void store_value(emu_map_t *map, const map_rule_t *rule, uint32_t addr, double value) {
    uint16_t words[MAP_WORDS_MAX];

    map_encode(rule->type, rule->order, value, words);
    if (map_kind_is_bits(rule->kind)) {
        set_bit(map, rule->kind, addr, words[0]);
        return;
    }
    for (int w = 0; w < map_type_words(rule->type); w++) {
        uint16_t *reg = reg_at(map, rule->kind, addr + w);
        if (reg) {
            *reg = words[w];
        }
    }
}

map_err_t map_apply_rule(emu_map_t *map, const map_rule_t *rule, uint32_t now_ms,
                         uint32_t (*random32)(void)) {
    map_err_t err = map_rule_check(map, rule);
    if (err != MAP_OK) {
        return err;
    }

    // Both checked before anything changes, so a failure leaves the map as it was
    if (!map->strict) {
        err = ensure_storage(map, rule->kind, rule->start, rule_end(rule), false);
        if (err != MAP_OK) {
            return err;
        }
    }
    err = trim_rules(map, rule->kind, rule->start, rule_end(rule), 1);
    if (err != MAP_OK) {
        return err;
    }
    if (!map->strict) {
        ensure_storage(map, rule->kind, rule->start, rule_end(rule), true);
    }
    insert_rule(map, rule);

    // A static rule sets its values once, so a master's writes can stick afterwards
    if (rule->mode == EMU_STATIC) {
        uint32_t words = map_type_words(rule->type);
        for (uint32_t v = 0; v < rule->count; v++) {
            store_value(map, rule, rule->start + v * words, rule->value + v * rule->value_step);
        }
    } else {
        map_tick_rule(map, map_find_rule(map, rule->kind, rule->start), now_ms, random32);
    }
    return MAP_OK;
}

map_err_t map_clear_rules(emu_map_t *map, uint8_t kind, uint16_t start, uint32_t count) {
    if (!map_kind_valid(kind) || count == 0 || (uint32_t)start + count > 0x10000) {
        return MAP_ERR_INVALID;
    }
    return trim_rules(map, kind, start, (uint32_t)start + count, 0);
}

void map_load_defaults(emu_map_t *map) {
    map_rule_t rule = {
        .count = 100,
        .start = 3000,
        .kind = MAP_INPUT,
        .type = MAP_TYPE_UINT16,
        .mode = EMU_STATIC,
        .value = 3001,
        .value_step = 1,
    };

    map_clear(map);
    map_add_block(map, MAP_INPUT, 3000, 100, 0);
    map_apply_rule(map, &rule, 0, NULL);
}

// ---- Reads

bool map_read_regs(const emu_map_t *map, uint8_t kind, uint16_t addr, uint32_t qty,
                   uint16_t *out) {
    for (uint32_t n = 0; n < qty; n++) {
        const uint16_t *reg = reg_at(map, kind, (uint32_t)addr + n);
        if (reg == NULL && map->strict) {
            return false;
        }
        out[n] = reg ? *reg : 0;
    }
    return true;
}

bool map_read_bits(const emu_map_t *map, uint8_t kind, uint16_t addr, uint32_t qty,
                   uint8_t *out) {
    memset(out, 0, (qty + 7) / 8);
    for (uint32_t n = 0; n < qty; n++) {
        int index = bit_index(map, kind, (uint32_t)addr + n);
        if (index < 0) {
            if (map->strict) {
                return false;
            }
            continue;
        }
        if (get_bit(map, index)) {
            out[n >> 3] |= 1 << (n & 7);
        }
    }
    return true;
}

bool map_peek_reg(const emu_map_t *map, uint8_t kind, uint16_t addr, uint16_t *value) {
    const uint16_t *reg = reg_at(map, kind, addr);
    *value = reg ? *reg : 0;
    return reg != NULL || !map->strict;
}

bool map_peek_bit(const emu_map_t *map, uint8_t kind, uint16_t addr, bool *value) {
    int index = bit_index(map, kind, addr);
    *value = index >= 0 && get_bit(map, index);
    return index >= 0 || !map->strict;
}

// ---- Emulation

static int value_phase(const map_rule_t *rule, uint32_t index) {
    int phase = (rule->phase_deg + (int32_t)(index % 360) * rule->phase_step_deg) % 360;
    return phase < 0 ? phase + 360 : phase;
}

static double sine_value(const map_rule_t *rule, uint32_t now_ms, int phase) {
    // Halved separately so the widest float ranges can't overflow
    double mid = rule->min / 2.0 + rule->max / 2.0;
    double amplitude = rule->max / 2.0 - rule->min / 2.0;
    float cycles = (float)(now_ms % rule->period_ms) / (float)rule->period_ms;

    cycles += (float)phase / 360.0f;

    double value = mid + amplitude * (double)sinf(2.0f * PI_F * cycles);
    if (value < rule->min) {
        return rule->min;
    }
    return value > rule->max ? rule->max : value;
}

static double square_value(const map_rule_t *rule, uint32_t now_ms, int phase) {
    uint64_t shifted = (uint64_t)now_ms + (uint64_t)rule->period_ms * phase / 360;
    uint32_t position = (uint32_t)(shifted % rule->period_ms);

    return (uint64_t)position * 100 < (uint64_t)rule->period_ms * rule->duty_pct ? rule->max
                                                                                 : rule->min;
}

static double random_value(const map_rule_t *rule, uint32_t random) {
    double fraction = (double)random / 4294967295.0;

    // Blended rather than min + fraction * span, which overflows on the widest float ranges
    return rule->min * (1.0 - fraction) + rule->max * fraction;
}

void map_tick_rule(emu_map_t *map, size_t index, uint32_t now_ms, uint32_t (*random32)(void)) {
    if (index >= map->rule_count) {
        return;
    }

    const map_rule_t *rule = &map->rules[index];
    uint32_t words = map_type_words(rule->type);

    if (rule->mode == EMU_STATIC) {
        return;
    }

    // Zero means new, so due now. The signed difference survives now_ms wrapping.
    if (rule->mode == EMU_RANDOM) {
        if (map->next_random_ms[index] != 0 &&
            (int32_t)(now_ms - map->next_random_ms[index]) < 0) {
            return;
        }
        map->next_random_ms[index] = now_ms + rule->period_ms;
    }

    for (uint32_t v = 0; v < rule->count; v++) {
        uint32_t addr = rule->start + v * words;
        double value;

        switch (rule->mode) {
            case EMU_SINE:
                value = sine_value(rule, now_ms, value_phase(rule, v));
                break;
            case EMU_SQUARE:
                value = square_value(rule, now_ms, value_phase(rule, v));
                break;
            case EMU_RANDOM:
                value = random_value(rule, random32());
                break;
            default:
                return;
        }
        store_value(map, rule, addr, value);
    }
}

void map_tick(emu_map_t *map, uint32_t now_ms, uint32_t (*random32)(void)) {
    for (size_t i = 0; i < map->rule_count; i++) {
        map_tick_rule(map, i, now_ms, random32);
    }
}
