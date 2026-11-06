// The emulated device's memory: blocks of addresses in each Modbus table, and rules that
// drive their values. No ESP-IDF dependencies, so it can be tested on a host.
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define MAP_REG_POOL   16384   // registers shared by the 3x and 4x tables
#define MAP_BIT_POOL   65536   // bits shared by the 0x and 1x tables
#define MAP_MAX_BLOCKS 320   // enough to cover the register pool in automatic pages
#define MAP_MAX_RULES  256
#define MAP_WORDS_MAX  4
#define MAP_PAGE       64    // addresses in an automatic block, before trimming to fit

#define MAP_PERIOD_MIN_MS 100

// Values match the Modicon reference prefix (0x, 1x, 3x, 4x)
typedef enum {
    MAP_COILS = 0,
    MAP_DISCRETE = 1,
    MAP_INPUT = 3,
    MAP_HOLDING = 4,
} map_kind_t;

typedef enum {
    EMU_STATIC = 0,  // value, plus value_step for each value after the first. Applied once.
    EMU_RANDOM = 1,  // new value between min and max every period_ms
    EMU_SINE = 2,    // between min and max over period_ms, registers only
    EMU_SQUARE = 3,  // max for duty_pct of period_ms, min for the rest
    EMU_MODE_COUNT,
} emu_mode_t;

typedef enum {
    MAP_TYPE_UINT16 = 0,
    MAP_TYPE_INT16,
    MAP_TYPE_UINT32,
    MAP_TYPE_INT32,
    MAP_TYPE_FLOAT32,
    MAP_TYPE_FLOAT64,
    MAP_TYPE_BOOL,  // the only type for 0x and 1x
    MAP_TYPE_COUNT,
} map_type_t;

// How a value's bytes sit on the wire, named for a 32-bit value. Bit 0 reverses the
// word order and bit 1 swaps the bytes inside each word.
typedef enum {
    MAP_ORDER_ABCD = 0,  // big-endian, the Modbus default
    MAP_ORDER_CDAB = 1,  // word swap
    MAP_ORDER_BADC = 2,  // byte swap
    MAP_ORDER_DCBA = 3,  // little-endian
} map_order_t;

typedef enum {
    MAP_OK = 0,
    MAP_ERR_INVALID,    // bad settings, or a range that wraps past 65535
    MAP_ERR_OVERLAP,    // a block already covers part of the range
    MAP_ERR_UNMAPPED,   // part of the range isn't inside any block
    MAP_ERR_FULL,       // out of blocks, rules or pool space
    MAP_ERR_NOT_FOUND,
} map_err_t;

#define MAP_BLOCK_READ_ONLY 0x01  // master writes get exception 02
#define MAP_BLOCK_AUTO      0x02  // made by the map to store a rule in open mode

// A range of addresses with storage. In strict mode, reads anywhere else get exception 02.
// Stored in NVS as-is: bump MAP_STORE_VERSION in map_store.c if this layout changes.
typedef struct {
    uint32_t count;      // addresses, 1 to 65536
    uint32_t offset;     // into the register or bit pool
    uint16_t start;
    uint8_t kind;        // map_kind_t
    uint8_t flags;       // MAP_BLOCK_ values
} map_block_t;

// Drives count values from start upward. A wider type takes several registers per value.
// Stored in NVS as-is: bump MAP_STORE_VERSION in map_store.c if this layout changes.
typedef struct {
    uint32_t count;           // values, not registers
    uint32_t period_ms;
    uint16_t start;
    uint8_t kind;
    uint8_t type;             // map_type_t
    uint8_t order;            // map_order_t
    uint8_t mode;             // emu_mode_t
    uint8_t duty_pct;         // EMU_SQUARE
    uint8_t reserved;
    int16_t phase_deg;        // EMU_SINE, EMU_SQUARE
    int16_t phase_step_deg;   // added for each value after the first
    double value;             // EMU_STATIC
    double value_step;        // EMU_STATIC
    double min;
    double max;
} map_rule_t;

// Both lists are kept sorted by (kind, start). Blocks never overlap and nor do rules.
//
// In open mode, the default, every address answers: those without storage read 0, and
// rules get storage automatically. In strict mode only blocks exist, as on a real device.
typedef struct {
    bool strict;
    map_block_t blocks[MAP_MAX_BLOCKS];
    size_t block_count;
    map_rule_t rules[MAP_MAX_RULES];
    size_t rule_count;
    uint32_t next_random_ms[MAP_MAX_RULES];
    uint16_t regs[MAP_REG_POOL];
    size_t regs_used;
    uint8_t bits[MAP_BIT_POOL / 8];
    size_t bits_used;
} emu_map_t;

bool map_kind_valid(uint8_t kind);
bool map_kind_is_bits(uint8_t kind);

// Registers taken by one value of this type. 1 for BOOL, which takes one bit.
int map_type_words(uint8_t type);

// Encodes value into words, in wire order. Integer types round and clamp to their range.
void map_encode(uint8_t type, uint8_t order, double value, uint16_t *words);

// Addresses a rule covers: count times the words per value
uint32_t map_rule_span(const map_rule_t *rule);

// Empties the map, leaving it in open mode
void map_clear(emu_map_t *map);

// 3x3001-3x3100 as UINT16, each holding its own register number, so a read shifted by one
// shows up immediately on the master.
void map_load_defaults(emu_map_t *map);

map_err_t map_add_block(emu_map_t *map, uint8_t kind, uint16_t start, uint32_t count,
                        uint8_t flags);

// Deletes the block containing addr, along with any rules inside it
map_err_t map_delete_block(emu_map_t *map, uint8_t kind, uint16_t addr);

map_err_t map_rule_check(const emu_map_t *map, const map_rule_t *rule);

// Adds a rule, trimming any it overlaps. A wider value it only partly overlaps loses its
// rule, keeping its current registers. In open mode, storage for the rule is added in
// automatic blocks. Applies in full or not at all.
map_err_t map_apply_rule(emu_map_t *map, const map_rule_t *rule, uint32_t now_ms,
                         uint32_t (*random32)(void));

// Removes rules from count addresses. The values stay as they are.
map_err_t map_clear_rules(emu_map_t *map, uint8_t kind, uint16_t start, uint32_t count);

// The rule covering addr, or -1
int map_find_rule(const emu_map_t *map, uint8_t kind, uint16_t addr);

// False if any address in the range is undefined, which only happens in strict mode
bool map_read_regs(const emu_map_t *map, uint8_t kind, uint16_t addr, uint32_t qty,
                   uint16_t *out);

// Packs bits the Modbus way: the first address in bit 0 of out[0]. False if any address is
// undefined, which only happens in strict mode.
bool map_read_bits(const emu_map_t *map, uint8_t kind, uint16_t addr, uint32_t qty,
                   uint8_t *out);

// Reads for the web page: false for addresses that don't exist, which only happens in
// strict mode
bool map_peek_reg(const emu_map_t *map, uint8_t kind, uint16_t addr, uint16_t *value);
bool map_peek_bit(const emu_map_t *map, uint8_t kind, uint16_t addr, bool *value);

// Recomputes the values of one rule. random32 is only called for random rules that are due.
void map_tick_rule(emu_map_t *map, size_t index, uint32_t now_ms, uint32_t (*random32)(void));

void map_tick(emu_map_t *map, uint32_t now_ms, uint32_t (*random32)(void));
