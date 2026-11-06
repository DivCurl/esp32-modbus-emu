// Thread-safe, persistent emulator map, plus the task that runs the emulation.
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "emu_map.h"

// A failed flash write, after the change itself succeeded in RAM
#define MAP_ERR_STORAGE 100

typedef struct {
    bool strict;
    map_block_t blocks[MAP_MAX_BLOCKS];
    size_t block_count;
    map_rule_t rules[MAP_MAX_RULES];
    size_t rule_count;
    size_t regs_used;
    size_t bits_used;
} map_layout_t;

// Loads the saved map, migrating an older one, or the defaults. Starts the emulation task.
void store_init(void);

bool store_read_regs(uint8_t kind, uint16_t addr, uint16_t qty, uint16_t *out);
bool store_read_bits(uint8_t kind, uint16_t addr, uint16_t qty, uint8_t *out);

void store_get_layout(map_layout_t *out);

// Fills values and defined for count addresses. Bits read as 0 or 1.
void store_peek(uint8_t kind, uint16_t start, uint32_t count, uint16_t *values, bool *defined);

// Each returns a map_err_t, or MAP_ERR_STORAGE, and saves the map on success
int store_add_block(uint8_t kind, uint16_t start, uint32_t count, uint8_t flags);
int store_delete_block(uint8_t kind, uint16_t addr);
int store_apply_rule(const map_rule_t *rule);
int store_clear_rules(uint8_t kind, uint16_t start, uint32_t count);
int store_reset_defaults(void);

// Open mode answers every address. Strict mode only answers blocks. Saved on its own, so
// resetting the map keeps it.
int store_set_strict(bool strict);
