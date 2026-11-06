// Tests for the logic that has no ESP-IDF dependencies. From the project root:
//   gcc -std=gnu11 -Imain test/host_test.c main/mb_frame.c main/emu_map.c main/wifi_led.c \
//       -lm -o host_test
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "emu_map.h"
#include "mb_frame.h"
#include "wifi_led.h"

static int s_failures;
static int s_checks;

#define CHECK(condition)                                                   \
    do {                                                                   \
        s_checks++;                                                        \
        if (!(condition)) {                                                \
            s_failures++;                                                  \
            printf("FAILED %s:%d: %s\n", __FILE__, __LINE__, #condition);  \
        }                                                                  \
    } while (0)

static emu_map_t s_map;

static uint32_t s_random_calls;
static uint32_t s_random_value;

static uint32_t fake_random(void) {
    s_random_calls++;
    return s_random_value;
}

// ---- Modbus frames against a map

static bool read_regs(void *ctx, uint8_t fc, uint16_t addr, uint16_t qty, uint16_t *out) {
    return map_read_regs(&s_map, fc == MB_FC_READ_INPUT ? MAP_INPUT : MAP_HOLDING, addr, qty, out);
}

static bool read_bits(void *ctx, uint8_t fc, uint16_t addr, uint16_t qty, uint8_t *out) {
    return map_read_bits(&s_map, fc == MB_FC_READ_COILS ? MAP_COILS : MAP_DISCRETE, addr, qty,
                         out);
}

static const mb_handlers_t HANDLERS = {.read_regs = read_regs, .read_bits = read_bits};

static size_t build_read(uint8_t *frame, uint8_t id, uint8_t fc, uint16_t addr, uint16_t qty) {
    frame[0] = id;
    frame[1] = fc;
    frame[2] = addr >> 8;
    frame[3] = addr & 0xFF;
    frame[4] = qty >> 8;
    frame[5] = qty & 0xFF;
    uint16_t crc = mb_crc16(frame, 6);
    frame[6] = crc & 0xFF;
    frame[7] = crc >> 8;
    return 8;
}

static mb_result_t send_frame(const uint8_t *req, size_t len, uint8_t *resp, size_t *resp_len,
                              mb_request_t *info) {
    return mb_handle_frame(req, len, 1, &HANDLERS, resp, resp_len, info);
}

static bool resp_crc_ok(const uint8_t *resp, size_t len) {
    uint16_t crc = mb_crc16(resp, len - 2);
    return resp[len - 2] == (crc & 0xFF) && resp[len - 1] == (crc >> 8);
}

static uint8_t exception_for(uint8_t fc, uint16_t addr, uint16_t qty) {
    uint8_t req[8];
    uint8_t resp[MB_FRAME_MAX];
    size_t resp_len = 0;
    mb_request_t info;

    build_read(req, 1, fc, addr, qty);
    if (send_frame(req, 8, resp, &resp_len, &info) != MB_RESULT_REPLY) {
        return 0xFF;
    }
    return info.exception;
}

static void test_crc(void) {
    // Published reference frame: 01 04 00 00 00 01 -> CRC 31 CA on the wire
    const uint8_t frame[] = {0x01, 0x04, 0x00, 0x00, 0x00, 0x01};
    CHECK(mb_crc16(frame, sizeof(frame)) == 0xCA31);

    // CRC-16/MODBUS check value
    CHECK(mb_crc16((const uint8_t *)"123456789", 9) == 0x4B37);
}

static void test_read_defaults(void) {
    uint8_t req[8];
    uint8_t resp[MB_FRAME_MAX];
    size_t resp_len = 0;
    mb_request_t info;

    map_load_defaults(&s_map);
    s_map.strict = true;
    build_read(req, 1, MB_FC_READ_INPUT, 3000, 100);

    CHECK(send_frame(req, 8, resp, &resp_len, &info) == MB_RESULT_REPLY);
    CHECK(info.exception == 0 && info.addr == 3000 && info.qty == 100);
    CHECK(resp_len == 205);
    CHECK(resp[0] == 1 && resp[1] == 4 && resp[2] == 200);
    CHECK(((resp[3] << 8) | resp[4]) == 3001);      // 3x3001
    CHECK(((resp[201] << 8) | resp[202]) == 3100);  // 3x3100
    CHECK(resp_crc_ok(resp, resp_len));

    // One past the end of the map, the classic off-by-one from a master
    CHECK(exception_for(MB_FC_READ_INPUT, 3001, 100) == MB_EX_ILLEGAL_ADDRESS);
    // The defaults only have input registers
    CHECK(exception_for(MB_FC_READ_HOLDING, 3000, 1) == MB_EX_ILLEGAL_ADDRESS);
    CHECK(exception_for(MB_FC_READ_COILS, 0, 1) == MB_EX_ILLEGAL_ADDRESS);
    CHECK(exception_for(MB_FC_READ_DISCRETE, 0, 1) == MB_EX_ILLEGAL_ADDRESS);
}

static void test_exceptions(void) {
    map_load_defaults(&s_map);

    CHECK(exception_for(MB_FC_READ_INPUT, 3000, 0) == MB_EX_ILLEGAL_VALUE);
    CHECK(exception_for(MB_FC_READ_INPUT, 3000, 126) == MB_EX_ILLEGAL_VALUE);
    CHECK(exception_for(MB_FC_READ_INPUT, 0xFFFF, 2) == MB_EX_ILLEGAL_ADDRESS);
    CHECK(exception_for(MB_FC_READ_COILS, 0, 0) == MB_EX_ILLEGAL_VALUE);
    CHECK(exception_for(MB_FC_READ_COILS, 0, 2001) == MB_EX_ILLEGAL_VALUE);

    uint8_t req[8];
    uint8_t resp[MB_FRAME_MAX];
    size_t resp_len = 0;
    mb_request_t info;
    build_read(req, 1, 0x10, 3000, 1);
    send_frame(req, 8, resp, &resp_len, &info);
    CHECK(info.exception == MB_EX_ILLEGAL_FUNCTION && resp[1] == 0x90);
    CHECK(resp_len == 5 && resp_crc_ok(resp, resp_len));
}

static void test_silent_frames(void) {
    uint8_t req[8];
    uint8_t resp[MB_FRAME_MAX];
    size_t resp_len = 0;
    mb_request_t info;

    map_load_defaults(&s_map);

    build_read(req, 2, MB_FC_READ_INPUT, 3000, 1);
    CHECK(send_frame(req, 8, resp, &resp_len, &info) == MB_RESULT_OTHER_SLAVE);

    build_read(req, 0, MB_FC_READ_INPUT, 3000, 1);
    CHECK(send_frame(req, 8, resp, &resp_len, &info) == MB_RESULT_OTHER_SLAVE);

    build_read(req, 1, MB_FC_READ_INPUT, 3000, 1);
    req[3] ^= 0x01;
    CHECK(send_frame(req, 8, resp, &resp_len, &info) == MB_RESULT_BAD_FRAME);

    CHECK(send_frame(req, 3, resp, &resp_len, &info) == MB_RESULT_BAD_FRAME);
    CHECK(send_frame(req, MB_FRAME_MAX + 1, resp, &resp_len, &info) == MB_RESULT_BAD_FRAME);
}

static void test_read_bits(void) {
    uint8_t req[8];
    uint8_t resp[MB_FRAME_MAX];
    size_t resp_len = 0;
    mb_request_t info;

    map_clear(&s_map);
    s_map.strict = true;
    CHECK(map_add_block(&s_map, MAP_COILS, 0, 20, false) == MAP_OK);
    CHECK(map_add_block(&s_map, MAP_DISCRETE, 0, 2000, false) == MAP_OK);

    // Coils 0, 2 and 9 on: 0x05 then 0x02
    map_rule_t on = {.count = 1, .kind = MAP_COILS, .type = MAP_TYPE_BOOL, .value = 1};
    on.start = 0;
    CHECK(map_apply_rule(&s_map, &on, 0, NULL) == MAP_OK);
    on.start = 2;
    CHECK(map_apply_rule(&s_map, &on, 0, NULL) == MAP_OK);
    on.start = 9;
    CHECK(map_apply_rule(&s_map, &on, 0, NULL) == MAP_OK);

    build_read(req, 1, MB_FC_READ_COILS, 0, 10);
    CHECK(send_frame(req, 8, resp, &resp_len, &info) == MB_RESULT_REPLY);
    CHECK(info.exception == 0);
    CHECK(resp_len == 7 && resp[1] == 1 && resp[2] == 2);
    CHECK(resp[3] == 0x05 && resp[4] == 0x02);
    CHECK(resp_crc_ok(resp, resp_len));

    // Bits past the quantity stay zero, even when the memory beyond is on
    build_read(req, 1, MB_FC_READ_COILS, 2, 1);
    send_frame(req, 8, resp, &resp_len, &info);
    CHECK(resp[2] == 1 && resp[3] == 0x01);
    build_read(req, 1, MB_FC_READ_COILS, 1, 1);
    send_frame(req, 8, resp, &resp_len, &info);
    CHECK(resp[3] == 0x00);

    // The largest read fills the frame exactly
    map_rule_t all = {.count = 2000, .kind = MAP_DISCRETE, .type = MAP_TYPE_BOOL, .value = 1};
    CHECK(map_apply_rule(&s_map, &all, 0, NULL) == MAP_OK);
    build_read(req, 1, MB_FC_READ_DISCRETE, 0, 2000);
    CHECK(send_frame(req, 8, resp, &resp_len, &info) == MB_RESULT_REPLY);
    CHECK(info.exception == 0 && resp[2] == 250 && resp_len == 255);
    CHECK(resp[3] == 0xFF && resp[252] == 0xFF);
    CHECK(resp_crc_ok(resp, resp_len));

    CHECK(exception_for(MB_FC_READ_COILS, 15, 6) == MB_EX_ILLEGAL_ADDRESS);
    CHECK(exception_for(MB_FC_READ_COILS, 15, 5) == 0);
}

// ---- Blocks

static void test_blocks(void) {
    uint16_t out[4];
    map_clear(&s_map);
    s_map.strict = true;

    CHECK(map_add_block(&s_map, MAP_HOLDING, 100, 10, false) == MAP_OK);
    CHECK(map_add_block(&s_map, MAP_HOLDING, 105, 10, false) == MAP_ERR_OVERLAP);
    CHECK(map_add_block(&s_map, MAP_HOLDING, 95, 6, false) == MAP_ERR_OVERLAP);
    CHECK(map_add_block(&s_map, MAP_HOLDING, 90, 10, false) == MAP_OK);   // touching is fine
    CHECK(map_add_block(&s_map, MAP_INPUT, 100, 10, false) == MAP_OK);    // other table
    CHECK(map_add_block(&s_map, MAP_HOLDING, 0xFFFF, 2, false) == MAP_ERR_INVALID);
    CHECK(map_add_block(&s_map, MAP_HOLDING, 0, 0, false) == MAP_ERR_INVALID);
    CHECK(map_add_block(&s_map, 2, 0, 1, false) == MAP_ERR_INVALID);
    CHECK(s_map.regs_used == 30);

    // Reads run across touching blocks, but not past the end of either
    CHECK(map_read_regs(&s_map, MAP_HOLDING, 98, 4, out));
    CHECK(!map_read_regs(&s_map, MAP_HOLDING, 108, 4, out));
    CHECK(!map_read_regs(&s_map, MAP_HOLDING, 89, 2, out));

    // Deleting a block keeps the others' values, and frees its space
    map_rule_t rule = {.count = 10, .start = 100, .kind = MAP_HOLDING, .value = 500,
                       .value_step = 1};
    CHECK(map_apply_rule(&s_map, &rule, 0, NULL) == MAP_OK);
    rule.kind = MAP_INPUT;
    rule.value = 700;
    CHECK(map_apply_rule(&s_map, &rule, 0, NULL) == MAP_OK);
    CHECK(map_delete_block(&s_map, MAP_HOLDING, 95) == MAP_OK);
    CHECK(map_delete_block(&s_map, MAP_HOLDING, 95) == MAP_ERR_NOT_FOUND);
    CHECK(s_map.regs_used == 20);
    CHECK(map_read_regs(&s_map, MAP_HOLDING, 100, 2, out) && out[0] == 500 && out[1] == 501);
    CHECK(map_read_regs(&s_map, MAP_INPUT, 109, 1, out) && out[0] == 709);

    // Deleting a block takes its rules with it
    CHECK(s_map.rule_count == 2);
    CHECK(map_delete_block(&s_map, MAP_HOLDING, 100) == MAP_OK);
    CHECK(s_map.rule_count == 1);
    CHECK(map_read_regs(&s_map, MAP_INPUT, 100, 1, out) && out[0] == 700);

    // The register pool is shared by 3x and 4x
    map_clear(&s_map);
    CHECK(map_add_block(&s_map, MAP_INPUT, 0, MAP_REG_POOL - 1, false) == MAP_OK);
    CHECK(map_add_block(&s_map, MAP_HOLDING, 0, 2, false) == MAP_ERR_FULL);
    CHECK(map_add_block(&s_map, MAP_HOLDING, 0, 1, false) == MAP_OK);

    // The bit pool covers one whole table, shared by 0x and 1x
    map_clear(&s_map);
    CHECK(map_add_block(&s_map, MAP_DISCRETE, 0, 0x10000, false) == MAP_OK);
    CHECK(map_add_block(&s_map, MAP_COILS, 0, 1, false) == MAP_ERR_FULL);
    CHECK(map_delete_block(&s_map, MAP_DISCRETE, 40000) == MAP_OK);
    CHECK(s_map.bits_used == 0);

    // Bit blocks pack by whole bytes, and survive a neighbour's deletion
    map_clear(&s_map);
    CHECK(map_add_block(&s_map, MAP_COILS, 0, 3, false) == MAP_OK);
    CHECK(map_add_block(&s_map, MAP_DISCRETE, 0, 10, false) == MAP_OK);
    CHECK(s_map.bits_used == 8 + 16);
    map_rule_t bits = {.count = 10, .kind = MAP_DISCRETE, .type = MAP_TYPE_BOOL, .value = 1};
    CHECK(map_apply_rule(&s_map, &bits, 0, NULL) == MAP_OK);
    CHECK(map_delete_block(&s_map, MAP_COILS, 1) == MAP_OK);
    uint8_t packed[2];
    CHECK(map_read_bits(&s_map, MAP_DISCRETE, 0, 10, packed));
    CHECK(packed[0] == 0xFF && packed[1] == 0x03);

    // Running out of blocks
    map_clear(&s_map);
    for (int i = 0; i < MAP_MAX_BLOCKS; i++) {
        CHECK(map_add_block(&s_map, MAP_HOLDING, i * 2, 1, false) == MAP_OK);
    }
    CHECK(map_add_block(&s_map, MAP_HOLDING, 1000, 1, false) == MAP_ERR_FULL);
}

// ---- Rules

static map_rule_t static_rule(uint16_t start, uint32_t count, double value, double step) {
    return (map_rule_t){.count = count, .start = start, .kind = MAP_HOLDING,
                        .type = MAP_TYPE_UINT16, .mode = EMU_STATIC, .value = value,
                        .value_step = step};
}

static void test_rule_checks(void) {
    map_clear(&s_map);
    s_map.strict = true;
    map_add_block(&s_map, MAP_HOLDING, 0, 100, false);
    map_add_block(&s_map, MAP_HOLDING, 100, 100, false);
    map_add_block(&s_map, MAP_COILS, 0, 16, false);

    map_rule_t rule = static_rule(0, 200, 0, 1);
    CHECK(map_rule_check(&s_map, &rule) == MAP_OK);   // across touching blocks
    rule.count = 201;
    CHECK(map_rule_check(&s_map, &rule) == MAP_ERR_UNMAPPED);

    // Static values must stay in the type's range for every value
    rule = static_rule(0, 10, 65530, 1);
    CHECK(map_rule_check(&s_map, &rule) == MAP_ERR_INVALID);
    rule = static_rule(0, 10, 65526, 1);
    CHECK(map_rule_check(&s_map, &rule) == MAP_OK);
    rule = static_rule(0, 1, -1, 0);
    CHECK(map_rule_check(&s_map, &rule) == MAP_ERR_INVALID);
    rule.type = MAP_TYPE_INT16;
    CHECK(map_rule_check(&s_map, &rule) == MAP_OK);

    // Types must suit the table
    rule.type = MAP_TYPE_BOOL;
    CHECK(map_rule_check(&s_map, &rule) == MAP_ERR_INVALID);
    rule = (map_rule_t){.count = 1, .kind = MAP_COILS, .type = MAP_TYPE_UINT16};
    CHECK(map_rule_check(&s_map, &rule) == MAP_ERR_INVALID);
    rule.type = MAP_TYPE_BOOL;
    CHECK(map_rule_check(&s_map, &rule) == MAP_OK);

    // Bits can't follow a sine wave
    rule.mode = EMU_SINE;
    rule.max = 1;
    rule.period_ms = 1000;
    CHECK(map_rule_check(&s_map, &rule) == MAP_ERR_INVALID);
    rule.mode = EMU_SQUARE;
    CHECK(map_rule_check(&s_map, &rule) == MAP_OK);

    // Live modes need a sensible range and period
    rule = (map_rule_t){.count = 1, .kind = MAP_HOLDING, .type = MAP_TYPE_UINT16,
                        .mode = EMU_SINE, .min = 10, .max = 5, .period_ms = 1000};
    CHECK(map_rule_check(&s_map, &rule) == MAP_ERR_INVALID);
    rule.max = 20;
    CHECK(map_rule_check(&s_map, &rule) == MAP_OK);
    rule.period_ms = MAP_PERIOD_MIN_MS - 1;
    CHECK(map_rule_check(&s_map, &rule) == MAP_ERR_INVALID);
    rule.period_ms = 1000;
    rule.mode = EMU_SQUARE;
    rule.duty_pct = 101;
    CHECK(map_rule_check(&s_map, &rule) == MAP_ERR_INVALID);

    rule = static_rule(0, 1, NAN, 0);
    CHECK(map_rule_check(&s_map, &rule) == MAP_ERR_INVALID);
    rule = static_rule(0, 1, 0, 0);
    rule.mode = EMU_MODE_COUNT;
    CHECK(map_rule_check(&s_map, &rule) == MAP_ERR_INVALID);
    rule = static_rule(0, 1, 0, 0);
    rule.order = 4;
    CHECK(map_rule_check(&s_map, &rule) == MAP_ERR_INVALID);
    rule = static_rule(0, 1, 0, 0);
    rule.type = MAP_TYPE_FLOAT64;
    rule.start = 197;
    CHECK(map_rule_check(&s_map, &rule) == MAP_ERR_UNMAPPED);   // last word past the block
    rule.start = 196;
    CHECK(map_rule_check(&s_map, &rule) == MAP_OK);
}

static void test_rule_trimming(void) {
    uint16_t out[10];

    map_clear(&s_map);
    map_add_block(&s_map, MAP_HOLDING, 0, 100, false);

    map_rule_t base = static_rule(0, 100, 1000, 1);
    CHECK(map_apply_rule(&s_map, &base, 0, NULL) == MAP_OK);

    // A rule in the middle splits the one under it, and the tail carries on its sequence
    map_rule_t middle = static_rule(40, 10, 7, 0);
    CHECK(map_apply_rule(&s_map, &middle, 0, NULL) == MAP_OK);
    CHECK(s_map.rule_count == 3);
    CHECK(s_map.rules[0].start == 0 && s_map.rules[0].count == 40);
    CHECK(s_map.rules[1].start == 40 && s_map.rules[1].count == 10);
    CHECK(s_map.rules[2].start == 50 && s_map.rules[2].count == 50);
    CHECK(s_map.rules[2].value == 1050);
    CHECK(map_read_regs(&s_map, MAP_HOLDING, 38, 4, out));
    CHECK(out[0] == 1038 && out[1] == 1039 && out[2] == 7 && out[3] == 7);
    CHECK(map_find_rule(&s_map, MAP_HOLDING, 45) == 1);

    // Covering several rules removes those wholly inside
    map_rule_t wide = static_rule(30, 30, 1, 0);
    CHECK(map_apply_rule(&s_map, &wide, 0, NULL) == MAP_OK);
    CHECK(s_map.rule_count == 3);
    CHECK(s_map.rules[0].count == 30 && s_map.rules[2].start == 60 && s_map.rules[2].count == 40);
    CHECK(s_map.rules[2].value == 1060);

    // Clearing keeps the values but drops the rules
    CHECK(map_clear_rules(&s_map, MAP_HOLDING, 0, 100) == MAP_OK);
    CHECK(s_map.rule_count == 0);
    CHECK(map_read_regs(&s_map, MAP_HOLDING, 59, 2, out) && out[0] == 1 && out[1] == 1060);

    // A wider value only partly covered loses its rule but keeps its registers
    map_rule_t real = {.count = 3, .start = 10, .kind = MAP_HOLDING, .type = MAP_TYPE_FLOAT32,
                       .value = 12.5};
    CHECK(map_apply_rule(&s_map, &real, 0, NULL) == MAP_OK);   // 10-11, 12-13, 14-15
    map_rule_t poke = static_rule(13, 1, 9, 0);
    CHECK(map_apply_rule(&s_map, &poke, 0, NULL) == MAP_OK);
    CHECK(s_map.rule_count == 3);
    CHECK(s_map.rules[0].start == 10 && s_map.rules[0].count == 1);
    CHECK(s_map.rules[1].start == 13);
    CHECK(s_map.rules[2].start == 14 && s_map.rules[2].count == 1);
    CHECK(map_find_rule(&s_map, MAP_HOLDING, 12) == -1);
    CHECK(map_read_regs(&s_map, MAP_HOLDING, 12, 2, out) && out[0] == 0x4148 && out[1] == 9);

    // Running out of rules fails without changing anything
    map_clear(&s_map);
    map_add_block(&s_map, MAP_HOLDING, 0, 1000, false);
    for (int i = 0; i < MAP_MAX_RULES; i++) {
        map_rule_t one = static_rule(i * 2, 1, i, 0);
        CHECK(map_apply_rule(&s_map, &one, 0, NULL) == MAP_OK);
    }
    map_rule_t extra = static_rule(999, 1, 0, 0);
    CHECK(map_apply_rule(&s_map, &extra, 0, NULL) == MAP_ERR_FULL);
    // Replacing one exactly needs no new room
    map_rule_t replace = static_rule(0, 1, 5, 0);
    CHECK(map_apply_rule(&s_map, &replace, 0, NULL) == MAP_OK);
    // Splitting one needs room for its far piece
    map_rule_t big = static_rule(600, 10, 0, 0);
    map_clear(&s_map);
    map_add_block(&s_map, MAP_HOLDING, 0, 1000, false);
    for (int i = 0; i < MAP_MAX_RULES - 1; i++) {
        map_rule_t one = static_rule(i * 2, 1, i, 0);
        map_apply_rule(&s_map, &one, 0, NULL);
    }
    CHECK(map_apply_rule(&s_map, &big, 0, NULL) == MAP_OK);
    CHECK(s_map.rule_count == MAP_MAX_RULES);
    map_rule_t split = static_rule(603, 1, 0, 0);
    CHECK(map_apply_rule(&s_map, &split, 0, NULL) == MAP_ERR_FULL);
    CHECK(s_map.rule_count == MAP_MAX_RULES);
    CHECK(s_map.rules[MAP_MAX_RULES - 1].count == 10);

    // Rules in one table don't trim another's
    map_clear(&s_map);
    map_add_block(&s_map, MAP_HOLDING, 0, 10, false);
    map_add_block(&s_map, MAP_INPUT, 0, 10, false);
    map_rule_t holding = static_rule(0, 10, 1, 0);
    map_rule_t input = holding;
    input.kind = MAP_INPUT;
    map_apply_rule(&s_map, &holding, 0, NULL);
    map_apply_rule(&s_map, &input, 0, NULL);
    CHECK(s_map.rule_count == 2);
}

static void test_open_mode(void) {
    uint16_t out[4];
    uint8_t bits[2];
    uint16_t value;
    bool bit;

    // Every address answers, reading 0, and the map stays empty until something is stored
    map_clear(&s_map);
    CHECK(!s_map.strict);
    CHECK(map_read_regs(&s_map, MAP_HOLDING, 4400, 1, out) && out[0] == 0);
    CHECK(map_read_regs(&s_map, MAP_INPUT, 65532, 4, out));
    CHECK(map_read_bits(&s_map, MAP_COILS, 65530, 6, bits) && bits[0] == 0);
    CHECK(map_peek_reg(&s_map, MAP_HOLDING, 9, &value) && value == 0);
    CHECK(map_peek_bit(&s_map, MAP_DISCRETE, 9, &bit) && !bit);
    CHECK(s_map.block_count == 0);

    // A rule gets storage in automatic, page-aligned blocks
    map_rule_t rule = static_rule(100, 10, 7, 0);
    CHECK(map_apply_rule(&s_map, &rule, 0, NULL) == MAP_OK);
    CHECK(s_map.block_count == 1);
    CHECK(s_map.blocks[0].start == 64 && s_map.blocks[0].count == MAP_PAGE);
    CHECK(s_map.blocks[0].flags == MAP_BLOCK_AUTO);
    CHECK(map_read_regs(&s_map, MAP_HOLDING, 99, 3, out));
    CHECK(out[0] == 0 && out[1] == 7 && out[2] == 7);

    // Spanning pages adds one block per page
    rule = static_rule(120, 20, 1, 0);   // 120-139, across 128
    CHECK(map_apply_rule(&s_map, &rule, 0, NULL) == MAP_OK);
    CHECK(s_map.block_count == 2);
    CHECK(s_map.blocks[1].start == 128 && s_map.blocks[1].count == MAP_PAGE);

    // Automatic blocks fit around blocks that are already there
    map_clear(&s_map);
    CHECK(map_add_block(&s_map, MAP_HOLDING, 10, 5, 0) == MAP_OK);
    CHECK(map_add_block(&s_map, MAP_HOLDING, 40, 5, 0) == MAP_OK);
    rule = static_rule(0, 64, 3, 0);
    CHECK(map_apply_rule(&s_map, &rule, 0, NULL) == MAP_OK);
    CHECK(s_map.block_count == 5);
    CHECK(s_map.blocks[0].start == 0 && s_map.blocks[0].count == 10);
    CHECK(s_map.blocks[2].start == 15 && s_map.blocks[2].count == 25);
    CHECK(s_map.blocks[4].start == 45 && s_map.blocks[4].count == 19);
    CHECK(map_read_regs(&s_map, MAP_HOLDING, 0, 1, out) && out[0] == 3);
    CHECK(map_read_regs(&s_map, MAP_HOLDING, 63, 1, out) && out[0] == 3);

    // The last page stops at 65535
    map_clear(&s_map);
    rule = static_rule(65535, 1, 9, 0);
    CHECK(map_apply_rule(&s_map, &rule, 0, NULL) == MAP_OK);
    CHECK(s_map.blocks[0].start == 65472 && s_map.blocks[0].count == MAP_PAGE);

    // Bits too
    map_clear(&s_map);
    map_rule_t coils = {.count = 3, .start = 70, .kind = MAP_COILS, .type = MAP_TYPE_BOOL,
                        .value = 1};
    CHECK(map_apply_rule(&s_map, &coils, 0, NULL) == MAP_OK);
    CHECK(s_map.blocks[0].start == 64 && s_map.bits_used == MAP_PAGE);
    CHECK(map_read_bits(&s_map, MAP_COILS, 69, 5, bits) && bits[0] == 0x0E);

    // Running out of pool leaves everything as it was
    map_clear(&s_map);
    CHECK(map_add_block(&s_map, MAP_INPUT, 0, MAP_REG_POOL - 32, 0) == MAP_OK);
    rule = static_rule(1000, 1, 5, 0);
    CHECK(map_apply_rule(&s_map, &rule, 0, NULL) == MAP_ERR_FULL);
    CHECK(s_map.block_count == 1 && s_map.rule_count == 0);

    // Switching to strict keeps the storage, but addresses outside it now fail
    map_clear(&s_map);
    rule = static_rule(100, 1, 5, 0);
    CHECK(map_apply_rule(&s_map, &rule, 0, NULL) == MAP_OK);
    s_map.strict = true;
    CHECK(map_read_regs(&s_map, MAP_HOLDING, 100, 1, out) && out[0] == 5);
    CHECK(!map_read_regs(&s_map, MAP_HOLDING, 4400, 1, out));
    CHECK(!map_peek_reg(&s_map, MAP_HOLDING, 4400, &value));
    rule = static_rule(4400, 1, 5, 0);
    CHECK(map_apply_rule(&s_map, &rule, 0, NULL) == MAP_ERR_UNMAPPED);
}

// ---- Encoding

static void check_encode(uint8_t type, uint8_t order, double value, int count,
                         uint16_t w0, uint16_t w1, uint16_t w2, uint16_t w3) {
    const uint16_t expected[MAP_WORDS_MAX] = {w0, w1, w2, w3};
    uint16_t words[MAP_WORDS_MAX] = {0};

    CHECK(map_type_words(type) == count);
    map_encode(type, order, value, words);
    CHECK(memcmp(words, expected, sizeof(words)) == 0);
}

static void test_encoding(void) {
    check_encode(MAP_TYPE_UINT16, MAP_ORDER_ABCD, 0x0102, 1, 0x0102, 0, 0, 0);
    check_encode(MAP_TYPE_UINT16, MAP_ORDER_CDAB, 0x0102, 1, 0x0102, 0, 0, 0);
    check_encode(MAP_TYPE_UINT16, MAP_ORDER_BADC, 0x0102, 1, 0x0201, 0, 0, 0);
    check_encode(MAP_TYPE_UINT16, MAP_ORDER_DCBA, 0x0102, 1, 0x0201, 0, 0, 0);
    check_encode(MAP_TYPE_INT16, MAP_ORDER_ABCD, -2, 1, 0xFFFE, 0, 0, 0);

    // Integer types round and clamp
    check_encode(MAP_TYPE_UINT16, MAP_ORDER_ABCD, 1.6, 1, 2, 0, 0, 0);
    check_encode(MAP_TYPE_UINT16, MAP_ORDER_ABCD, 70000, 1, 0xFFFF, 0, 0, 0);
    check_encode(MAP_TYPE_INT16, MAP_ORDER_ABCD, -40000, 1, 0x8000, 0, 0, 0);

    check_encode(MAP_TYPE_UINT32, MAP_ORDER_ABCD, 3000000000.0, 2, 0xB2D0, 0x5E00, 0, 0);
    check_encode(MAP_TYPE_INT32, MAP_ORDER_ABCD, -2, 2, 0xFFFF, 0xFFFE, 0, 0);
    check_encode(MAP_TYPE_INT32, MAP_ORDER_CDAB, 0x01020304, 2, 0x0304, 0x0102, 0, 0);

    // 12.5 is 0x41480000
    check_encode(MAP_TYPE_FLOAT32, MAP_ORDER_ABCD, 12.5, 2, 0x4148, 0x0000, 0, 0);
    check_encode(MAP_TYPE_FLOAT32, MAP_ORDER_CDAB, 12.5, 2, 0x0000, 0x4148, 0, 0);
    check_encode(MAP_TYPE_FLOAT32, MAP_ORDER_BADC, 12.5, 2, 0x4841, 0x0000, 0, 0);
    check_encode(MAP_TYPE_FLOAT32, MAP_ORDER_DCBA, 12.5, 2, 0x0000, 0x4841, 0, 0);

    // 1.5 is 0x3FF8000000000000
    check_encode(MAP_TYPE_FLOAT64, MAP_ORDER_ABCD, 1.5, 4, 0x3FF8, 0, 0, 0);
    check_encode(MAP_TYPE_FLOAT64, MAP_ORDER_CDAB, 1.5, 4, 0, 0, 0, 0x3FF8);
    check_encode(MAP_TYPE_FLOAT64, MAP_ORDER_BADC, 1.5, 4, 0xF83F, 0, 0, 0);
    check_encode(MAP_TYPE_FLOAT64, MAP_ORDER_DCBA, 1.5, 4, 0, 0, 0, 0xF83F);

    check_encode(MAP_TYPE_BOOL, MAP_ORDER_ABCD, 1, 1, 1, 0, 0, 0);
    check_encode(MAP_TYPE_BOOL, MAP_ORDER_ABCD, 0.4, 1, 0, 0, 0, 0);
}

// ---- Emulation

static void test_static_rules(void) {
    uint16_t out[4];

    map_clear(&s_map);
    map_add_block(&s_map, MAP_HOLDING, 0, 20, false);

    map_rule_t reals = {.count = 2, .start = 0, .kind = MAP_HOLDING, .type = MAP_TYPE_FLOAT32,
                        .value = 12.5, .value_step = -13.5};
    CHECK(map_apply_rule(&s_map, &reals, 0, NULL) == MAP_OK);
    CHECK(map_read_regs(&s_map, MAP_HOLDING, 0, 4, out));
    CHECK(out[0] == 0x4148 && out[1] == 0 && out[2] == 0xBF80 && out[3] == 0);   // 12.5, -1

    // A static value is set once, so later changes to its registers stick through ticks
    s_map.regs[0] = 0x1234;
    map_tick(&s_map, 5000, NULL);
    CHECK(map_read_regs(&s_map, MAP_HOLDING, 0, 1, out) && out[0] == 0x1234);
}

static void test_sine(void) {
    uint16_t out[2];

    map_clear(&s_map);
    map_add_block(&s_map, MAP_INPUT, 0, 10, false);
    map_rule_t rule = {.count = 1, .kind = MAP_INPUT, .type = MAP_TYPE_UINT16, .mode = EMU_SINE,
                       .min = 0, .max = 1000, .period_ms = 10000};
    CHECK(map_apply_rule(&s_map, &rule, 0, NULL) == MAP_OK);

    const uint32_t times[] = {0, 2500, 5000, 7500, 10000};
    const uint16_t expected[] = {500, 1000, 500, 0, 500};
    for (int i = 0; i < 5; i++) {
        map_tick(&s_map, times[i], NULL);
        CHECK(map_read_regs(&s_map, MAP_INPUT, 0, 1, out) && out[0] == expected[i]);
    }

    // Each value is a phase step further on
    rule.count = 4;
    rule.phase_step_deg = 90;
    CHECK(map_apply_rule(&s_map, &rule, 0, NULL) == MAP_OK);
    uint16_t four[4];
    CHECK(map_read_regs(&s_map, MAP_INPUT, 0, 4, four));
    CHECK(four[0] == 500 && four[1] == 1000 && four[2] == 500 && four[3] == 0);

    // Fractional values survive in float types, across both words
    rule = (map_rule_t){.count = 1, .kind = MAP_INPUT, .type = MAP_TYPE_FLOAT32,
                        .mode = EMU_SINE, .min = 0, .max = 1, .period_ms = 10000};
    CHECK(map_apply_rule(&s_map, &rule, 0, NULL) == MAP_OK);
    CHECK(map_read_regs(&s_map, MAP_INPUT, 0, 2, out) && out[0] == 0x3F00 && out[1] == 0);

    // The widest ranges don't overflow
    rule.type = MAP_TYPE_FLOAT64;
    rule.min = -DBL_MAX;
    rule.max = DBL_MAX;
    CHECK(map_apply_rule(&s_map, &rule, 2500, NULL) == MAP_OK);
    CHECK(map_read_regs(&s_map, MAP_INPUT, 0, 1, out) && (out[0] & 0x7FF0) != 0x7FF0);
}

static void test_square(void) {
    uint16_t out[1];
    uint8_t bits[1];

    map_clear(&s_map);
    map_add_block(&s_map, MAP_HOLDING, 0, 4, false);
    map_add_block(&s_map, MAP_DISCRETE, 0, 4, false);

    map_rule_t rule = {.count = 1, .kind = MAP_HOLDING, .type = MAP_TYPE_UINT16,
                       .mode = EMU_SQUARE, .min = 10, .max = 20, .period_ms = 1000,
                       .duty_pct = 25};
    CHECK(map_apply_rule(&s_map, &rule, 0, NULL) == MAP_OK);
    map_tick(&s_map, 0, NULL);
    CHECK(map_read_regs(&s_map, MAP_HOLDING, 0, 1, out) && out[0] == 20);
    map_tick(&s_map, 249, NULL);
    CHECK(map_read_regs(&s_map, MAP_HOLDING, 0, 1, out) && out[0] == 20);
    map_tick(&s_map, 250, NULL);
    CHECK(map_read_regs(&s_map, MAP_HOLDING, 0, 1, out) && out[0] == 10);
    map_tick(&s_map, 1000, NULL);
    CHECK(map_read_regs(&s_map, MAP_HOLDING, 0, 1, out) && out[0] == 20);

    // Bits blink, and a phase step staggers them: 180 degrees apart here
    rule = (map_rule_t){.count = 2, .kind = MAP_DISCRETE, .type = MAP_TYPE_BOOL,
                        .mode = EMU_SQUARE, .min = 0, .max = 1, .period_ms = 1000,
                        .duty_pct = 50, .phase_step_deg = 180};
    CHECK(map_apply_rule(&s_map, &rule, 0, NULL) == MAP_OK);
    map_tick(&s_map, 100, NULL);
    CHECK(map_read_bits(&s_map, MAP_DISCRETE, 0, 2, bits) && bits[0] == 0x01);
    map_tick(&s_map, 600, NULL);
    CHECK(map_read_bits(&s_map, MAP_DISCRETE, 0, 2, bits) && bits[0] == 0x02);

    // Duty extremes are always off and always on
    rule.count = 1;
    rule.phase_step_deg = 0;
    rule.duty_pct = 0;
    CHECK(map_apply_rule(&s_map, &rule, 0, NULL) == MAP_OK);
    for (uint32_t t = 0; t < 2000; t += 50) {
        map_tick(&s_map, t, NULL);
        CHECK(map_read_bits(&s_map, MAP_DISCRETE, 0, 1, bits) && bits[0] == 0);
    }
    rule.duty_pct = 100;
    CHECK(map_apply_rule(&s_map, &rule, 0, NULL) == MAP_OK);
    for (uint32_t t = 0; t < 2000; t += 50) {
        map_tick(&s_map, t, NULL);
        CHECK(map_read_bits(&s_map, MAP_DISCRETE, 0, 1, bits) && bits[0] == 1);
    }
}

static void test_random(void) {
    uint16_t out[2];
    uint8_t bits[1];

    map_clear(&s_map);
    map_add_block(&s_map, MAP_INPUT, 0, 4, false);
    map_add_block(&s_map, MAP_COILS, 0, 8, false);

    map_rule_t rule = {.count = 1, .kind = MAP_INPUT, .type = MAP_TYPE_INT16,
                       .mode = EMU_RANDOM, .min = -5, .max = 5, .period_ms = 1000};
    s_random_calls = 0;
    s_random_value = 0;

    // A new rule updates straight away, then holds for its period
    CHECK(map_apply_rule(&s_map, &rule, 5000, fake_random) == MAP_OK);
    CHECK(s_random_calls == 1);
    CHECK(map_read_regs(&s_map, MAP_INPUT, 0, 1, out) && out[0] == (uint16_t)-5);

    s_random_value = 0xFFFFFFFFu;
    map_tick(&s_map, 5999, fake_random);
    CHECK(s_random_calls == 1);
    map_tick(&s_map, 6000, fake_random);
    CHECK(s_random_calls == 2);
    CHECK(map_read_regs(&s_map, MAP_INPUT, 0, 1, out) && out[0] == 5);

    // Floats land between the bounds
    rule = (map_rule_t){.count = 1, .kind = MAP_INPUT, .type = MAP_TYPE_FLOAT32,
                        .mode = EMU_RANDOM, .min = 0, .max = 1, .period_ms = 1000};
    s_random_value = 0x80000000u;
    CHECK(map_apply_rule(&s_map, &rule, 8000, fake_random) == MAP_OK);
    CHECK(map_read_regs(&s_map, MAP_INPUT, 0, 2, out) && out[0] == 0x3F00 && out[1] == 0);

    // Bits get a random state each
    rule = (map_rule_t){.count = 8, .kind = MAP_COILS, .type = MAP_TYPE_BOOL,
                        .mode = EMU_RANDOM, .min = 0, .max = 1, .period_ms = 1000};
    s_random_value = 0xFFFFFFFFu;
    CHECK(map_apply_rule(&s_map, &rule, 0, fake_random) == MAP_OK);
    CHECK(map_read_bits(&s_map, MAP_COILS, 0, 8, bits) && bits[0] == 0xFF);
    s_random_value = 0;
    map_tick(&s_map, 1000, fake_random);
    CHECK(map_read_bits(&s_map, MAP_COILS, 0, 8, bits) && bits[0] == 0x00);

    // Deadlines survive the millisecond clock wrapping
    size_t index = (size_t)map_find_rule(&s_map, MAP_COILS, 0);
    s_map.next_random_ms[index] = 500;
    s_random_calls = 0;
    map_tick(&s_map, 0xFFFFFF00u, fake_random);
    CHECK(s_random_calls == 0);
    map_tick(&s_map, 600, fake_random);
    CHECK(s_random_calls == 8);
}

// ---- WiFi LED

static int count_rising_edges(wifi_led_state_t state, uint32_t start_ms, uint32_t span_ms) {
    int edges = 0;
    bool last = wifi_led_level(state, start_ms);

    for (uint32_t t = start_ms + 50; t < start_ms + span_ms; t += 50) {
        bool level = wifi_led_level(state, t);
        edges += level && !last;
        last = level;
    }
    return edges;
}

static void test_wifi_led(void) {
    // Connected beats everything else
    CHECK(wifi_led_state(true, true, false, 0) == WIFI_LED_CONNECTED);
    CHECK(wifi_led_state(true, true, true, 999999) == WIFI_LED_CONNECTED);

    CHECK(wifi_led_state(false, true, false, 0) == WIFI_LED_CONNECTING);
    CHECK(wifi_led_state(false, true, false, WIFI_LED_FAULT_AFTER_MS - 1) == WIFI_LED_CONNECTING);

    // Each way into a fault
    CHECK(wifi_led_state(false, false, false, 0) == WIFI_LED_FAULT);
    CHECK(wifi_led_state(false, true, true, 0) == WIFI_LED_FAULT);
    CHECK(wifi_led_state(false, true, false, WIFI_LED_FAULT_AFTER_MS) == WIFI_LED_FAULT);

    // Solid never turns off
    for (uint32_t t = 0; t < 10000; t += 50) {
        CHECK(wifi_led_level(WIFI_LED_CONNECTED, t));
    }

    // One second on, one second off
    CHECK(wifi_led_level(WIFI_LED_CONNECTING, 0));
    CHECK(wifi_led_level(WIFI_LED_CONNECTING, 950));
    CHECK(!wifi_led_level(WIFI_LED_CONNECTING, 1000));
    CHECK(!wifi_led_level(WIFI_LED_CONNECTING, 1950));
    CHECK(wifi_led_level(WIFI_LED_CONNECTING, 2000));
    CHECK(count_rising_edges(WIFI_LED_CONNECTING, 100, 10000) == 5);

    // Two flashes, each visible at the tick rate, then a pause
    CHECK(wifi_led_level(WIFI_LED_FAULT, 100));
    CHECK(!wifi_led_level(WIFI_LED_FAULT, 200));
    CHECK(wifi_led_level(WIFI_LED_FAULT, 350));
    CHECK(!wifi_led_level(WIFI_LED_FAULT, 500));
    CHECK(!wifi_led_level(WIFI_LED_FAULT, 1400));
    CHECK(count_rising_edges(WIFI_LED_FAULT, 1000, 1500) == 2);

    // Faults blink faster than connecting, so the two can't be confused
    CHECK(count_rising_edges(WIFI_LED_FAULT, 1000, 15000) >
          2 * count_rising_edges(WIFI_LED_CONNECTING, 1000, 15000));

    // Down since just before the millisecond clock wrapped, and it's 16 ms after
    uint32_t down_since_ms = 0xFFFFFFF0u;
    uint32_t now_ms = 16;
    CHECK(wifi_led_state(false, true, false, now_ms - down_since_ms) == WIFI_LED_CONNECTING);
}

int main(void) {
    test_crc();
    test_read_defaults();
    test_exceptions();
    test_silent_frames();
    test_read_bits();
    test_blocks();
    test_rule_checks();
    test_rule_trimming();
    test_open_mode();
    test_encoding();
    test_static_rules();
    test_sine();
    test_square();
    test_random();
    test_wifi_led();

    printf("%d checks, %d failed\n", s_checks, s_failures);
    return s_failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
