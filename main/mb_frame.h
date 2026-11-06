// Modbus RTU frame handling. No ESP-IDF dependencies, so it can be tested on a host.
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define MB_FRAME_MAX 256

#define MB_FC_READ_COILS    0x01
#define MB_FC_READ_DISCRETE 0x02
#define MB_FC_READ_HOLDING  0x03
#define MB_FC_READ_INPUT    0x04

#define MB_EX_ILLEGAL_FUNCTION 0x01
#define MB_EX_ILLEGAL_ADDRESS  0x02
#define MB_EX_ILLEGAL_VALUE    0x03

typedef enum {
    MB_RESULT_REPLY,        // resp holds a frame to send
    MB_RESULT_BAD_FRAME,    // too short or bad CRC, stay silent
    MB_RESULT_OTHER_SLAVE,  // valid frame for another address or broadcast, stay silent
} mb_result_t;

typedef struct {
    uint8_t fc;
    uint8_t exception;  // 0 when the request succeeded
    uint16_t addr;
    uint16_t qty;
} mb_request_t;

// Each returns false if any address in the range is undefined. fc is the function code
// being served. Bits are packed the Modbus way: the first address in bit 0 of out[0].
typedef struct {
    bool (*read_regs)(void *ctx, uint8_t fc, uint16_t addr, uint16_t qty, uint16_t *out);
    bool (*read_bits)(void *ctx, uint8_t fc, uint16_t addr, uint16_t qty, uint8_t *out);
    void *ctx;
} mb_handlers_t;

uint16_t mb_crc16(const uint8_t *data, size_t len);

// resp must hold MB_FRAME_MAX bytes. info is only filled in for MB_RESULT_REPLY.
mb_result_t mb_handle_frame(const uint8_t *req, size_t len, uint8_t slave_id,
                            const mb_handlers_t *handlers,
                            uint8_t *resp, size_t *resp_len, mb_request_t *info);
