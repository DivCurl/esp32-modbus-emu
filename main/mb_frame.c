#include "mb_frame.h"

#define MB_MAX_READ_REGS 125
#define MB_MAX_READ_BITS 2000

uint16_t mb_crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; bit++) {
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
        }
    }
    return crc;
}

static size_t finish_frame(uint8_t *resp, size_t len) {
    uint16_t crc = mb_crc16(resp, len);
    resp[len++] = crc & 0xFF;  // CRC is the one little-endian field in RTU
    resp[len++] = crc >> 8;
    return len;
}

static size_t exception_frame(uint8_t *resp, uint8_t id, uint8_t fc, uint8_t code) {
    resp[0] = id;
    resp[1] = fc | 0x80;
    resp[2] = code;
    return finish_frame(resp, 3);
}

// Checks the address and quantity fields shared by every read
static uint8_t parse_read(const uint8_t *req, size_t len, mb_request_t *info, uint16_t max_qty) {
    if (len != 8) {
        return MB_EX_ILLEGAL_VALUE;
    }

    info->addr = (req[2] << 8) | req[3];
    info->qty = (req[4] << 8) | req[5];

    if (info->qty == 0 || info->qty > max_qty) {
        return MB_EX_ILLEGAL_VALUE;
    }
    if ((uint32_t)info->addr + info->qty > 0x10000) {
        return MB_EX_ILLEGAL_ADDRESS;
    }
    return 0;
}

static uint8_t read_registers(const uint8_t *req, size_t len, mb_request_t *info,
                              const mb_handlers_t *handlers, uint8_t *resp, size_t *resp_len) {
    uint8_t exception = parse_read(req, len, info, MB_MAX_READ_REGS);
    if (exception != 0) {
        return exception;
    }

    uint16_t values[MB_MAX_READ_REGS];
    if (!handlers->read_regs(handlers->ctx, info->fc, info->addr, info->qty, values)) {
        return MB_EX_ILLEGAL_ADDRESS;
    }

    resp[0] = req[0];
    resp[1] = info->fc;
    resp[2] = info->qty * 2;
    for (uint16_t i = 0; i < info->qty; i++) {
        resp[3 + i * 2] = values[i] >> 8;
        resp[4 + i * 2] = values[i] & 0xFF;
    }
    *resp_len = finish_frame(resp, 3 + info->qty * 2);
    return 0;
}

static uint8_t read_bits(const uint8_t *req, size_t len, mb_request_t *info,
                         const mb_handlers_t *handlers, uint8_t *resp, size_t *resp_len) {
    uint8_t exception = parse_read(req, len, info, MB_MAX_READ_BITS);
    if (exception != 0) {
        return exception;
    }

    uint8_t bytes = (info->qty + 7) / 8;
    if (!handlers->read_bits(handlers->ctx, info->fc, info->addr, info->qty, &resp[3])) {
        return MB_EX_ILLEGAL_ADDRESS;
    }

    resp[0] = req[0];
    resp[1] = info->fc;
    resp[2] = bytes;
    *resp_len = finish_frame(resp, 3 + bytes);
    return 0;
}

mb_result_t mb_handle_frame(const uint8_t *req, size_t len, uint8_t slave_id,
                            const mb_handlers_t *handlers,
                            uint8_t *resp, size_t *resp_len, mb_request_t *info) {
    if (len < 4 || len > MB_FRAME_MAX) {
        return MB_RESULT_BAD_FRAME;
    }

    uint16_t received = req[len - 2] | (req[len - 1] << 8);
    if (mb_crc16(req, len - 2) != received) {
        return MB_RESULT_BAD_FRAME;
    }

    if (req[0] != slave_id) {
        return MB_RESULT_OTHER_SLAVE;
    }

    *info = (mb_request_t){.fc = req[1]};

    switch (info->fc) {
        case MB_FC_READ_COILS:
        case MB_FC_READ_DISCRETE:
            info->exception = read_bits(req, len, info, handlers, resp, resp_len);
            break;
        case MB_FC_READ_HOLDING:
        case MB_FC_READ_INPUT:
            info->exception = read_registers(req, len, info, handlers, resp, resp_len);
            break;
        default:
            info->exception = MB_EX_ILLEGAL_FUNCTION;
            break;
    }

    if (info->exception != 0) {
        *resp_len = exception_frame(resp, req[0], info->fc, info->exception);
    }
    return MB_RESULT_REPLY;
}
