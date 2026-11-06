// Modbus RTU slave on the UART. Runs in its own task, independent of WiFi.
#pragma once

#include <stdint.h>

#include "mb_frame.h"
#include "settings.h"

#define MB_RECENT_MAX 8

#define MB_CAPTURE_MAX   32
#define MB_CAPTURE_BYTES 64

// Flags on a captured frame
#define MB_CAP_TX      0x01  // sent by the emulator, otherwise received
#define MB_CAP_PARITY  0x02
#define MB_CAP_FRAMING 0x04
#define MB_CAP_BREAK   0x08
#define MB_CAP_OVERRUN 0x10
#define MB_CAP_CRC_OK  0x20  // bytes pass the CRC, whatever the UART flagged

typedef struct {
    uint32_t time_ms;
    uint16_t len;    // bytes in the frame, which may be more than were kept
    uint8_t flags;
    uint8_t data[MB_CAPTURE_BYTES];
} mb_capture_t;

typedef struct {
    mb_request_t request;
    uint32_t time_ms;
} mb_recent_t;

typedef struct {
    uint32_t rx_frames;      // valid frames addressed to this slave
    uint32_t tx_frames;
    uint32_t exceptions;
    uint32_t crc_errors;
    uint32_t parity_errors;
    uint32_t framing_errors;
    uint32_t breaks;
    uint32_t overruns;
    uint32_t other_slave;
    mb_recent_t recent[MB_RECENT_MAX];  // oldest to newest
    uint8_t recent_count;
} mb_stats_t;

void modbus_slave_start(const serial_cfg_t *cfg);

// Takes effect between frames, with no restart
void modbus_slave_apply(const serial_cfg_t *cfg);

void modbus_slave_get_config(serial_cfg_t *cfg);
void modbus_slave_get_stats(mb_stats_t *stats);

// Copies captured frames, oldest first, into out, which holds MB_CAPTURE_MAX entries.
// Returns the number copied.
size_t modbus_slave_get_capture(mb_capture_t *out);

// Clears the counters, recent requests and capture
void modbus_slave_reset_stats(void);
