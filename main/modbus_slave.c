#include "modbus_slave.h"

#include <stdbool.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "soc/uart_reg.h"

#include "app_config.h"
#include "leds.h"
#include "map_store.h"

#define UART_RX_BUFFER   1024
#define UART_QUEUE_LEN   20

// RTU ends a frame after 3.5 silent characters. The hardware counts whole symbols.
#define UART_FRAME_GAP_SYMBOLS 4

// Data normally arrives with the end-of-frame timeout. Only a frame ending exactly as the
// FIFO fills misses it, so wait out more than a FIFO's worth of characters before giving up.
#define IDLE_FLUSH_CHARS 130
#define BITS_PER_CHAR    11

// Longest time LEDs and pending changes wait while the line is idle
#define LOOP_WAIT_MS     5

#define MB_TASK_STACK    4096
#define MB_TASK_PRIORITY 20
#define MB_TASK_CORE     1  // WiFi runs on core 0

static const char *TAG = "MODBUS";

static QueueHandle_t s_uart_queue;

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static serial_cfg_t s_cfg;
static serial_cfg_t s_pending;
static bool s_pending_valid;
static mb_stats_t s_stats;
static mb_capture_t s_capture[MB_CAPTURE_MAX];
static size_t s_capture_next;
static size_t s_capture_count;

// Frame being assembled. Only touched by the Modbus task.
static uint8_t s_frame[MB_FRAME_MAX];
static size_t s_frame_len;
static uint8_t s_frame_flags;
static TickType_t s_last_event;
static TickType_t s_idle_flush_ticks;

static uart_parity_t uart_parity(uint8_t parity) {
    switch (parity) {
        case 'E': return UART_PARITY_EVEN;
        case 'O': return UART_PARITY_ODD;
        default:  return UART_PARITY_DISABLE;
    }
}

static uart_stop_bits_t uart_stop_bits(uint8_t stop_bits) {
    return stop_bits == 2 ? UART_STOP_BITS_2 : UART_STOP_BITS_1;
}

static void reset_frame(void) {
    s_frame_len = 0;
    s_frame_flags = 0;
}

static void set_idle_flush(uint32_t baud) {
    s_idle_flush_ticks = pdMS_TO_TICKS(IDLE_FLUSH_CHARS * BITS_PER_CHAR * 1000 / baud + 1);
}

static void uart_open(const serial_cfg_t *cfg) {
    uart_config_t config = {
        .baud_rate = cfg->baud,
        .data_bits = UART_DATA_8_BITS,
        .parity = uart_parity(cfg->parity),
        .stop_bits = uart_stop_bits(cfg->stop_bits),
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // The ISR binds to the calling core, so this has to run in the Modbus task
    ESP_ERROR_CHECK(uart_driver_install(EMU_UART_PORT, UART_RX_BUFFER, 0, UART_QUEUE_LEN,
                                        &s_uart_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(EMU_UART_PORT, &config));
    ESP_ERROR_CHECK(uart_set_pin(EMU_UART_PORT, EMU_PIN_UART_TX, EMU_PIN_UART_RX,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_set_rx_timeout(EMU_UART_PORT, UART_FRAME_GAP_SYMBOLS));

    // The driver leaves framing errors off by default
    ESP_ERROR_CHECK(uart_enable_intr_mask(EMU_UART_PORT, UART_FRM_ERR_INT_ENA_M));
    set_idle_flush(cfg->baud);
}

static void uart_reconfigure(const serial_cfg_t *cfg) {
    uart_wait_tx_done(EMU_UART_PORT, pdMS_TO_TICKS(100));
    ESP_ERROR_CHECK(uart_set_baudrate(EMU_UART_PORT, cfg->baud));
    ESP_ERROR_CHECK(uart_set_parity(EMU_UART_PORT, uart_parity(cfg->parity)));
    ESP_ERROR_CHECK(uart_set_stop_bits(EMU_UART_PORT, uart_stop_bits(cfg->stop_bits)));
    ESP_ERROR_CHECK(uart_set_rx_timeout(EMU_UART_PORT, UART_FRAME_GAP_SYMBOLS));
    ESP_ERROR_CHECK(uart_flush_input(EMU_UART_PORT));
    xQueueReset(s_uart_queue);
    reset_frame();
    set_idle_flush(cfg->baud);
    ESP_LOGI(TAG, "Serial now %lu 8%c%u, slave %u", (unsigned long)cfg->baud, cfg->parity,
             cfg->stop_bits, cfg->slave_id);
}

static void apply_pending(void) {
    serial_cfg_t cfg;
    bool pending;

    taskENTER_CRITICAL(&s_mux);
    pending = s_pending_valid;
    cfg = s_pending;
    s_pending_valid = false;
    taskEXIT_CRITICAL(&s_mux);

    if (pending) {
        uart_reconfigure(&cfg);
        taskENTER_CRITICAL(&s_mux);
        s_cfg = cfg;
        taskEXIT_CRITICAL(&s_mux);
    }
}

// Call inside the critical section
static void record_request(const mb_request_t *request) {
    if (s_stats.recent_count == MB_RECENT_MAX) {
        memmove(&s_stats.recent[0], &s_stats.recent[1],
                (MB_RECENT_MAX - 1) * sizeof(s_stats.recent[0]));
        s_stats.recent_count--;
    }
    s_stats.recent[s_stats.recent_count].request = *request;
    s_stats.recent[s_stats.recent_count].time_ms = (uint32_t)(esp_timer_get_time() / 1000);
    s_stats.recent_count++;
}

// Call inside the critical section
static void capture_frame(const uint8_t *data, size_t len, uint8_t flags) {
    mb_capture_t *entry = &s_capture[s_capture_next];
    size_t kept = len < MB_CAPTURE_BYTES ? len : MB_CAPTURE_BYTES;

    entry->time_ms = (uint32_t)(esp_timer_get_time() / 1000);
    entry->len = len;
    entry->flags = flags;
    memcpy(entry->data, data, kept);

    s_capture_next = (s_capture_next + 1) % MB_CAPTURE_MAX;
    if (s_capture_count < MB_CAPTURE_MAX) {
        s_capture_count++;
    }
}

static bool frame_crc_ok(const uint8_t *frame, size_t len) {
    if (len < 4 || len > MB_FRAME_MAX) {
        return false;
    }
    return mb_crc16(frame, len - 2) == (frame[len - 2] | (frame[len - 1] << 8));
}

static bool read_registers(void *ctx, uint8_t fc, uint16_t addr, uint16_t qty, uint16_t *out) {
    return store_read_regs(fc == MB_FC_READ_INPUT ? MAP_INPUT : MAP_HOLDING, addr, qty, out);
}

static bool read_bits(void *ctx, uint8_t fc, uint16_t addr, uint16_t qty, uint8_t *out) {
    return store_read_bits(fc == MB_FC_READ_COILS ? MAP_COILS : MAP_DISCRETE, addr, qty, out);
}

static const mb_handlers_t HANDLERS = {
    .read_regs = read_registers,
    .read_bits = read_bits,
};

static void process_frame(void) {
    static uint8_t resp[MB_FRAME_MAX];
    size_t resp_len = 0;
    mb_request_t request = {0};
    mb_result_t result = MB_RESULT_BAD_FRAME;

    // Modbus discards frames with UART errors even if the CRC happens to pass. A break
    // says nothing about the bytes around it, so that one is left to the CRC.
    bool rejected = (s_frame_flags & (MB_CAP_PARITY | MB_CAP_FRAMING | MB_CAP_OVERRUN)) != 0;
    uint8_t flags = s_frame_flags;

    if (frame_crc_ok(s_frame, s_frame_len)) {
        flags |= MB_CAP_CRC_OK;
    }

    if (!rejected) {
        result = mb_handle_frame(s_frame, s_frame_len, s_cfg.slave_id, &HANDLERS,
                                 resp, &resp_len, &request);
    }

    taskENTER_CRITICAL(&s_mux);
    capture_frame(s_frame, s_frame_len, flags);
    switch (result) {
        case MB_RESULT_REPLY:
            s_stats.rx_frames++;
            s_stats.tx_frames++;
            s_stats.exceptions += request.exception != 0;
            record_request(&request);
            capture_frame(resp, resp_len, MB_CAP_TX | MB_CAP_CRC_OK);
            break;
        case MB_RESULT_OTHER_SLAVE:
            s_stats.other_slave++;
            break;
        case MB_RESULT_BAD_FRAME:
            // UART errors were already counted as they happened, and an error with
            // no bytes around it isn't a CRC failure
            s_stats.crc_errors += !rejected && s_frame_len > 0;
            break;
    }
    taskEXIT_CRITICAL(&s_mux);

    if (result == MB_RESULT_REPLY) {
        leds_pulse_tx();
        uart_write_bytes(EMU_UART_PORT, resp, resp_len);
    }

    reset_frame();
}

static void count_error(uint32_t *counter) {
    taskENTER_CRITICAL(&s_mux);
    (*counter)++;
    taskEXIT_CRITICAL(&s_mux);
}

static void handle_data(const uart_event_t *event) {
    size_t room = sizeof(s_frame) - s_frame_len;
    size_t want = event->size < room ? event->size : room;
    int got = uart_read_bytes(EMU_UART_PORT, s_frame + s_frame_len, want, 0);

    leds_pulse_rx();

    if (got > 0) {
        s_frame_len += got;
    }
    if (event->size > room) {
        uart_flush_input(EMU_UART_PORT);
        s_frame_flags |= MB_CAP_OVERRUN;
        count_error(&s_stats.overruns);
    }
    if (event->timeout_flag) {
        process_frame();
    }
}

static void handle_event(const uart_event_t *event) {
    switch (event->type) {
        case UART_DATA:
            handle_data(event);
            break;
        case UART_FIFO_OVF:
        case UART_BUFFER_FULL:
            uart_flush_input(EMU_UART_PORT);
            xQueueReset(s_uart_queue);
            reset_frame();
            count_error(&s_stats.overruns);
            break;
        case UART_PARITY_ERR:
            s_frame_flags |= MB_CAP_PARITY;
            count_error(&s_stats.parity_errors);
            break;
        case UART_FRAME_ERR:
            s_frame_flags |= MB_CAP_FRAMING;
            count_error(&s_stats.framing_errors);
            break;
        case UART_BREAK:
            s_frame_flags |= MB_CAP_BREAK;
            count_error(&s_stats.breaks);
            break;
        default:
            break;
    }
}

static void modbus_task(void *arg) {
    uart_open(&s_cfg);
    leds_init();
    ESP_LOGI(TAG, "Slave %u on UART%d at %lu 8%c%u", s_cfg.slave_id, EMU_UART_PORT,
             (unsigned long)s_cfg.baud, s_cfg.parity, s_cfg.stop_bits);

    while (1) {
        uart_event_t event;

        if (xQueueReceive(s_uart_queue, &event, pdMS_TO_TICKS(LOOP_WAIT_MS))) {
            handle_event(&event);
            s_last_event = xTaskGetTickCount();
        } else if ((s_frame_len > 0 || s_frame_flags != 0) &&
                   xTaskGetTickCount() - s_last_event >= s_idle_flush_ticks) {
            // Error events arrive mid-frame, so a short wait here would split the frame
            process_frame();
        }

        leds_service();
        apply_pending();
    }
}

void modbus_slave_start(const serial_cfg_t *cfg) {
    s_cfg = *cfg;

    BaseType_t created = xTaskCreatePinnedToCore(modbus_task, "modbus", MB_TASK_STACK, NULL,
                                                 MB_TASK_PRIORITY, NULL, MB_TASK_CORE);
    configASSERT(created == pdPASS);
}

void modbus_slave_apply(const serial_cfg_t *cfg) {
    taskENTER_CRITICAL(&s_mux);
    s_pending = *cfg;
    s_pending_valid = true;
    taskEXIT_CRITICAL(&s_mux);
}

void modbus_slave_get_config(serial_cfg_t *cfg) {
    taskENTER_CRITICAL(&s_mux);
    *cfg = s_pending_valid ? s_pending : s_cfg;
    taskEXIT_CRITICAL(&s_mux);
}

void modbus_slave_get_stats(mb_stats_t *stats) {
    taskENTER_CRITICAL(&s_mux);
    *stats = s_stats;
    taskEXIT_CRITICAL(&s_mux);
}

size_t modbus_slave_get_capture(mb_capture_t *out) {
    taskENTER_CRITICAL(&s_mux);
    size_t count = s_capture_count;
    size_t first = (s_capture_next + MB_CAPTURE_MAX - count) % MB_CAPTURE_MAX;
    for (size_t i = 0; i < count; i++) {
        out[i] = s_capture[(first + i) % MB_CAPTURE_MAX];
    }
    taskEXIT_CRITICAL(&s_mux);
    return count;
}

void modbus_slave_reset_stats(void) {
    taskENTER_CRITICAL(&s_mux);
    memset(&s_stats, 0, sizeof(s_stats));
    s_capture_next = 0;
    s_capture_count = 0;
    taskEXIT_CRITICAL(&s_mux);
}
