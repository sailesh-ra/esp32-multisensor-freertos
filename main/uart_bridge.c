/*
* uart_bridge.c -- ESP32 UART sensor bridge task
* 
* Drains uart_sensor_queue and serialises readings in whichever
* protocol is selected by ACTIVE_PROTOCOL in uart_bridge.h.
* 
* All three encoders produce the same physical data; only the
* wire representation differs so the Pi kernel driver can be
* tested against each one independently
*/

#include "uart_bridge.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

static const char *TAG = "uart_bridge";

QueueHandle_t uart_sensor_queue;

/* =====================================================
 *   CRC-8/SMBUS     (poly 0x07, init 0x00, no reflect) 
 *   Matches the kernel driver's crc8_smbus() exactly.
 * ===================================================== */
static uint8_t crc8_smbus(const uint8_t *data, size_t len)
{
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            crc = (crc & 0x80) ? ((crc << 1) ^ 0x07) : (crc << 1);
        }
    }
    return crc;
}
/* ============================================================
 * PROTOCOL 0 -- ASCII 
 * 
 * Wire format (fixed-width fields, always 30 bytes + '\n'):
 *      " T:+023.50,D:00145.3,S:000001\n" 
 *        |  |-----| |-------|  |----|
 *        |  temp    dist       seq
 *        |  (±XXX.XX °C)  (XXXXX.X cm)
 * 
 * All fields are zero-padded so the Pi can use sscanf without
 * worrying about variable-length tokens.
 * ============================================================*/
static int build_ascii(const uart_frame_t *s, uint8_t *out, size_t maxlen)
{
    return snprintf((char *)out, maxlen,
                    "T:%+07.2f,D:%07.1f,S:%06lu\n",
                    s->temp_c, s->dist_cm, s->seq);
}

/* =============================================================
 *  PROTOCOL 1 — BINARY
 *
 *  Compact 10-byte frame; layout is documented in uart_bridge.h.
 *  Fixed-point encoding avoids float-to-string overhead and
 *  lets the kernel driver parse without floating-point ops.
 * ============================================================= */
static int build_binary(const uart_frame_t *s, uint8_t *out, size_t maxlen)
{
    if (maxlen < BIN_FRAME_SIZE) return -1;

    bin_frame_t *f      = (bin_frame_t *)out;
    f->magic_h          = BIN_MAGIC_H;
    f->magic_l          = BIN_MAGIC_L;
    f->seq              = (uint8_t)(s->seq & 0xFF);
    f->type             = BIN_FRAME_TYPE;
    f->plen             = BIN_PAYLOAD_LEN;
    f->temp_x100        = (int16_t)roundf(s->temp_c  * 100.0f);
    f->dist_x10         = (uint16_t)roundf(s->dist_cm * 10.0f);

    /* CRC covers : seq, type, plen, temp_x100[0], temp_x100[1],
     *              dist_x10[0], dist_x10[1]    (7 bytes from offset 2) */
    f->crc = crc8_smbus(out + BIN_CRC_START, BIN_CRC_LEN);

    return (int)BIN_FRAME_SIZE;
}

/* ===========================================================
 * PROTOCOL 2 - JSON
 * 
 * Wire format: 
 *   {"seq":1, "t":23.50, "d":145.3}\n
 * 
 * Self-describing and easy to consume from any language.
 * largest praactical payload: ~40 bytes.
 * ===========================================================*/
static int build_json(const uart_frame_t *s, uint8_t *out, size_t maxlen)
{
    return snprintf((char *)out, maxlen,
                    "{\"seq\":%lu,\"t\":%.2f,\"d\":%.1f}\n", 
                     s->seq, s->temp_c, s->dist_cm);
}

/* ============================================================ 
 * UART hardware initialisation  
 * ============================================================ */
static void uart_hw_init(void)
{
    const uart_config_t cfg = {
        .baud_rate      = UART_BAUD,
        .data_bits      = UART_DATA_8_BITS,
        .parity         = UART_PARITY_DISABLE,
        .stop_bits      = UART_STOP_BITS_1,
        .flow_ctrl      = UART_HW_FLOWCTRL_DISABLE,
        .source_clk     = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, 
                                 UART_TX_PIN, UART_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, 
                                        UART_BUF_SIZE,  /*  RX ring buf */
                                        UART_BUF_SIZE,  /*  TX RING buf */
                                        0, NULL, 0));
    
    ESP_LOGI(TAG, "UART%d @ %d baud TX=GPIO%d    RX=GPIO%d", 
             UART_PORT, UART_BAUD,  UART_TX_PIN, UART_RX_PIN);                                                    
}

/* ==============================================================
 *  UART bridge FreeRTOS task
 *
 *  Blocks on uart_sensor_queue (max 2s watchdog), encodes
 *  the frame in the active protocol, and writes it to the UART.
 * ==============================================================*/
void uart_bridge_task(void *pvParameters)
{
    uart_frame_t frame;
    uint8_t      tx_buf[128];
    int          len;

    ESP_LOGI(TAG, "task started, protocol=%d", ACTIVE_PROTOCOL);

    for(;;) {
        if (xQueueReceive(uart_sensor_queue, &frame, 
                           pdMS_TO_TICKS(2000)) != pdTRUE) {
            ESP_LOGW(TAG, "sensor queue timeout - no data from sensors");
            continue;
        }

#if     ACTIVE_PROTOCOL == PROTO_ASCII
            len = build_ascii(&frame, tx_buf, sizeof(tx_buf));
#elif   ACTIVE_PROTOCOL == PROTO_BINARY
            len = build_binary(&frame, tx_buf, sizeof(tx_buf));
#elif   ACTIVE_PROTOCOL == PROTO_JSON
            len = build_json(&frame, tx_buf, sizeof(tx_buf));
#else
#error "ACTIVE_PROTOCOL must be PROTO_ASCII or PROTO_BINARY or PROTO_JSON"
#endif

        if (len > 0) {
            uart_write_bytes(UART_PORT, tx_buf, (size_t)len);
                ESP_LOGI(TAG, "TX: %d bytes sent", len);
#if ACTIVE_PROTOCOL != PROTO_BINARY
            /* ASCII / JSON: also log the line for debug */
            tx_buf[len - 1] = '\0';     /* strip trailing newline */
            ESP_LOGD(TAG, "TX: %s", (char *)tx_buf);
#else
            ESP_LOGD(TAG, "TX: %d bytes     seq=%u  t=%d  d=%u",
                     len, (unsigned)frame.seq, 
                     (int)roundf(frame.temp_c * 100),
                     (unsigned)roundf(frame.dist_cm * 10));
#endif
        }
    }
}

/* =============================================================
 *  Public init - call from app_main() before sensor tasks start
 * ============================================================= */
void uart_bridge_init(void)
{
    /* Depth-4 queue; overwrite policy handled by aggregator */
    uart_sensor_queue = xQueueCreate(4, sizeof(uart_frame_t));
    configASSERT(uart_sensor_queue != NULL);

    uart_hw_init();

    xTaskCreate(uart_bridge_task, "uart_bridge" ,
                4096,        /* stack words*/
                NULL,
                5,           /* priority - above sensors, below idle */
                NULL);

    ESP_LOGI(TAG, "bridge initialised protocol=%d", ACTIVE_PROTOCOL);
}
