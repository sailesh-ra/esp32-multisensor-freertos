#pragma once
/*
* uart_bridge.h - ESP32 <-> Raspberry Pi UART sensor bridge
*
* Three protocol modes (select via ACTIVE_PROTOCOL):
*   PROTO_ASCII     "T:+023.50, D:00145.3, S:000001\n" human-readable
*   PROTO_BINARY    10-byte framed packet with CRC-8   compact + robust
*   PROTO_JSON      {"seq":1, "t":23.50, "d":145.3}\n  self-describing
*
* Hardware wiring (3.3 V logic - direct connection, no level shifter needed):
*   ESP32 GPIO17 (UART2 TX) ---> Pi GPIO15 / pin 10 (UART0 RXD)
*   ESP32 GPIO16 (UART2 RX) <--- Pi GPIO14 / pin 8  (UART0 TXD)
*   ESP32 GND               ---- Pi GND / pin 6
*/

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <stdint.h>

/* --- Active protocol - change this one line to switch modes --- */
#define ACTIVE_PROTOCOL     PROTO_BINARY

/* --- UART hardware --- */
#define UART_PORT           UART_NUM_2
#define UART_TX_PIN         17          /* -> Pi GPIO15 (RXD) */
#define UART_RX_PIN         16          /* <- Pi GPIO14 (TXD) */
#define UART_BAUD           115200
#define UART_BUF_SIZE       512

/* --- Binary frame constants --- */
#define BIN_MAGIC_H         0xAA
#define BIN_MAGIC_L         0x55
#define BIN_FRAME_TYPE      0X01        /* combined temp+dist frame */
#define BIN_PAYLOAD_LEN     4           /* int16 tempx100 + uint16 distx10 */
#define BIN_FRAME_SIZE      10          /* total bytes on the wire */
#define BIN_CRC_START       2           /* CRC covers bytes [2...8]*/
#define BIN_CRC_LEN         7

/* --- Protocol selector type --- */
#define PROTO_ASCII   0
#define PROTO_BINARY  1
#define PROTO_JSON    2


/*
* Binary frame layout (packed, 10 bytes):
* 
*   Byte    Field           Description
*   ----    -----           -----------
*      0    magic_h 0xAA    frame sync high
*      1    magic_l 0x55    frame sync low
*      2    seq     u8      sequence counter (wraps 0-255)
*      3    type    0x01    frame type: combined temp + dist
*      4    plen    0x04    payload length in bytes
*    5-6    temp    i16le   temperature x 100       (+2350 = +23.50 °C)
*    7-8    dist    u16le   distance x 10           ( 1453 =  145.3 cm)
*      9    crc     u8      CRC-8/SMBUS over bytes  [2..8]
*/

typedef struct __attribute__((packed)) {
    uint8_t     magic_h;
    uint8_t     magic_l;
    uint8_t     seq;
    uint8_t     type;
    uint8_t     plen;
    int16_t     temp_x100;  /* little-endian*/
    uint16_t    dist_x10;
    uint8_t     crc;
} bin_frame_t;

/*
* Aggregated sensor sample - pushed to uart_sensor_queue by sensor tasks.
* The UART bridge task drains this queue and serializes frames.
*/
typedef struct {
    float       temp_c;     /* DS18B20 temperature in °C */
    float       dist_cm;    /* HC-SR04 distance in cm    */
    uint32_t    seq;        /* monotonically increasing  */ 
} uart_frame_t;

/* Queue used to pass sensor samples to the UART bridge task. 
 * Declared here so DS18B20 / HC-SR04 aggregator can extern it.
 */
extern QueueHandle_t uart_sensor_queue;

/* Initialize UART hardware, create queue, and spawn bridge task. */
void uart_bridge_init(void);