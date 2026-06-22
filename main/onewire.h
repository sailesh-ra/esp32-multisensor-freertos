#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"

/* GPIO pin for 1-wire bus*/
#define ONEWIRE_GPIO    GPIO_NUM_4

/* DS18B20 ROM commands*/
#define CMD_SKIP_ROM        0xCC
#define CMD_CONVERT_T       0x44
#define CMD_READ_SCRATCHPAD 0xBE

/* Function prototypes*/
bool    onewire_reset(void);
void    onewire_write_bit(bool bit);
bool    onewire_read_bit(void);
void    onewire_write_byte(uint8_t byte);
uint8_t onewire_read_byte(void);
uint8_t onewire_crc8(uint8_t *data, uint8_t len);

