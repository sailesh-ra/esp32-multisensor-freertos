// lcd_i2c.h
#pragma once
#include <stdint.h>

#define LCD_I2C_ADDR        0x27
#define LCD_SDA_GPIO        21
#define LCD_SCL_GPIO        22

/* LCD commands */
#define LCD_CLEAR           0x01
#define LCD_HOME            0x02
#define LCD_ENTRY_MODE      0x06
#define LCD_DISPLAY_ON      0x0C
#define LCD_FUNCTION_4BIT   0x28

void lcd_init(void);
void lcd_clear(void);
void lcd_set_cursor(uint8_t row, uint8_t col);
void lcd_print(const char *str);
void lcd_printf(uint8_t row, uint8_t col, const char *fmt, ...);