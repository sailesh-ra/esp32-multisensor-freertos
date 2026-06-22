// lcd_i2c.c

#include "lcd_i2c.h"
#include "driver/i2c_master.h"
#include "esp_rom_sys.h"
#include <stdarg.h>
#include <stdio.h>
#include "freertos/portmacro.h"

/* PCF8574 pin maapping to LCD */
#define PIN_RS  0x01    /* Register select */
#define PIN_RW  0x02    /* Read/Write      */
#define PIN_EN  0x04    /* Enable          */
#define PIN_BL  0x08    /* Backlight       */

static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t dev_handle;

/* Send single byte to PC8574 over I2C */
static void i2c_write_byte(uint8_t data)
{
    i2c_master_transmit(dev_handle, &data, 1, 100);
}

/* Send 4-bit nibble to LCD */
static void lcd_send_nibble(uint8_t nibble)
{
    uint8_t data = (nibble & 0x0F) << 4;
    i2c_write_byte(data | PIN_EN | PIN_BL);
    esp_rom_delay_us(1);
    i2c_write_byte((data & ~PIN_EN) | PIN_BL);
    esp_rom_delay_us(50);
}

/* Send full byte as two nibbles */
static void lcd_send_byte(uint8_t byte, uint8_t mode)
{
    uint8_t high = (byte & 0xF0) | mode | PIN_BL;
    uint8_t low  = ((byte << 4) & 0xF0) | mode | PIN_BL;
    i2c_write_byte(high | PIN_EN);
    esp_rom_delay_us(1);
    i2c_write_byte(high & ~PIN_EN);
    esp_rom_delay_us(50);
    i2c_write_byte(low | PIN_EN);
    esp_rom_delay_us(1);
    i2c_write_byte(low & ~PIN_EN);
    esp_rom_delay_us(50); 
}

/* Send command to LCD */
static void lcd_command(uint8_t cmd)
{
    lcd_send_byte(cmd, 0x00);
}

/* Send data character to LCD */
static void lcd_data(uint8_t data)
{
    lcd_send_byte(data, PIN_RS);
}

void lcd_init(void)
{
    /* Configure I2C bus */
    i2c_master_bus_config_t bus_config = {
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .i2c_port          = I2C_NUM_0,
        .sda_io_num        = LCD_SDA_GPIO,
        .scl_io_num        = LCD_SCL_GPIO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true
    };
    i2c_new_master_bus(&bus_config, &bus_handle);

    /* Add LCD device */
    i2c_device_config_t dev_config = {
        .dev_addr_length    = I2C_ADDR_BIT_LEN_7,
        .device_address     = LCD_I2C_ADDR,
        .scl_speed_hz       = 100000
    };
    i2c_master_bus_add_device(bus_handle, &dev_config, &dev_handle);

    /* LCD initialization sequence */
    esp_rom_delay_us(50000); /* Wait for LCD power up */

    /* Switch to a 4-bit mode */
    lcd_send_nibble(0x03);
    esp_rom_delay_us(4500);
    lcd_send_nibble(0x03);
    esp_rom_delay_us(4500);
    lcd_send_nibble(0x03);
    esp_rom_delay_us(150);
    lcd_send_nibble(0x02); /* Set 4-bit mode */

    /* Configure LCD */
    lcd_command(LCD_FUNCTION_4BIT); /* 4-bit, 2 lines, 5x8 font */
    lcd_command(LCD_DISPLAY_ON);    /* Display on, cursor off   */
    lcd_command(LCD_CLEAR);         /* Clear Display            */
    esp_rom_delay_us(2000);
    lcd_command(LCD_ENTRY_MODE);    /* Entry mode set           */

    /* Turn on backlight */
    i2c_write_byte(PIN_BL);
}

void lcd_clear(void)
{
    lcd_command(LCD_CLEAR);
    esp_rom_delay_us(2000);
}

void lcd_set_cursor(uint8_t row, uint8_t col)
{
    uint8_t row_offsets[] = {0x00, 0x40};
    lcd_command(0x80 | (col + row_offsets[row]));
}

void lcd_print(const char *str)
{
    while(*str) {
        lcd_data(*str++);
    }
}

void lcd_printf(uint8_t row, uint8_t col, const char *fmt, ...)
{
    char buf[17]; /* Max 16 chars + null */
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    lcd_set_cursor(row,col);
    lcd_print(buf);
}