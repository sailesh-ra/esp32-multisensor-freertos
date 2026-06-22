//onewire.c

#include "onewire.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"

/* Pull bus LOW*/
static void bus_low(void)
{
    gpio_set_direction(ONEWIRE_GPIO, GPIO_MODE_OUTPUT); // where GPIO_MODE_OUTPUT ?
    gpio_set_level(ONEWIRE_GPIO, 0); 
}

/* Release bus - pull-up takes it HIGH*/
static void bus_release(void)
{
    gpio_set_direction(ONEWIRE_GPIO, GPIO_MODE_INPUT);
}

/* Read bus level*/
static bool bus_read(void)
{
    gpio_set_direction(ONEWIRE_GPIO, GPIO_MODE_INPUT);
    return gpio_get_level(ONEWIRE_GPIO);
}

/*
* Reset pulse - master pulls LOW for 480us
* then releases and waits for presence pulse
*/
bool onewire_reset(void)
{
    portDISABLE_INTERRUPTS();
    bus_low();
    esp_rom_delay_us(480);
    bus_release();
    esp_rom_delay_us(70);
    bool presence = !bus_read(); /* LOW = device present */
    esp_rom_delay_us(410);
    portENABLE_INTERRUPTS();
    return presence;
}

/*
* Write a single bit
* Write 1: pull LOW for 6us, release for 64us
* Write 0: pull LOW for 60us, release for 10us
*/

void onewire_write_bit(bool bit)
{
    portDISABLE_INTERRUPTS();
    if(bit) {
        bus_low();
        esp_rom_delay_us(6);
        bus_release();
        esp_rom_delay_us(64);
    } else {
        bus_low();
        esp_rom_delay_us(60);
        bus_release();
        esp_rom_delay_us(10);
    }
    portENABLE_INTERRUPTS();
}

/*
* Read a single bit
* Pull LOW for 6us, release, sample after 9us
*/
bool onewire_read_bit(void)
{
    portDISABLE_INTERRUPTS();
    bus_low();
    esp_rom_delay_us(6);
    bus_release();
    esp_rom_delay_us(9);
    bool bit = bus_read();
    esp_rom_delay_us(55);
    portENABLE_INTERRUPTS();
    return bit;
}

/* Read byte LSB first */
uint8_t onewire_read_byte(void)
{
    uint8_t byte = 0;
    for (int i = 0; i < 8; i++) {
        byte |= (onewire_read_bit() << i);
    }
    return byte;
}

/* Write byte LSB first */
void onewire_write_byte(uint8_t byte)
{
    for (int i = 0; i < 8; i++) {
        onewire_write_bit(byte & 0x01);
        byte >>= 1;
    }
}

/*
* CRC8 verification - Dallas/Maxim polynomial 0x31
* Used to verify scratchpad data integrity
*/
uint8_t onewire_crc8(uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        uint8_t byte = data[i];
        for (uint8_t j = 0; j < 8; j++) {
            uint8_t mix = (crc ^ byte) & 0x01;
            crc >>= 1;
            if(mix) crc ^= 0x8C;
            byte >>= 1;
        }
    }
    return crc;
}