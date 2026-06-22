#include "hcsr04.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"

void hcsr04_init(void)
{
    /* TRIG pin - output */
    gpio_config_t trig = {
        .pin_bit_mask = (1ULL << TRIG_GPIO),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };
    gpio_config(&trig);

    /* ECHO pin - input */
    gpio_config_t echo = {
        .pin_bit_mask = (1ULL << ECHO_GPIO),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };
    gpio_config(&echo);

    gpio_set_level(TRIG_GPIO, 0);
}

float hcsr04_read_distance(void)
{
    /* Send 10us trigger pulse */
    portDISABLE_INTERRUPTS();
    gpio_set_level(TRIG_GPIO, 0);
    esp_rom_delay_us(2);
    gpio_set_level(TRIG_GPIO, 1);
    esp_rom_delay_us(10);
    gpio_set_level(TRIG_GPIO, 0);
    portENABLE_INTERRUPTS();

    /* Wait for ECHO to go HIGH with timeout*/
    uint32_t timeout = 0;
    while (gpio_get_level(ECHO_GPIO) == 0) {
        esp_rom_delay_us(1);
        if (++timeout > 30000) return -1.0f;    }

    /* Measure pulse width */
    uint32_t start = 0;
    uint32_t count = 0;
    while (gpio_get_level(ECHO_GPIO) == 1) {
        esp_rom_delay_us(1);
        count++;
        if (count > 30000) return -1.0f;
    }

    /* Convert to cm: sound travels 343m/s = 0.0343 cm/us */
    /* Divide by 2 for round trip */
    return (count * 0.0343f) / 2.0f;
}