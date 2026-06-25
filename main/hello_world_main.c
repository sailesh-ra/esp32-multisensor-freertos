#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "onewire.h"
#include "hcsr04.h"
#include "lcd_i2c.h"
#include "uart_bridge.h"

/* Sensor data structure passed via queue */
typedef struct {
    float temperature;
    float distance;
} sensor_data_t;

/* Queue handle */
static QueueHandle_t sensor_queue;

static float ds18b20_read_temperature(void)
{
    /* Step 1: Reset + presence check */
    if(!onewire_reset()) {
        printf("ERROR: No device found 1-wire bus!\n");
        return -999.0f;
    }

    /* Step 2: Skip ROM (only one device on bus)*/
    onewire_write_byte(CMD_SKIP_ROM);

    /* Step 3: Start temperature conversion*/
    onewire_write_byte(CMD_CONVERT_T);

    /* Step 4: Wait for conversion ( 750ms max)*/
    vTaskDelay(750 / portTICK_PERIOD_MS);

    /* Step 5: Reset again*/
    onewire_reset();
    onewire_write_byte(CMD_SKIP_ROM);

    /* Step 6: Read scratchpad */
    onewire_write_byte(CMD_READ_SCRATCHPAD);

    uint8_t scratchpad[9];
    for (int i = 0; i < 9; i++) {
        scratchpad[i] = onewire_read_byte();
    }

    /* Step 7: Verify CRC */
    if (onewire_crc8(scratchpad, 8) != scratchpad[8]) {
        printf("ERROR: CRC mismatch!\n");
        return -999.0f;
    }

    /* Step 8: Convert raw bytes to temperature */
    int16_t raw = (scratchpad[1] << 8) | scratchpad[0];
    return raw / 16.0f;
}

void sensor_task(void *pvParameter)
{
    static uint32_t seq = 0;
    /* Configure GPIO */
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << ONEWIRE_GPIO),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    hcsr04_init();

    printf("Custom 1-Wire DS18B20 Driver Starting...\n");

    while (1) {
        sensor_data_t data;
        data.temperature = ds18b20_read_temperature();
        data.distance    = hcsr04_read_distance();

        // float temp = ds18b20_read_temperature();
       /* if ( temp != -999.0f) {
            printf("Temperature: %.2f C\n", temp);
        } */ 
       /* Send to queue - don't block if full */
        xQueueSend(sensor_queue, &data, 0);

        uart_frame_t uart_data = {
            .temp_c     = data.temperature,
            .dist_cm    = data.distance,
            .seq        = seq++,    // bridge task doesn't use seq from here 
        };
        xQueueSend(uart_sensor_queue, &uart_data, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

/* Task 2 - Low priority: receives from queue and displays */
void display_task(void *pvParameter)
{

    sensor_data_t data;
    while(1) {
        /* Block until data arrives in queue */
        if(xQueueReceive(sensor_queue, &data, portMAX_DELAY)) {
            printf("+=========================+\n");
            printf("| Temp: %6.2f C           |\n", data.temperature);
           if(data.distance < 0) {
            printf("| Dist:  OUT OF RANGE     |\n");    
            } else {
            printf("| Dist: %6.2f cm          |\n", data.distance);
            }
            printf("+=========================+\n");

            /* LCD output */
            lcd_clear();
            lcd_printf(0,0, "T:%.2f C", data.temperature);
            if(data.distance < 0 ) {
                lcd_printf(1, 0, "D: OUT OF RANGE");
            } else {
                lcd_printf(1, 0, "D:%.2f cm", data.distance);
            }
        }
    }
}

void app_main(void)
{
    /* Create queue for 5 sensor readings */
    sensor_queue = xQueueCreate(5, sizeof(sensor_data_t));
    lcd_init();
    uart_bridge_init();
    lcd_clear();
    lcd_printf(0,0, "Sensor System");
    lcd_printf(1,0, "Starting...");

    printf("Multi-Sensor FreeRTOS System Starting...\n");

    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 3, NULL);
    xTaskCreate(display_task, "display_task", 2048, NULL, 1, NULL);
}