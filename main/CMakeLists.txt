idf_component_register(SRCS "hello_world_main.c" "onewire.c" "hcsr04.c" "lcd_i2c.c"
                        INCLUDE_DIRS "."
                        REQUIRES esp_driver_i2c esp_driver_gpio)