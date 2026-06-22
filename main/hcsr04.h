#pragma once
#include <stdint.h>
#include "driver/gpio.h"

#define TRIG_GPIO GPIO_NUM_5
#define ECHO_GPIO GPIO_NUM_18

void    hcsr04_init(void);
float   hcsr04_read_distance(void); 