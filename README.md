# ESP32 Multi-Sensor System with Custom Drivers

A FreeRTOS-based data acquisition system on ESP32 with custom-built device drivers (no library abstractions).

## Features

- **Custom 1-Wire protocol driver** for DS18B20 temperature sensor with microsecond-precision bit-banging and CRC8 verification
- **Custom HC-SR04 ultrasonic driver** with timeout handling
- **Custom I2C LCD driver** for PCF8574T-based 16x2 display
- **FreeRTOS multi-task architecture** with priority-based scheduling and inter-task queue communication

## Hardware

- ESP32-WROOM-32 development board
- DS18B20 temperature sensor (1-Wire)
- HC-SR04 ultrasonic distance sensor
- 16x2 LCD with PCF8574T I2C adapter

## Pin Configuration

| Component | GPIO |
|-----------|------|
| DS18B20 DAT | GPIO4 |
| HC-SR04 TRIG | GPIO5 |
| HC-SR04 ECHO | GPIO18 |
| LCD SDA | GPIO21 |
| LCD SCL | GPIO22 |

## Architecture

\`\`\`
sensor_task (priority 3) ──┐
                            ├──→ FreeRTOS queue ──→ display_task (priority 1)
                            │                       │
                            │                       ├── Serial output
                            │                       └── LCD output
\`\`\`

## Key Technical Decisions

- **Bit-banged 1-Wire** instead of library: full timing control, interrupt-disabled critical sections
- **Hardware I2C peripheral**: no interrupt protection needed (self-clocking protocol)
- **FreeRTOS queues**: decouple producer (sensors) from consumer (display)

## Build

\`\`\`bash
idf.py set-target esp32
idf.py build flash monitor
\`\`\`

## Built With

- ESP-IDF v6.0.1
- FreeRTOS (built into ESP-IDF)