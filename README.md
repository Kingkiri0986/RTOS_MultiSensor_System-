# RTOS Multisensor System

A real-time embedded system for integrated sensor data acquisition, processing, and fusion using FreeRTOS on ESP32/STM32 microcontrollers.

## 📋 Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Hardware Requirements](#hardware-requirements)
- [Software Requirements](#software-requirements)
- [System Architecture](#system-architecture)
- [Installation](#installation)
- [Configuration](#configuration)
- [Usage](#usage)
- [Task Description](#task-description)
- [Sensor Support](#sensor-support)
- [API Documentation](#api-documentation)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)
- [License](#license)

## 🎯 Overview

This project implements a multisensor data acquisition system built on FreeRTOS, enabling simultaneous operation of multiple sensors with real-time processing capabilities. The system uses task-based architecture for efficient sensor management, data fusion, and communication.

### Key Capabilities

- Concurrent multi-sensor operation without blocking
- Real-time data processing and sensor fusion
- Priority-based task scheduling
- Low-latency response to sensor events
- Modular and extensible design

## ✨ Features

- **Multi-Sensor Support**: Temperature, Humidity, Motion (PIR), Ultrasonic, Light (LDR), Gas sensors
- **Real-Time Processing**: FreeRTOS-based task management with configurable priorities
- **Sensor Fusion**: Combines multiple sensor inputs for enhanced decision-making
- **Data Logging**: Stores sensor data to SD card or cloud (optional)
- **Display Support**: Real-time visualization on OLED/LCD displays
- **Wireless Communication**: WiFi/Bluetooth support for remote monitoring
- **Low Power Modes**: Configurable sleep modes for battery-powered applications
- **Interrupt-Driven**: Efficient sensor reading using hardware interrupts where applicable

## 🔧 Hardware Requirements

### Microcontroller Platforms

- **ESP32** (recommended) - Built-in FreeRTOS, WiFi, Bluetooth
- **STM32** (F4/F7/H7 series) - ARM Cortex-M with FreeRTOS
- **Arduino Mega 2560** with FreeRTOS library

### Supported Sensors

| Sensor Type | Model | Interface | Purpose |
|-------------|-------|-----------|---------|
| Temperature/Humidity | DHT11/DHT22 | Digital | Environmental monitoring |
| Temperature | DS18B20 | One-Wire | Precision temperature |
| Motion | PIR HC-SR501 | Digital | Motion detection |
| Distance | HC-SR04 | Digital | Ultrasonic ranging |
| Light | LDR/BH1750 | Analog/I2C | Ambient light sensing |
| Gas | MQ-2/MQ-135 | Analog | Air quality monitoring |
| Pressure | BMP180/BME280 | I2C | Barometric pressure |

### Additional Components

- OLED Display (128x64, I2C) - Optional
- SD Card Module - Optional for data logging
- Power Supply (5V/3.3V depending on sensors)
- Breadboard and jumper wires

## 💻 Software Requirements

- **Arduino IDE** (1.8.19 or later) or **PlatformIO**
- **ESP32 Board Package** (for ESP32) or **STM32duino** (for STM32)
- **FreeRTOS** (built-in for ESP32, library for Arduino)

### Required Libraries

```
- Wire.h (I2C communication)
- SPI.h (SPI communication)
- DHT.h (DHT sensor library)
- Adafruit_SSD1306.h (OLED display)
- OneWire.h, DallasTemperature.h (DS18B20)
- WiFi.h or ESP8266WiFi.h (for wireless features)
```

Install via Arduino Library Manager or PlatformIO.

## 🏗️ System Architecture

### Task Structure

```
┌─────────────────────────────────────────┐
│         RTOS Scheduler (FreeRTOS)       │
└─────────────────────────────────────────┘
         │         │         │         │
    ┌────▼───┐ ┌──▼────┐ ┌──▼────┐ ┌──▼────┐
    │Sensor  │ │Data   │ │Fusion │ │Display│
    │Tasks   │ │Process│ │Task   │ │Task   │
    │(High)  │ │(Med)  │ │(Med)  │ │(Low)  │
    └────┬───┘ └──┬────┘ └──┬────┘ └──┬────┘
         │         │         │         │
         └─────────┴─────────┴─────────┘
                    │
            ┌───────▼────────┐
            │  Data Queues   │
            │  & Semaphores  │
            └────────────────┘
```

### Data Flow

1. **Sensor Tasks** read raw data at defined intervals
2. **Processing Tasks** filter and validate sensor readings
3. **Fusion Task** combines multiple sensor inputs
4. **Display Task** updates UI and sends data to cloud
5. **Management Task** handles configuration and errors

## 📥 Installation

### Step 1: Clone the Repository

```bash
git clone https://github.com/yourusername/rtos_multisensor_system.git
cd rtos_multisensor_system
```

### Step 2: Install Dependencies

**Arduino IDE:**
- Open Arduino IDE
- Go to Sketch → Include Library → Manage Libraries
- Search and install required libraries

**PlatformIO:**
```bash
pio lib install
```

### Step 3: Hardware Setup

1. Connect sensors according to the wiring diagram in `/docs/wiring_diagram.png`
2. Connect ESP32/STM32 to your computer via USB
3. Verify all connections match the pin definitions in `config.h`

### Step 4: Upload Code

**Arduino IDE:**
```
1. Open rtos_multisensor_system.ino
2. Select Board and Port from Tools menu
3. Click Upload
```

**PlatformIO:**
```bash
pio run --target upload
```

## ⚙️ Configuration

Edit `config.h` to customize your setup:

```cpp
// Sensor Pin Configuration
#define DHT_PIN 4
#define PIR_PIN 5
#define TRIG_PIN 12
#define ECHO_PIN 13
#define LDR_PIN 34

// Task Priorities
#define SENSOR_TASK_PRIORITY 3
#define PROCESS_TASK_PRIORITY 2
#define DISPLAY_TASK_PRIORITY 1

// Sampling Rates (milliseconds)
#define DHT_SAMPLE_RATE 2000
#define PIR_SAMPLE_RATE 500
#define ULTRASONIC_SAMPLE_RATE 1000

// WiFi Configuration (Optional)
#define WIFI_SSID "YourSSID"
#define WIFI_PASSWORD "YourPassword"
```

## 🚀 Usage

### Basic Operation

1. Power on the system
2. Sensors will initialize automatically
3. Data will be displayed on serial monitor (115200 baud) or OLED
4. Monitor real-time readings and system status

### Serial Monitor Output

```
[INIT] System Starting...
[INIT] FreeRTOS Kernel Started
[SENSOR] DHT22 Initialized
[SENSOR] PIR Sensor Ready
[SENSOR] Ultrasonic Ready
[SYSTEM] All sensors operational

[DATA] Temp: 24.5°C | Humidity: 65% | Motion: No | Distance: 45cm
```

### WiFi/Cloud Integration

Enable WiFi in `config.h` to send data to cloud platforms:
- ThingSpeak
- Blynk
- MQTT Broker
- Custom REST API

## 📝 Task Description

### Sensor Sampling Tasks

- **Priority**: High (3)
- **Stack Size**: 2048 bytes
- **Period**: Sensor-dependent (500ms - 5000ms)
- **Function**: Read raw sensor data and push to processing queue

### Data Processing Tasks

- **Priority**: Medium (2)
- **Stack Size**: 4096 bytes
- **Function**: Filter noise, validate ranges, calculate moving averages

### Fusion Task

- **Priority**: Medium (2)
- **Stack Size**: 4096 bytes
- **Function**: Combine sensor data, detect patterns, trigger alerts

### Display/Communication Task

- **Priority**: Low (1)
- **Stack Size**: 3072 bytes
- **Function**: Update display, send data via WiFi/Serial

## 🔌 Sensor Support

### Adding New Sensors

1. Create sensor initialization function in `sensors.cpp`
2. Implement sensor reading task
3. Add configuration to `config.h`
4. Register task in `main.cpp`

Example:
```cpp
void sensorTask(void *parameter) {
    while(1) {
        // Read sensor
        float value = readSensor();
        
        // Send to queue
        xQueueSend(sensorQueue, &value, portMAX_DELAY);
        
        // Task delay
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
```

## 📚 API Documentation

### Core Functions

#### `initSensors()`
Initializes all configured sensors.

#### `createTasks()`
Creates and starts all FreeRTOS tasks.

#### `getSensorData(sensor_type_t type)`
Returns the latest reading from specified sensor.

#### `fuseSensorData()`
Combines multiple sensor readings for decision-making.

For detailed API documentation, see `/docs/API.md`

## 🐛 Troubleshooting

### Common Issues

**Sensor not detected:**
- Verify wiring and pin configuration
- Check sensor power supply voltage
- Test sensor with standalone sketch first

**Task stack overflow:**
- Increase stack size in task creation
- Monitor stack usage with `uxTaskGetStackHighWaterMark()`

**WiFi connection fails:**
- Verify SSID and password in config.h
- Check WiFi signal strength
- Ensure ESP32 has adequate power supply

**System crashes/reboots:**
- Check for memory leaks
- Verify interrupt handlers are ISR-safe
- Monitor heap fragmentation

## 🤝 Contributing

Contributions are welcome! Please follow these steps:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/NewSensor`)
3. Commit your changes (`git commit -m 'Add BMP280 sensor support'`)
4. Push to the branch (`git push origin feature/NewSensor`)
5. Open a Pull Request

Please ensure code follows the existing style and includes appropriate comments.

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 👥 Authors

- Your Name - Initial work - [@Kingkiri0986](https://github.com/Kingkiri0986)

## 🙏 Acknowledgments

- FreeRTOS community for excellent RTOS documentation
- ESP32 Arduino core developers
- Sensor library maintainers

## 📞 Support

For questions or issues:
- Open an issue on GitHub
- Email: paramtap0809@gmail.com
- Documentation: [Wiki](https://github.com/Kingkiri0986/rtos_multisensor_system/wiki)

---

**⭐ If you find this project helpful, please give it a star!**