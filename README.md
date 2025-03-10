# PicoW Garden Monitor

This project is a **Garden Monitoring System** developed using **Rust** and the **Raspberry Pi Pico W**. It monitors environmental parameters like **temperature**, **humidity**, and **soil moisture**, and controls a **water pump** to maintain optimal conditions for plant care.

## Features

- **Temperature Monitoring**: Alerts if the temperature exceeds or falls below predefined thresholds.
- **Humidity Monitoring**: Alerts if humidity is outside the desired range.
- **Soil Moisture Monitoring**: Activates the water pump when moisture levels are low.
- **Wi-Fi Connectivity**: Remote monitoring via Wi-Fi.
- **LED Indicators**: 
  - Green LED: System is functioning properly.
  - Red LED: Alerts for out-of-range parameters.
- **Buzzer**: Audible alert when an issue is detected.

## Requirements

- **Hardware**:
  - Raspberry Pi Pico W
  - DHT22 Sensor (Temperature and Humidity)
  - Soil Moisture Sensor
  - Water Pump
  - LEDs and Buzzer

- **Software**:
  - Rust
  - `embedded-hal` (hardware abstraction)
  - `embassy` (asynchronous programming)
  - `wifi` crate (Wi-Fi connectivity)
  - `adc` (analog sensor reading)

## Usage

Once powered on, the system will monitor environmental conditions. Alerts will trigger if parameters are out of range, and the water pump will activate when soil moisture is too low.

## Future Improvements

- Add a web interface for remote monitoring.
- Implement cloud-based data logging.
- Expand with additional sensors (e.g., light level, CO2).
![WhatsApp Image 2025-02-19 at 22 40 11_b4438324](https://github.com/user-attachments/assets/8419b835-edf1-481b-ac52-26ac22e1edf9)
![WhatsApp Image 2025-02-19 at 22 40 11_9ada9cd8](https://github.com/user-attachments/assets/c19a465e-daab-4e76-b059-fe39bc37b277)

