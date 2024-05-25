# LightRanger8 Click Sensor Integration with STM32H7 and ESP32
## Project Overview
This project demonstrates the integration of the LightRanger8 Click sensor with two different microcontroller platforms: STM32H7 and ESP32.
For the ESP32 [this](https://github.com/stm32duino/VL53L3CX) library is used and
for STM32H7 we used parts of [this](https://github.com/MikroElektronika/mikrosdk_click_v2/tree/master/clicks/lightranger8) library, we didn't use everything from that library and
we started with [this](https://github.com/LAPSyLAB/STM32H7_Discovery_VIN_Projects/tree/main/STM32H750B-DK_I2C_Basic_Demo) project.

## Repository Structure
- STM32H7/: Contains the project files and code for integrating the LightRanger8 sensor with the STM32H7 microcontroller.
- ESP32/: Contains the project files and code for integrating the LightRanger8 sensor with the ESP32 development board.

## Prerequisites
- STM32H7 development board (e.g., STM32H750B-DK)
- ESP32 development board
- LightRanger8 Click sensor
- Arduino IDE (for ESP32)
- STM32CubeIDE (for STM32H7)
- Breadboard and jumper wires

## Challenges and Solutions
Arduino Library Complexity: The Arduino library for the LightRanger8 sensor is extensive and lacks comprehensive documentation, making it challenging to understand its inner workings.
We mitigated this by using a more concise and understandable library from MikroElektronika for the STM32H7.
Documentation Gaps: There was a lack of detailed documentation for the sensor, particularly regarding register mappings and detailed functionality.
We relied on experimentation and existing code examples to fill these gaps.

## Results
- Arduino library on ESP32: Achieved precise measurements and multi-object detection, demonstrating the sensor's high reliability under various lighting conditions.
- STM32H7: Enabled flexible configuration and optimization of sensor settings, though with slightly less precision compared to the ESP32 implementation.

## Demonstration Videos
[Live visualization of measurements](https://www.youtube.com/watch?v=adKKxkB6kxs)
