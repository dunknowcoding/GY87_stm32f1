# Stm32duino GY-87/85 Sensor Fusion

---
Main source code (exclude libs) for testing sensor fusion algorithms with GY-85 9-Axis IMU Sensor and GY-87 10-Axis IMU Sensor. This repo is specially made for YouTube channel **牛志伟机器人实验室** @[NiusRobotLab](https://www.youtube.com/@NiusRobotLab)

![Sensor Fusion](./assets/sensors.png)

---

## GY-85

One of the classic 9-Axis sensors, based on I2C protocol only. This sensor is favored for testing sensor fusion algorithms (At least I did 15 years ago). This module consists of 3 sensors:
Accelerometer - ADXL345, Magnetometer - HMC5883 (Some using QMC5883), Gyroscope - ITG3200.

## GY-87

Fine update of GY-85:
Accelerometer - MPU6050, Magnetometer - QMC5883L, Gyroscope - MPU6050.
An additional Barometer - BMP085 (10th DOF), this repo also reads temperature, altitude and air pressure from this sensor

## Sensor Fusion Algorithms

- Kalman Filter and Complementary Filter
- Madgwick AHRS Filter (6DOF fusion)
- Mahony AHRS Filter (6DOF fusion)

## Library Dependencies

These libraries require mannual installation inside the Arduino IDE.

1. Kalman Filter Library
2. Grove 3-Axis Digital Gyro
3. Adafruit ADXL345
4. Adafruit HMC5883 Unified
5. FastIMU
6. Grove Barometer Sensor
7. Madgwick
8. Mahony

## Check This Video for Details

The sensor has been tested with 
- STM32F103C8T6

:point_down:

[![BNO055 Test](https://img.youtube.com/vi/f0TQxYB9auw/0.jpg)](https://www.youtube.com/shorts/f0TQxYB9auw)

## TODO

- Heading is inaccurate for GY-85/GY-87 after 6DOF fusion, Madgwick and Mahony filters.
- Slow convergence, and divergence for Mahony filters (some issue of the library).
- Incorrect heading caculation for Madgwick and Mahony, probably due to magnetometer range.
- Quaternion is not implemented, Gimbal lock will be obeserved.
- Test on other MCUs including Arduino UNO, ESP32, RP Pi Pico, etc.
---
2026 MIT License
Happy Chinese New Year