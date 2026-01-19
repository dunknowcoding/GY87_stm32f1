#ifndef globals_h
#define globals_h

#include "utils.h"

#define IMU_ADDRESS 0x68

float chipTemperature;
bool isFirstData = true;

#ifdef GY85
    Adafruit_HMC5883_Unified mag; // Assign a unique ID to this sensor at the same time
    Adafruit_ADXL345_Unified accel; // Assign a unique ID to this sensor at the same time
    ITG3200 gyro;
#endif

#ifdef GY87
    IMU_HYBRID<MPU6050, QMC5883L> IMU(Wire);
    calData calib = {0};
    BMP085 barometer;
    float airTemperature;
    float pressure;
    float altitude;
#endif

#ifdef KALMAN
    Kalman kalmanX; // Create the Kalman instances
    Kalman kalmanY;
    Kalman kalmanZ;
    double gyroXangle, gyroYangle, gyroZangle; // Angle calculate using the gyro only
    double compAngleX, compAngleY, compAngleZ; // Calculated angle using a complementary filter
    double kalAngleX, kalAngleY, kalAngleZ; // Calculated angle using a Kalman filter
    double yaw; // Heading
    uint32_t timer;
#endif

#ifdef MADGWICK_AHRS
    Madgwick MadgwickAHRS;
    double roll, pitch, yaw;
#endif

#ifdef MAHONY_AHRS
    Mahony MahonyAHRS;
    float roll, pitch, yaw;
#endif

#endif