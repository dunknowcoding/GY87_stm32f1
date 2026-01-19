#ifndef utils_h
#define utils_h

#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_HMC5883_U.h>
#include <ITG3200.h>
#include <Kalman.h>
#include <MadgwickAHRS.h>
#include <MahonyAHRS.h>
#include <FastIMU.h>
#include <BMP085.h>
#include "../config.h"

#define QMC5883L_RANGE_2G    0.12207  // µT per LSB for ±2 Gauss range
#define QMC5883L_RANGE_8G    0.04883  // µT per LSB for ±8 Gauss range (default)

extern float chipTemperature;
extern bool isFirstData;

#ifdef GY85
    extern Adafruit_HMC5883_Unified mag;
    extern Adafruit_ADXL345_Unified accel;
    extern ITG3200 gyro;
#endif

#ifdef GY87
    extern IMU_HYBRID<MPU6050, QMC5883L> IMU;
    extern calData calib;

    #if defined(DISP_ALTITUDE) || defined(DISP_AIR_TEMP) || defined(DISP_PRESSURE)
        extern BMP085 barometer;
        extern float airTemperature;
        extern float pressure;
        extern float altitude;
    #endif

#endif

#ifdef KALMAN
  extern Kalman kalmanX; // Create the Kalman instances
  extern Kalman kalmanY;
  extern Kalman kalmanZ;
  extern double gyroXangle, gyroYangle, gyroZangle; // Angle calculate using the gyro only
  extern double compAngleX, compAngleY, compAngleZ; // Calculated angle using a complementary filter
  extern double kalAngleX, kalAngleY, kalAngleZ; // Calculated angle using a Kalman filter
  extern double yaw; // Heading
  extern uint32_t timer;
#endif

#ifdef MADGWICK_AHRS
    extern Madgwick MadgwickAHRS;
    extern double roll, pitch, yaw;
#endif

#ifdef MAHONY_AHRS
    extern Mahony MahonyAHRS;
    extern float roll, pitch, yaw;
#endif

struct AccelRead {
    double x;
    double y;
    double z;
};

struct GyroRead {
    float x;
    float y;
    float z;
};

struct MagRead {
    double x;
    double y;
    double z;
};

#ifdef GY85
    void readAccelMag(AccelRead&, MagRead&);
    void readGyro(ITG3200&, GyroRead&);
#endif

#ifdef GY87
    void readIMU(AccelData&, MagData&, GyroData&);
#endif

void convertMagRawToUT(MagData&, MagRead&);
void accelDataCopy(AccelData&, AccelRead&);
void gyroDataCopy(GyroData&, GyroRead&);
void magDataCopy(MagData&, MagRead&);
void initFilter(AccelRead&, MagRead&);
void updateFilter(AccelRead&, GyroRead&, MagRead&);
void updateYaw(MagRead&, double, double, double*);

#endif