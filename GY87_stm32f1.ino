
#include "src/globals.h"

void setup() {
    Serial.begin(115200);
    while(!Serial) Serial.println("Waiting for serial port...");
    #ifdef GY85
        mag = Adafruit_HMC5883_Unified(10001);
        accel = Adafruit_ADXL345_Unified(10002);
        /* accelerometer initialization*/
        if(!accel.begin()) {
            Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
            while(1);
        }
        Serial.println("Accelerometer ADXL345 found!");
        accel.setRange(ADXL345_RANGE_2_G); // ADXL345_RANGE_8_G, ADXL345_RANGE_4_G, ADXL345_RANGE_2_G
        if(!mag.begin()) {
            Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
            while(1);
        }
        Serial.println("Magnetometer HMC5883 found!");
        /* gyro initialization*/
        gyro.init();
        Serial.println("Gyroscope ITG3200 found!");
        Serial.print("Gyro performs zeroCalibrating...");
        gyro.zeroCalibrate(200, 10);//sample 200 times to calibrate and it will take 200*10ms
        Serial.println("Done!");
    #endif

    #ifdef GY87
        Wire.begin();
        Wire.setClock(400000); //400khz clock
        int err = IMU.init(calib, IMU_ADDRESS);
        if (err != 0) {
            Serial.print("Error initializing IMU: ");
            Serial.println(err);
            while (true) ;
        }
        #ifdef GY87_CALIB
            Serial.println("Calibrating GY-87 ...");
            delay(1000);
            Serial.println("Move IMU in figure 8 pattern until done.");
            delay(3000);
            IMU.calibrateMag(&calib);
            Serial.println("QMC5883L calibration done!");
            Serial.println("Keep IMU level.");
            delay(5000);
            IMU.calibrateAccelGyro(&calib);
            Serial.println("Calibration done!");
            Serial.println("Accel biases X/Y/Z: ");
            Serial.print(calib.accelBias[0]);
            Serial.print(", ");
            Serial.print(calib.accelBias[1]);
            Serial.print(", ");
            Serial.println(calib.accelBias[2]);
            Serial.println("Gyro biases X/Y/Z: ");
            Serial.print(calib.gyroBias[0]);
            Serial.print(", ");
            Serial.print(calib.gyroBias[1]);
            Serial.print(", ");
            Serial.println(calib.gyroBias[2]);
            Serial.println("Mag biases X/Y/Z: ");
            Serial.print(calib.magBias[0]);
            Serial.print(", ");
            Serial.print(calib.magBias[1]);
            Serial.print(", ");
            Serial.println(calib.magBias[2]);
            Serial.println("Mag Scale X/Y/Z: ");
            Serial.print(calib.magScale[0]);
            Serial.print(", ");
            Serial.print(calib.magScale[1]);
            Serial.print(", ");
            Serial.println(calib.magScale[2]);
            delay(5000);
            IMU.init(calib, IMU_ADDRESS);
        #endif
        err = IMU.setGyroRange(250);      //USE THESE TO SET THE RANGE, IF AN INVALID RANGE IS SET IT WILL RETURN -1
        err = IMU.setAccelRange(2);       //THESE TWO SET THE GYRO RANGE TO ±250 DPS AND THE ACCELEROMETER RANGE TO ±2g
        if (err != 0) {
            Serial.print("Error Setting range: ");
            Serial.println(err);
            while (true);
        }
        Serial.println("GY87 successfully initialized!");
        #if defined(DISP_ALTITUDE) || defined(DISP_AIR_TEMP) || defined(DISP_PRESSURE)
            barometer.init();
        #endif
    #endif
    delay(100);
}

void loop() {
    AccelRead accelRead;
    MagRead magRead;
    GyroRead gyroRead;

    #ifdef GY85
        readAccelMag(accelRead, magRead);
        readGyro(gyro, gyroRead);
        chipTemperature = gyro.getTemperature();
    #endif

    #ifdef GY87
        AccelData accelData;    //Sensor data
        GyroData gyroData;
        MagData magData;
        readIMU(accelData, magData, gyroData);
        accelDataCopy(accelData, accelRead);
        gyroDataCopy(gyroData, gyroRead);
        convertMagRawToUT(magData, magRead);
        chipTemperature = IMU.getTemp();
    #endif
    
    initFilter(accelRead, magRead);
    updateFilter(accelRead, gyroRead, magRead);
    delay(2);
}