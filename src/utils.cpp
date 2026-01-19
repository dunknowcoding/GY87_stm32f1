#include "utils.h"

#ifdef GY85
    void readAccelMag(AccelRead& accelEvent, MagRead& magEvent) {
        sensors_event_t accelRaw, magRaw;
        accel.getEvent(&accelRaw);
        mag.getEvent(&magRaw);
        
        accelEvent.x = accelRaw.acceleration.x;
        accelEvent.y = accelRaw.acceleration.y;
        accelEvent.z = accelRaw.acceleration.z;
        
        magEvent.x = magRaw.magnetic.x;
        magEvent.y = magRaw.magnetic.y;
        magEvent.z = magRaw.magnetic.z;
    }

    void readGyro(ITG3200& gyroscope, GyroRead& gyroEvent) {
        gyroscope.getAngularVelocity(&gyroEvent.x, &gyroEvent.y, &gyroEvent.z);
    }
#endif

#ifdef GY87
    void accelDataCopy(AccelData& data, AccelRead& event) {
        event.x = static_cast<double>(data.accelX);
        event.y = static_cast<double>(data.accelY);
        event.z = static_cast<double>(data.accelZ);
    }

    void gyroDataCopy(GyroData& data, GyroRead& event) {
        event.x = data.gyroX;
        event.y = data.gyroY;
        event.z = data.gyroZ;
}
    void convertMagRawToUT(MagData& rawData, MagRead& magUT) {
        magUT.x = rawData.magX * QMC5883L_SENSITIVITY;
        magUT.y = rawData.magY * QMC5883L_SENSITIVITY;
        magUT.z = rawData.magZ * QMC5883L_SENSITIVITY;
    }

    void readIMU(AccelData& accelEvent, MagData& magEvent, GyroData& gyroEvent) {
        IMU.update();
        IMU.getMag(&magEvent);
        while (!magEvent.magX && !magEvent.magY && !magEvent.magZ) { // wait for valid magnetometer values
            IMU.update();
            IMU.getMag(&magEvent);
        }
        IMU.getAccel(&accelEvent);
        IMU.getGyro(&gyroEvent);
    }

    void getAirTemp() {
        airTemperature = barometer.bmp085GetTemperature(barometer.bmp085ReadUT()); //bmp085ReadUT MUST be called first
    }

    #ifdef DISP_ALTITUDE
        void getAltitude() {
            altitude = barometer.calcAltitude(DISP_ALTITUDE);
        }
    #endif

    void getPressure() {
        pressure = barometer.bmp085GetPressure(barometer.bmp085ReadUP());
    }

#endif

#ifdef KALMAN
    void updateYaw(MagRead& magRead, double kalAngleX, double kalAngleY, double *yaw) {
        #ifndef GY87_CALIB
            double _magX = magRead.x * -1.0; // Invert axis - this it done here, as it should be done after the calibration
            double _magY = magRead.y * -1.0;
            double _magZ = magRead.z * -1.0;
        #else
            double _magX = magRead.x; // after GY-85 calibration
            double _magY = magRead.y;
            double _magZ = magRead.z;
        #endif
        double rollAngle = kalAngleX * DEG_TO_RAD;
        double pitchAngle = kalAngleY * DEG_TO_RAD;
        //double Bfy = _magZ * sin(rollAngle) - _magY * cos(rollAngle);
        double Bfy = _magY * cos(rollAngle) - _magZ * sin(rollAngle);
        double Bfx = _magX * cos(pitchAngle) + _magY * sin(pitchAngle) * sin(rollAngle) + _magZ * sin(pitchAngle) * cos(rollAngle);
        *yaw = atan2(-Bfy, Bfx) * RAD_TO_DEG;
        //yaw *= -1;
    }

    void initFilter(AccelRead& accelRead, MagRead& magRead) {
        if (isFirstData) isFirstData = false; else return;
        #ifdef RESTRICT_PITCH // Eq. 25 and 26
            double roll  = atan2(accelRead.y, accelRead.z) * RAD_TO_DEG;
            double pitch = atan(-accelRead.x / sqrt(accelRead.y * accelRead.y + accelRead.z * accelRead.z)) * RAD_TO_DEG;
        #else // Eq. 28 and 29
            double roll  = atan(accelRead.y / sqrt(accelRead.x * accelRead.x + accelRead.z * accelRead.z)) * RAD_TO_DEG;
            double pitch = atan2(-accelRead.x, accelRead.z) * RAD_TO_DEG;
        #endif
        updateYaw(magRead, kalAngleX, kalAngleY, &yaw);
        kalmanX.setAngle(roll); // Set starting angle
        kalmanY.setAngle(pitch);
        kalmanZ.setAngle(yaw);
        gyroXangle = compAngleX = roll;
        gyroYangle = compAngleY = pitch;
        gyroZangle = compAngleZ = yaw;
        timer = micros();
    }

    void updateFilter(AccelRead& accelRead, GyroRead& gyroRead, MagRead& magRead) {
        if (isFirstData) return;
        double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
        timer = micros();
        // #define RESTRICT_PITCH
        #ifdef GYRO_RAD_TO_DEG
            double gyroXrate = gyroRead.x * DEG_TO_RAD; // Convert to deg/s, for rad/s
            double gyroYrate = gyroRead.y * DEG_TO_RAD;
            double gyroZrate = gyroRead.z * DEG_TO_RAD;
        #else
            double gyroXrate = gyroRead.x; // deg/s
            double gyroYrate = gyroRead.y;
            double gyroZrate = gyroRead.z;
        #endif
        /* Yaw estimation */
        updateYaw(magRead, kalAngleX, kalAngleY, &yaw);
        // This fixes the transition problem when the yaw angle jumps between -180 and 180 degrees
        if ((yaw < -90 && kalAngleZ > 90) || (yaw > 90 && kalAngleZ < -90)) {
            kalmanZ.setAngle(yaw);
            kalAngleZ = yaw;
            } 
        else 
            kalAngleZ = kalmanZ.getAngle(yaw, gyroZrate, dt); // Calculate the angle using a Kalman filter
        /* Roll and Pitch estimation */
        #ifdef RESTRICT_PITCH // Eq. 25 and 26
            double roll  = atan2(accelRead.y, accelRead.z) * RAD_TO_DEG;
            double pitch = atan(-accelRead.x / sqrt(accelRead.y * accelRead.y + accelRead.z * accelRead.z)) * RAD_TO_DEG;
        #else // Eq. 28 and 29
            double roll  = atan(accelRead.y / sqrt(accelRead.x * accelRead.x + accelRead.z * accelRead.z)) * RAD_TO_DEG;
            double pitch = atan2(-accelRead.x, accelRead.z) * RAD_TO_DEG;
        #endif
        // #define RESTRICT_PITCH
        #ifdef RESTRICT_PITCH
            // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
            if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
                kalmanX.setAngle(roll);
                compAngleX = roll;
                kalAngleX = roll;
                gyroXangle = roll;
                }
            else
                kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
            if (abs(kalAngleX) > 90) gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
            kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
        #else
            // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
            if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
                kalmanY.setAngle(pitch);
                compAngleY = pitch;
                kalAngleY = pitch;
                gyroYangle = pitch;
                }
            else
                kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter
            if (abs(kalAngleY) > 90) gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
            kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
        #endif
        // #define UNBIASED_GYRO_RATE
        #ifdef UNBIASED_GYRO_RATE
            gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
            gyroYangle += kalmanY.getRate() * dt;
            gyroZangle += kalmanZ.getRate() * dt;
        #else
            gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
            gyroYangle += gyroYrate * dt;
            gyroZangle += gyroZrate * dt;
        #endif
        // Calculate the angle using a Complimentary filter
        compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; 
        compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;
        compAngleZ = 0.93 * (compAngleZ + gyroZrate * dt) + 0.07 * yaw;
        // Reset the gyro angle when it has drifted too much
        if (gyroXangle < -180 || gyroXangle > 180) gyroXangle = kalAngleX;
        if (gyroYangle < -180 || gyroYangle > 180) gyroYangle = kalAngleY;
        if (gyroZangle < -180 || gyroZangle > 180) gyroZangle = kalAngleZ;
        /* Print Results */
        // #define DISP_RAW_ACC
        #ifdef DISP_RAW_ACC // Accelerometer raw data
            Serial.print("Accel-x:"); Serial.print(accelRead.x); Serial.print("\t");
            Serial.print("Accel-y:"); Serial.print(accelRead.y); Serial.print("\t");
            Serial.print("Accel-z:"); Serial.print(accelRead.z); Serial.print("\t");
            Serial.print("\t");
        #endif
        // #define DISP_RAW_GYRO
        #ifdef DISP_RAW_GYRO // Gyro raw data
            Serial.print("Gyro-x:"); Serial.print(gyroRead.x); Serial.print("\t");
            Serial.print("Gyro-y:"); Serial.print(gyroRead.y); Serial.print("\t");
            Serial.print("Gyro-z:"); Serial.print(gyroRead.z); Serial.print("\t");
            Serial.print("\t");
        #endif
        // #define DISP_ROLL
        #ifdef DISP_ROLL // Roll - measurements and filtering results
            Serial.print("Roll-r:"); Serial.print(roll); Serial.print("\t");
            Serial.print("Roll-c:"); Serial.print(compAngleX); Serial.print("\t");
            Serial.print("Roll-k:"); Serial.print(kalAngleX); Serial.print("\t");
            Serial.print("\t");
        #endif
        // #define DDISP_PITCH
        #ifdef DISP_PITCH // Pitch - measurements and filtering results
            Serial.print("Pitch-r:"); Serial.print(pitch); Serial.print("\t");
            Serial.print("Pitch-c:"); Serial.print(compAngleY); Serial.print("\t");
            Serial.print("Pitch-k:"); Serial.print(kalAngleY); Serial.print("\t");
            Serial.print("\t");
        #endif
        // #define DISP_YAW
        #ifdef DISP_YAW // Yaw - measurements and filtering results
            Serial.print("Yaw-r:"); Serial.print(yaw); Serial.print("\t");
            Serial.print("Yaw-c:"); Serial.print(compAngleZ); Serial.print("\t");
            Serial.print("Yaw-k:"); Serial.print(kalAngleZ); Serial.print("\t");
            Serial.print("\t");
        #endif
        // #define DISP_GYRO_ANGLE
        #ifdef DISP_GYRO_ANGLE // Yaw - measurements and filtering results
            Serial.print("GyroX:"); Serial.print(gyroXangle); Serial.print("\t");
            Serial.print("GyroY:"); Serial.print(gyroYangle); Serial.print("\t");
            Serial.print("GyroZ:"); Serial.print(gyroZangle); Serial.print("\t");
        #endif
        // #define DISP_CHIP_TEMP
        #ifdef DISP_CHIP_TEMP
            // temperature = tempRaw / 340.0 + 36.53; //MPU6050 raw data
            Serial.print("Celsius-Chip:"); Serial.print(chipTemperature, 2); Serial.print("\t");
        #endif
        // #define DISP_AIR_TEMP
        #ifdef DISP_AIR_TEMP
            getAirTemp();
            Serial.print("Celsius-Air:"); Serial.print(airTemperature, 2); Serial.print("\t");
        #endif
        // #define DISP_PRESSURE
        #ifdef DISP_PRESSURE
            getPressure();
            Serial.print("Pressure-kPa:"); Serial.print(pressure/1000, 3); Serial.print("\t");
            Serial.print("Pressure-atm:"); Serial.print(pressure/101325.0, 4); Serial.print("\t");
        #endif
        // #define DISP_ALTITUDE
        #ifdef DISP_ALTITUDE
            getAltitude();
            Serial.print("Altitude:"); Serial.print(altitude, 2);
        #endif
        // ready for next reading
        Serial.print("\r\n");
    }
#endif

#ifdef MADGWICK_AHRS

    void initFilter(AccelRead& accelRead, MagRead& magRead) {
        if (isFirstData) {
            isFirstData = false;
            MadgwickAHRS.begin(100); // 100 Hz sample rate
        }
        else {
            return;
        }
    }
    // 2-g range accelerometer setting
    // float convertRawAcceleration(int aRaw) {
    //     // since we are using 2G range
    //     // -2g maps to a raw value of -32768
    //     // +2g maps to a raw value of 32767
    //     float a = (aRaw * 2.0) / 32768.0;
    //     return a;
    // }
    // 250-degree range gyroscope setting
    // float convertRawGyro(int gRaw) {
    //     // since we are using 250 degrees/seconds range
    //     // -250 maps to a raw value of -32768
    //     // +250 maps to a raw value of 32767
    //     float g = (gRaw * 250.0) / 32768.0;
    //     return g;
    // }

    void updateFilter(AccelRead& accelRead, GyroRead& gyroRead, MagRead& magRead) {
        if (isFirstData) return;
        #ifdef GYRO_DEG_TO_RAD
            double gyroXrate = gyroRead.x * DEG_TO_RAD; // deg to rad
            double gyroYrate = gyroRead.y * DEG_TO_RAD;
            double gyroZrate = gyroRead.z * DEG_TO_RAD;
        #else
            double gyroXrate = gyroRead.x; // deg/s
            double gyroYrate = gyroRead.y;
            double gyroZrate = gyroRead.z;
        #endif
        // Update Madgwick filter
        // float ax = convertRawAcceleration(accelRead.x);
        // float ay = convertRawAcceleration(accelRead.y);
        // float az = convertRawAcceleration(accelRead.z);
        // float gx = convertRawGyro(gyroRead.x);
        // float gy = convertRawGyro(gyroRead.y);
        // float gz = convertRawGyro(gyroRead.z);
        // MadgwickAHRS.updateIMU(gx, gy, gz, ax, ay, az);
        MadgwickAHRS.updateIMU(gyroXrate, gyroYrate, gyroZrate, accelRead.x, accelRead.y, accelRead.z);
        // Get filtered angles
        roll = MadgwickAHRS.getRoll();
        pitch = MadgwickAHRS.getPitch();
        yaw = MadgwickAHRS.getYaw();
        // Hold the module so that Z is pointing 'up' and you can measure the heading without x&y
        // Calculate heading when the magnetometer is level, then correct for signs of axis.
        float heading = atan2(magRead.y, magRead.x);
        #ifdef DECLI_ARNGL
            heading += DECLI_ARNGL;
        #endif
        // Correct for when signs are reversed.
        if(heading < 0) heading += 2*PI;
        // Check for wrap due to addition of declination.
        if(heading > 2*PI) heading -= 2*PI;
        // Convert radians to degrees for readability.
        heading *= RAD_TO_DEG; 
        Serial.print("Roll:"); Serial.print(roll); Serial.print("\t");
        Serial.print("Pitch:"); Serial.print(pitch); Serial.print("\t");
        Serial.print("Yaw-Alg:"); Serial.print(yaw); Serial.print("\t");
        Serial.print("Yaw-Mag:"); Serial.print(heading); Serial.print("\t");
        // #define DISP_CHIP_TEMP
        #ifdef DISP_CHIP_TEMP
            // temperature = tempRaw / 340.0 + 36.53; //MPU6050 raw data
            Serial.print("Celsius-Chip:"); Serial.print(chipTemperature, 2); Serial.print("\t");
        #endif
        // #define DISP_AIR_TEMP
        #ifdef DISP_AIR_TEMP
            getAirTemp();
            Serial.print("Celsius-Air:"); Serial.print(airTemperature, 2); Serial.print("\t");
        #endif
        // #define DISP_PRESSURE
        #ifdef DISP_PRESSURE
            getPressure();
            Serial.print("Pressure-kPa:"); Serial.print(pressure/1000, 3); Serial.print("\t");
            Serial.print("Pressure-atm:"); Serial.print(pressure/101325.0, 4); Serial.print("\t");
        #endif
        // #define DISP_ALTITUDE
        #ifdef DISP_ALTITUDE
            getAltitude();
            Serial.print("Altitude:"); Serial.print(altitude, 2);
        #endif
        Serial.print("\r\n");
    }

#endif

#ifdef MAHONY_AHRS

    void initFilter(AccelRead& accelRead, MagRead& magRead) {
        if (isFirstData)
            isFirstData = false;
        else
            return;
    }

    bool readyToPrint() {
        static unsigned long nowMillis;
        static unsigned long thenMillis;
        // If the Processing visualization sketch is sending "s"
        // then send new data each time it wants to redraw
        while (Serial.available()) {
            int val = Serial.read();
            if (val == 's') {
                thenMillis = millis();
                return true;
            }
        }
        // Otherwise, print 8 times per second, for viewing as
        // scrolling numbers in the Arduino Serial Monitor
        nowMillis = millis();
        if (nowMillis - thenMillis > 125) {
            thenMillis = nowMillis;
            return true;
        }
        return false;
    }

    //2-g range accelerometer setting
    float convertRawAcceleration(int aRaw) {
        // since we are using 2G range
        // -2g maps to a raw value of -32768
        // +2g maps to a raw value of 32767
        float a = (aRaw * 2.0) / 32768.0;
        return a;
    }

    //250-degree range gyroscope setting
    float convertRawGyro(int gRaw) {
        // since we are using 250 degrees/seconds range
        // -250 maps to a raw value of -32768
        // +250 maps to a raw value of 32767
        float g = (gRaw * 250.0) / 32768.0;
        return g;
    }

    void updateFilter(AccelRead& accelRead, GyroRead& gyroRead, MagRead& magRead) {
        if (isFirstData) return;
        // Update Mahony filter
        #ifdef RAW_TO_FLOAT_2G_250DEG
            float ax = convertRawAcceleration(accelRead.x);
            float ay = convertRawAcceleration(accelRead.y);
            float az = convertRawAcceleration(accelRead.z);
            float gx = convertRawGyro(gyroRead.x);
            float gy = convertRawGyro(gyroRead.y);
            float gz = convertRawGyro(gyroRead.z);
            // Update the Mahony filter, with scaled gyroscope
        #else
            float ax = (float)accelRead.x;
            float ay = (float)accelRead.y;
            float az = (float)accelRead.z;
            float gx = (float)gyroRead.x;
            float gy = (float)gyroRead.y;
            float gz = (float)gyroRead.z;
        #endif
        #ifdef SCALE
            float gyroScale = SCALE;
            MahonyAHRS.updateIMU(gx * gyroScale, gy * gyroScale, gz * gyroScale, ax, ay, az);
        #else
            MahonyAHRS.updateIMU(gx, gy, gz, ax, ay, az);
        #endif
        if (readyToPrint()) {
            // Get filtered angles
            roll = MahonyAHRS.getRoll();
            pitch = MahonyAHRS.getPitch();
            yaw = MahonyAHRS.getYaw();
            // Hold the module so that Z is pointing 'up' and you can measure the heading without x&y
            // Calculate heading when the magnetometer is level, then correct for signs of axis.
            float heading = atan2(magRead.y, magRead.x);
            Serial.print(magRead.x); Serial.print("     "); Serial.print(magRead.y);Serial.println("");
            #ifdef DECLI_ARNGL
                heading += DECLI_ARNGL;
            #endif
            // Correct for when signs are reversed.
            if(heading < 0) heading += 2*PI;
            // Check for wrap due to addition of declination.
            if(heading > 2*PI) heading -= 2*PI;
            // Convert radians to degrees for readability.
            heading *= RAD_TO_DEG; 
            Serial.print("Roll:"); Serial.print(roll); Serial.print("\t");
            Serial.print("Pitch:"); Serial.print(pitch); Serial.print("\t");
            Serial.print("Yaw-Alg:"); Serial.print(yaw); Serial.print("\t");
            Serial.print("Yaw-Mag:");Serial.print(heading); Serial.print("\t");
            // #define DISP_CHIP_TEMP
            #ifdef DISP_CHIP_TEMP
                // temperature = tempRaw / 340.0 + 36.53; //MPU6050 raw data
                Serial.print("Celsius-Chip:"); Serial.print(chipTemperature, 2); Serial.print("\t");
            #endif
            // #define DISP_CHIP_TEMP
            #ifdef DISP_CHIP_TEMP
                // temperature = tempRaw / 340.0 + 36.53; //MPU6050 raw data
                Serial.print("Celsius-Chip:"); Serial.print(chipTemperature, 2); Serial.print("\t");
            #endif
            // #define DISP_AIR_TEMP
            #ifdef DISP_AIR_TEMP
                getAirTemp();
                Serial.print("Celsius-Air:"); Serial.print(airTemperature, 2); Serial.print("\t");
            #endif
            // #define DISP_PRESSURE
            #ifdef DISP_PRESSURE
                getPressure();
                Serial.print("Pressure-kPa:"); Serial.print(pressure/1000, 3); Serial.print("\t");
                Serial.print("Pressure-atm:"); Serial.print(pressure/101325.0, 4); Serial.print("\t");
            #endif
            // #define DISP_ALTITUDE
            #ifdef DISP_ALTITUDE
                getAltitude();
                Serial.print("Altitude:"); Serial.print(altitude, 2);
            #endif
            Serial.print("\r\n");
        }
    }

#endif