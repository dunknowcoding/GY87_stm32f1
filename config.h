#ifndef config_h
#define config_h

/*********************************************************************************/
//GY-85 settings
/*********************************************************************************/
#define GY85
#define DISP_CHIP_TEMP // Gyro temperature display

/*********************************************************************************/
//GY-87 settings
/*********************************************************************************/
// #define GY87
// // #define GY87_CALIB
// #define QMC5883L_SENSITIVITY QMC5883L_RANGE_8G // Use the sensitivity that matches your sensor's range setting, 8G or 2G
// // To calculate a more accurate altitude, enter the correct mean sea-level pressure. For example, if the current pressure is 1019.00 hPa
// // enter 101900 since we include two decimal places in the integer value
// #define DISP_ALTITUDE 101900 // in meters
// #define DISP_AIR_TEMP // in Celsius
// #define DISP_PRESSURE // in Pascals and atmosphere 
// #define DISP_CHIP_TEMP // Gyro temperature display

/*********************************************************************************/
/*only ONE set of filter options below are vaild*/
/*********************************************************************************/

//MADGWICK_AHRS settings
/*********************************************************************************/
// #define MADGWICK_AHRS // For GY-85, <GYRO_DEG_TO_RAD> is enabled
// #define GYRO_DEG_TO_RAD // Convert to rad/s, for deg/s, MADGWICK_AHRS

// // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
// // Find yours here: http://www.magnetic-declination.com/
// // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians (#define DECLI_ARNGL 0.22;)
// // If you cannot find your Declination, comment out this line, your compass will be slightly off.
// #define DECLI_ARNGL 0.1885;

/*********************************************************************************/
//KALMAN settings
/*********************************************************************************/
// #define KALMAN // For GY-85, <GYRO_RAD_TO_DEG> is disabled 
// // #define GYRO_RAD_TO_DEG // Convert to deg/s, for rad/s
// // #define UNBIASED_GYRO_RATE // Calculate gyro angle using the unbiased rate
// #define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead
// // Serial display settings, at lease printing one item
// // #define DISP_RAW_ACC // print accelerometer raw data
// // #define DISP_RAW_GYRO // print gyroscope raw data
// #define DISP_ROLL // Roll - measurements and filtering results
// #define DISP_PITCH // Pitch - measurements and filtering results
// // #define DISP_YAW // Yaw - measurements and filtering results
// // #define DISP_GYRO_ANGLE // Display Gyro incrementals, defined by <UNBIASED_GYRO_RATE>

/*********************************************************************************/
// MAHONY_AHRS settings
/*********************************************************************************/
#define MAHONY_AHRS
#define SCALE 0.001 // TODO: the filter updates too fast
// #define RAW_TO_FLOAT_2G_250DEG // Change raw data (CurieIMU, 2G, 250deg/s) to [-32768, 32767]

// Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
// Find yours here: http://www.magnetic-declination.com/
// Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians (#define DECLI_ARNGL 0.22;)
// If you cannot find your Declination, comment out this line, your compass will be slightly off.
#define DECLI_ARNGL 0.1885;

#endif