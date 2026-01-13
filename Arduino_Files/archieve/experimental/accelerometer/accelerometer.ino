/******************************************************************************
 * File: accelerometer.ino
 * Author: Kobe Prior
 * Date: January 12, 2026
 * 
 * Description:
 *   This Arduino sketch is used to experiment with the MPU6050 accelerometer.
 *   The program captures vibration data from the sensor for analysis, 
 *   including acceleration in the X, Y, and Z axes, and optionally computes 
 *   derived metrics such as velocity and displacement.
 * 
 * Usage:
 *   - Connect the MPU6050 to the Arduino via I2C (SDA, SCL).
 *   - Initialize the sensor using Wire library.
 *   - Capture raw acceleration and/or gyroscope data.
 *   - Print data to Serial for logging or further processing.
 * 
 * Hardware:
 *   - MPU6050 accelerometer/gyroscope module
 *   - Arduino board (e.g., Uno, Nano)
 * 
 * Notes:
 *   - Ensure proper pull-up resistors are connected on SDA/SCL lines if needed.
 *   - Use library functions or direct I2C reads as required for experiments.
 *****************************************************************************/
//TODO
