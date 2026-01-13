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
 *   -SCL Pin 22
 *   -SDA Pin 21
 * Libraries:
 * download and install adafruit mpu6050
 * 
 * Notes:
 *   - Ensure proper pull-up resistors are connected on SDA/SCL lines if needed.
 *   - Use library functions or direct I2C reads as required for experiments.
 *****************************************************************************/

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

void setup(void) {
  Serial.begin(115200);
  while (!Serial) {
    delay(10); // will pause Zero, Leonardo, etc until serial console opens
  }

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("");
  delay(100);
}

void loop() {
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */
  Serial.print("AccelZ:");
  Serial.print(a.acceleration.z);
  delay(10);
}
