#ifndef ACCEL_HPP
#define ACCEL_HPP

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

class Accel {
public:
    // Constructor
    Accel() {}

    // Initialize the MPU6050
    bool begin(MPU6050_accelRange_t accelRange = MPU6050_RANGE_16_G,
               MPU6050_gyroRange_t gyroRange = MPU6050_RANGE_250_DEG,
               mpu6050_bandwidth_t filterBandwidth = MPU6050_BAND_21_HZ) {
        if (!mpu.begin()) {
            return false;
        }
        mpu.setAccelerometerRange(accelRange);
        mpu.setGyroRange(gyroRange);
        mpu.setFilterBandwidth(filterBandwidth);
        delay(100); // small stabilization delay
        return true;
    }

    // Get accelerometer Z-axis value
    float getAccelZ() {
        sensors_event_t a;
        mpu.getEvent(&a, nullptr, nullptr);
        return a.acceleration.z;
    }

private:
    Adafruit_MPU6050 mpu;
};

/* Global instance */
static Accel accel;

#endif // ACCEL_HPP
