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
    bool begin() {
        if (!mpu.begin()) {
            return false;
        }
        mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
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

#endif // ACCEL_HPP
