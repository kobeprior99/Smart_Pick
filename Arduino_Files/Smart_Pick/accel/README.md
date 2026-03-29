/**
 * @file Accelerometer.hpp
 * @brief Interface for interacting with the onboard ADXL375 accelerometer over I2C.
 *
 * This header file defines the ACC class, which provides functionality to
 * configure, read, and debug the onboard accelerometer using the I2C interface.
 * It includes methods for initialization, interrupt handling, and retrieving
 * acceleration magnitude data.
 *
 * Hardware Connections (I2C Interface):
 * ------------------------------------
 * | Signal      | GPIO Pin |
 * |-------------|----------|
 * | SDA         | IO41     |
 * | SCL         | IO40     |
 * | Interrupt 1 | IO36     |
 * | Interrupt 2 | IO35     |
 *
 * Features:
 * ---------
 * - Initializes ADXL375 accelerometer with configurable settings
 * - Reads raw acceleration data and computes magnitude
 * - Supports activity detection via hardware interrupts
 * - Provides debug utilities for register inspection and live data output
 *
 * Usage:
 * ------
 * Instantiate the ACC class with a TwoWire object, then call Setup()
 * to initialize the device before reading data.
 *
 * Example:
 * --------
 * TwoWire I2C = Wire;
 * ACC accel(I2C);
 * accel.Setup();
 * uint32_t magnitude = accel.readAccelMagnitude();
 *
 * Notes:
 * ------
 * - Raw acceleration data is returned in LSB and must be converted
 *   externally using the sensor scale factor (~49 mg/LSB for ADXL375).
 * - Reading INT_SOURCE clears interrupt flags.
 * - Please adjust the threshold for activity as necessary from testing 
 * @author Kobe Prior
 * @date 2026
 */
