/**
 * @file CDCConverter.hpp
 * @brief Interface for interacting with the AD7746 capacitance-to-digital converter (CDC) over I2C.
 *
 * This file defines the CDC class, which provides functionality to configure,
 * initialize, and read capacitance data from the AD7746 CDC. The device is part
 * of the CN0552 platform and enables high-resolution capacitive sensing.
 *
 * Device Overview (AD7746 / CN0552):
 * ----------------------------------
 * - Capacitance Measurement Range:
 *   • ±4.096 pF (standard range, up to ~17 pF common-mode)
 *   • Extended up to ~50 pF (with bulk capacitance up to ~200 pF)
 *
 * - Resolution:
 *   • Down to ~4 aF (attofarads)
 *
 * - Accuracy:
 *   • ~4 fF (standard range)
 *   • ~40–50 fF (extended range)
 *
 * - Update Rate:
 *   • 10 Hz to 90 Hz
 *   • Recommended:
 *       - 16.1 SPS for 50/60 Hz noise rejection
 *       - ~90 SPS for faster response
 *
 * - Measurement Characteristics:
 *   • Measures small differential capacitance changes
 *   • Supports large static (common-mode) capacitance offsets
 *
 * Features:
 * ---------
 * - Initializes AD7746 CDC with continuous conversion mode
 * - Reads 24-bit raw capacitance data
 * - Supports extended measurement range via excitation configuration
 * - Includes data-ready polling via status register
 *
 * Usage:
 * ------
 * Instantiate the CDC class with a TwoWire object, then call Setup()
 * before reading capacitance data.
 *
 * Example:
 * --------
 * TwoWire I2C = Wire;
 * CDC cdc(I2C);
 *
 * if (cdc.Setup()) {
 *     if (cdc.dataReady()) {
 *         uint32_t cap = cdc.readCapacitanceRaw();
 *     }
 * }
 *
 * Notes:
 * ------
 * - Raw output is a 24-bit value and must be converted to capacitance (farads)
 *   using device-specific scaling from the AD7746 datasheet.
 * - Offset DAC (CAPDAC) is used to shift the measurement range.
 * - Excitation settings (EXCA/EXCB) enable extended measurement range.
 * - Data-ready flag must be checked before reading new data.
 *
 * Register Reference:
 * -------------------
 * Register definitions are based on the Analog Devices no-OS driver:
 * https://github.com/analogdevicesinc/no-OS/blob/main/drivers/cdc/ad7746/ad7746.h
 *
 * @author Kobe Prior
 * @date 2026
 */
