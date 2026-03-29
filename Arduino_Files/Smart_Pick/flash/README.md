/**
 * @file Flash.hpp
 * @brief High-performance SPI flash logging interface with double buffering.
 *
 * This file defines a templated flash logging class designed for efficient,
 * high-speed data acquisition and storage in external SPI flash memory.
 * It is optimized for embedded systems requiring reliable, non-volatile
 * logging of sensor data (e.g., accelerometer and capacitance measurements).
 *
 * Project:
 * --------
 * EDNS 492 - Senior Design
 * Smart Pick for Rock Mining
 *
 * Authors:
 * --------
 * Sophia Mimlitz, Kobe Prior
 * Date: 2026
 *
 * Hardware Interface (SPI):
 * --------------------------
 * | Signal | GPIO Pin |
 * |--------|----------|
 * | SCK    | IO47     |
 * | MOSI   | IO48     |
 * | MISO   | IO13     |
 * | CS     | IO14     |
 *
 * Flash Characteristics:
 * ----------------------
 * - Page Size:   256 bytes
 * - Sector Size: 4 KB
 * - Supports JEDEC ID detection
 * - Uses standard SPI command set (WREN, READ, PP, SE, etc.)
 *
 * Key Features:
 * -------------
 * - Double-buffered logging (Red/Blue buffers) for continuous data capture
 * - Non-blocking flash writes via service routine
 * - Automatic page alignment and boundary handling
 * - Configurable logging region in flash memory
 * - Overflow detection for data safety
 * - Timing utilities for performance benchmarking
 *
 * Data Format:
 * ------------
 * FlashPacket (16 bytes total):
 * - timestamp     : uint32_t
 * - mag_accel     : uint32_t (accelerometer magnitude)
 * - capacitance   : uint32_t (CDC reading)
 * - reserved      : uint32_t (alignment / future use)
 *
 * Logging Architecture:
 * ---------------------
 * - Data is first stored in RAM buffers (Red/Blue)
 * - When a buffer fills, it is marked "ready"
 * - serviceFlashWrite() writes full buffers to flash in the background
 * - Ensures minimal disruption to real-time data acquisition
 *
 * Usage:
 * ------
 * Instantiate the class with a compile-time buffer size:
 *
 *   flash<64> logger;  // 64 packets per buffer
 *
 * Initialize and configure:
 *
 *   logger.begin();
 *   logger.configureLogRegion(0x000000, 0x100000); // 1 MB region
 *   logger.eraseLogRegion();
 *
 * Logging data:
 *
 *   FlashPacket pkt = {timestamp, accel_mag, cap_value, 0};
 *   logger.appendPacket(pkt);
 *
 * In main loop:
 *
 *   logger.serviceFlashWrite();
 *
 * Finalizing:
 *
 *   logger.finalizeLog();
 *
 * Notes:
 * ------
 * - Flash must be erased before writing.
 * - Writes are page-limited (256 bytes max per operation).
 * - serviceFlashWrite() should be called frequently to avoid overflow.
 * - Overflow occurs if both buffers fill before being written to flash.
 * - Logging region boundaries are strictly enforced for safety.
 *
 * Performance:
 * ------------
 * - Includes utilities to measure:
 *   • Sector erase time
 *   • Page program time
 * - Useful for tuning buffer size and sampling rate.
 *
 * @warning
 * Failure to service buffers fast enough will result in data loss.
 *
 * @author Kobe Prior
 * @date 2026
 */
