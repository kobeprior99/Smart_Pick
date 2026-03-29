/**
 * @file LEDs.hpp
 * @brief Simple GPIO-based LED control interface for system status indication.
 *
 * This file defines the LEDs class, which provides an abstraction for controlling
 * onboard LEDs used for visual feedback in an embedded system. The LEDs are used
 * to indicate system states such as recording, data access, and threshold events.
 *
 * Hardware Interface:
 * -------------------
 * | LED  | GPIO Pin |
 * |------|----------|
 * | Red  | IO2      |
 * | B1   | IO43     |
 * | B2   | IO44     |
 * | B3   | IO42     |
 *
 * Features:
 * ---------
 * - Initializes LED GPIO pins and ensures known startup state
 * - Provides clear semantic functions for system state indication
 * - Blocking flash functionality for event signaling
 * - Startup LED sequence for visual system check
 *
 * LED Behavior:
 * -------------
 * - Red LED:
 *   • ON  → Recording active
 *   • OFF → Recording stopped
 *
 * - B1 LED:
 *   • ON  → Data read operation active
 *   • OFF → Data read complete
 *
 * - B2 LED:
 *   • Flash → Threshold A event detected
 *
 * - B3 LED:
 *   • Flash → Threshold B event detected
 *
 * Startup Sequence:
 * -----------------
 * - All LEDs are initialized to OFF
 * - Sequential flash pattern is executed twice:
 *   RED → B1 → B2 → B3
 * - Confirms correct GPIO configuration and system startup
 *
 * Usage:
 * ------
 * LEDs leds;
 * leds.Init();
 *
 * // Recording state
 * leds.Start_Recording();
 * leds.End_Recording();
 *
 * // Data read indication
 * leds.Read_Data();
 * leds.End_Read_Data();
 *
 * // Event signaling
 * leds.ThresholdA_Reached();
 * leds.ThresholdB_Reached();
 *
 * Notes:
 * ------
 * - Flash operations use blocking delays (delay()), which may impact
 *   real-time performance if used in time-critical sections.
 * - Designed for simple debugging and user feedback rather than
 *   high-frequency signaling.
 *
 * @author Kobe Prior
 * @date 2026
 */
