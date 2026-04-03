/*
 * CENTRALIZED CONSTANTS CONFIGURATION
 * =====================================
 * 
 * This file consolidates all magic numbers and constants used throughout
 * the project to improve maintainability and reduce code duplication.
 * 
 * Previous locations: 2_Remote.h, 3_ESC.h, and other configuration files
 * 
 * Date: 2026-04-03
 * Phase: Refactoring Phase 1 - Centralize Constants
 */

#ifndef CONFIG_CONSTANTS_H
#define CONFIG_CONSTANTS_H

// ============================================================================
// PULSE SIGNAL CALIBRATION CONSTANTS
// ============================================================================
// Used in: 2_Remote.h (Remote configuration profiles)
// Purpose: Define the neutral position and signal range for RC pulses

// Neutral range: 1500 +/- this value (microseconds)
// Typical value: 30 microseconds (safe margin around 1500µs neutral position)
const uint16_t PULSE_NEUTRAL_RANGE = 30;

// Maximum pulse span from neutral position
// Theory: 500µs (1500 center +/- 500 = 1000-2000ms)
// Practical: Usually around 480-500µs depending on remote type
// Array of values for different remote configurations:
const uint16_t PULSE_SPAN_DEFAULT = 480;      // Most remotes
const uint16_t PULSE_SPAN_RGT_EX86100 = 500;  // RGT MT-305
const uint16_t PULSE_SPAN_FRSKY_HARMONY = 495; // FrSky Tandem XE Harmony Loader

// Standard pulse range (PWM servo signal)
const uint16_t PULSE_MIN = 1000;  // Minimum pulse width (microseconds)
const uint16_t PULSE_MAX = 2000;  // Maximum pulse width (microseconds)
const uint16_t PULSE_CENTER = 1500; // Center/neutral pulse width (microseconds)

// ============================================================================
// COMMUNICATION PROTOCOL CONSTANTS
// ============================================================================
// Used in: 2_Remote.h (Communication settings)

// SBUS communication
const uint32_t SBUS_BAUD_DEFAULT = 100000;        // Standard baud rate
const uint32_t SBUS_BAUD_MIN = 96000;             // Lower working limit
const uint32_t SBUS_BAUD_MAX = 104000;            // Upper working limit
const uint16_t SBUS_FAILSAFE_TIMEOUT_MS = 100;   // Failsafe trigger timeout (milliseconds)

// ============================================================================
// ESC & MOTOR CONTROL CONSTANTS
// ============================================================================
// Used in: 3_ESC.h (ESC settings)

// Top speed adjustment (pulse span for ESC output)
// Default: 500 (full ESC power available, 1000 = half power, etc.)
const uint16_t ESC_PULSE_SPAN_DEFAULT = 500;

// Additional takeoff punch (acceleration assist around neutral)
// Range: 0-150+ depending on motor/ESC combination
const uint16_t ESC_TAKEOFF_PUNCH_DEFAULT = 70;

// Additional reverse speed boost
// Range: 0-220+ depending on ESC type
const uint16_t ESC_REVERSE_PLUS_DEFAULT = 0;

// Direction change limit for HYDROSTATIC_MODE
// Range: 0-100 (percentage of throttle)
const uint16_t DIRECTION_CHANGE_LIMIT_DEFAULT = 80;

// RZ7886 motor driver frequency
// Recommended: 500 Hz (not audible when engine sound running)
const uint16_t RZ7886_FREQUENCY_DEFAULT = 500;

// RZ7886 drag brake duty cycle
// Range: 0-100% (100 = maximum brake power while standing)
const uint8_t RZ7886_DRAGBRAKE_DUTY_DEFAULT = 100;

// Brake margin (prevents rolling back while braking)
// Range: 0-20 (experimental, NEVER above 20!)
// Value: 10 for RZ7886 + 370 motor, otherwise 0
const uint16_t BRAKE_MARGIN_DEFAULT = 0;

// Crawler mode ESC ramp time
// Range: 10-15 (lower = more direct control, less virtual inertia)
// WARNING: Very low settings may damage transmission!
const uint8_t CRAWLER_ESC_RAMP_TIME_DEFAULT = 10;

// Global acceleration percentage
// Range: 100-200% (200 for Jeep, 150 for 1/8 Landy)
const uint16_t GLOBAL_ACCELERATION_PERCENTAGE_DEFAULT = 100;

// ============================================================================
// BATTERY PROTECTION CONSTANTS
// ============================================================================
// Used in: 3_ESC.h (Battery settings)

// LiPo cell voltage thresholds
const float CUTOFF_VOLTAGE_DEFAULT = 3.3;        // Per cell (NEVER below 3.2V!)
const float FULLY_CHARGED_VOLTAGE_DEFAULT = 4.2; // Per cell (NEVER above!)
const float RECOVERY_HYSTERESIS_DEFAULT = 0.2;   // Hysteresis voltage (V)

// Voltage divider resistor values (Ohms)
// WARNING: Ratio must be 4:1 or higher! (e.g., 10k/2k, 20k/4k, 100k/20k)
// NEVER use ratios below 4:1 (e.g., 10k/5k = 2:1) - will damage controller!
const uint32_t RESISTOR_TO_BATTERY_PLUS_DEFAULT = 9400;  // Ohms
const uint32_t RESISTOR_TO_GND_DEFAULT = 1000;          // Ohms

// Diode voltage drop compensation
// Typical: 0.31-0.34V for SS34 diode
const float DIODE_DROP_DEFAULT = 0.31;

// Out of fuel alert volume
const volatile int OUT_OF_FUEL_VOLUME_PERCENTAGE_DEFAULT = 80;

// ============================================================================
// CHANNEL CONFIGURATION ARRAY SIZES
// ============================================================================
// Used in: 2_Remote.h (Remote configuration profiles)

// Number of RC channels supported (0-16, where 0 is unused)
const uint8_t MAX_RC_CHANNELS = 16;
const uint8_t CHANNEL_ARRAY_SIZE = 17; // 0-16 inclusive

// Special channel values
const uint8_t CHANNEL_NONE = 0xFF; // Represents no channel assigned

// ============================================================================
// EEPROM & SYSTEM CONSTANTS
// ============================================================================
// Used in: 0_generalSettings.h (System settings)

// EEPROM configuration ID
// Change this to reset EEPROM defaults on boot
const uint8_t EEPROM_ID_DEFAULT = 5;

// WiFi transmission power options
// Lower power = less speaker noise & longer battery life
const wifi_power_t WIFI_POWER_DEFAULT = WIFI_POWER_7dBm; // Recommended

// Configuration website default credentials
// These can be changed via the web interface
const String WIFI_SSID_DEFAULT = "My_Truck";
const String WIFI_PASSWORD_DEFAULT = "123456789";

// ============================================================================
// NOTES ON MAGIC NUMBERS
// ============================================================================
/*
 * PULSE_NEUTRAL_RANGE (30µs):
 *   - Safe margin around 1500µs neutral position
 *   - Prevents jitter near center position
 *   - Allows channels to detect slight stick movements
 * 
 * PULSE_SPAN (480-500µs):
 *   - Theory suggests 500µs (full range from 1000-2000ms)
 *   - Practice shows 480-500µs depending on remote calibration
 *   - Different remotes may need adjustment
 * 
 * SBUS_BAUD (100000):
 *   - Standard SBUS baud rate
 *   - Can be lowered if channels unstable (96000-104000 working range)
 * 
 * ESC calibration values:
 *   - escPulseSpan: Varies greatly by ESC/motor/transmission combo
 *   - escTakeoffPunch: Assists weak motors around neutral (0-150+)
 *   - escReversePlus: Compensates weak reverse capability
 * 
 * Battery protection:
 *   - CUTOFF_VOLTAGE: 3.3V per LiPo cell is industry standard
 *   - RESISTOR values: Critical! Wrong ratios damage controller
 */

#endif // CONFIG_CONSTANTS_H