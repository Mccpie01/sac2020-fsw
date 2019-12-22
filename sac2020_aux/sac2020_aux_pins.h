#ifndef SAC2020_AUX_PINS_H
#define SAC2020_AUX_PINS_H

/**
 * GPS.
 */
#define PIN_GPS_FIX 3 // High when GPS lock is held.

/**
 * RF module.
 */
#define PIN_RFM_ENABLE  4
#define PIN_RFM_RESET   5
#define PIN_RFM_INTR    16
#define PIN_RFM_CHIPSEL 20

/**
 * Fault LEDs.
 */
#define PIN_LED_SD_FAULT  21 // SD card.
#define PIN_LED_RFM_FAULT 22 // RF module.
#define PIN_LED_GPS_FAULT 23 // GPS.
#define PIN_LED_FNW_FAULT 19 // Flight computer network.

/**
 * SD card.
 */
#define PIN_SDCARD_CHIPSEL 15

#endif
