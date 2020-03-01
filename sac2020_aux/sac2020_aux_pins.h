/**
 *             [ANOTHER FINE PRODUCT FROM THE NONSENSE FACTORY]
 *
 * Flight software for the Longhorn Rocketry Association's Spaceport America
 * Cup 2020 rocket. Built for the LRA Generation 2 flight computer, which also
 * flies in LRA's NASA Student Launch 2020 rocket. The configurability of the
 * software--combined with modularity and all-in-one-package style of the
 * Generation 2 flight computer--means the software can be easily adapted to run
 * in any high-power rocket.
 *
 * @file      sac2020_main_pins.h
 * @purpose   Pinouts for the auxiliary flight computer node.
 * @author    Stefan deBruyn
 * @updated   2/2/2020
 */

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
