/**
 *             [ANOTHER FINE PRODUCT FROM THE NONSENSE FACTORY]
 *
 * Flight software for the Longhorn Rocketry Association's Spaceport America
 * Cup 2020 rocket. Built for the LRA Generation 2 flight computer.
 *
 * @file      sac2020_main_pins.h
 * @purpose   Pinouts for the main flight computer node.
 * @author    Stefan deBruyn
 * @updated   2/2/2020
 */

#ifndef SAC2020_MAIN_PINS_H
#define SAC2020_MAIN_PINS_H

/**
 * BNO055 IMU.
 */
#define PIN_BNO055_RESET 2
#define PIN_BNO055_ADDR  3
#define PIN_BNO055_INTR  16

/**
 * Canard fin servos.
 */
#define PIN_SERVO1_PWM 4
#define PIN_SERVO2_PWM 5

/**
 * Fault LEDs.
 */
#define PIN_LED_SYS_FAULT   9
#define PIN_LED_IMU_FAULT   10
#define PIN_LED_BLE_FAULT   13
#define PIN_LED_PYRO2_FAULT 12
#define PIN_LED_PYRO1_FAULT 17
#define PIN_LED_FNW_FAULT   20
#define PIN_LED_BARO_FAULT  21

/**
 * Recovery pyros.
 */
#define PIN_PIN_RTS 6
#define PIN_BLE_CTS 13
#define PIN_BLE_MOD 22
#define PIN_BLE_DFU 23
#define BLE_SERIAL  Serial3

/**
 * Continuity checking.
 */
#define PIN_CONT_ENABLE    6
#define PIN_CONT_POWER     13
#define PIN_CONT_PYRO1_CHK 22
#define PIN_CONT_PYRO2_CHK 23

/**
 * Miscellaneous.
 */
#define PIN_LLC_OUT_ENABLE 11
#define PIN_BUZZER         15

#endif
