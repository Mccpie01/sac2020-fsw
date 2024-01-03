

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
 * @file      sac2020_profile.h
 * @purpose   Constants specific to the vehicle and flight profile including
 *            state transition time bounds, liftoff trigger, etc.
 *            Current configuration: SLI rocket, L1355 motor.
 * @author    Stefan deBruyn
 * @updated   2/3/2020
 */

#ifndef SAC2020_PROFILE_H
#define SAC2020_PROFILE_H

#include <Photic.hpp>
#include "sac2020_lib.h"

/**
 * The following values define the time windows around detection of each flight
 * event. The event may not be detected by sensor readings before the low bound
 * of the window. The event will automatically trigger after the high bound,
 * regardless of sensor readings. All values are in seconds since liftoff.
 */

/**
 * Motor burnout detection window.
 */
#define EVENT_BURNOUT_T_LOW_S  2
#define EVENT_BURNOUT_T_HIGH_S 5

/**
 * Canard deployment detection window.
 */
#define EVENT_CANARDS_T_LOW_S  9
#define EVENT_CANARDS_T_HIGH_S 10

/**
 * Apogee/drogue deployment detection window.
 */
#define EVENT_APOGEE_T_LOW_S  12
#define EVENT_APOGEE_T_HIGH_S 20

/**
 * Main deployment detection window.
 */
#define EVENT_MAIN_T_LOW_S  40
#define EVENT_MAIN_T_HIGH_S 56

/**
 * Mission conclusion detection window.
 */
#define EVENT_CONCLUDE_T_HIGH_S 90

/**
 * Altitude relative to launchpad at which to deploy canards.
 */
#define CANARD_DEPLOYMENT_ALTITUDE_M 999999

/**
 * Altitude relative to launchpad at which to deploy main parachute.
 */
#define MAIN_DEPLOYMENT_ALTITUDE_M 182.88

/**
 * Minimum acceleration rolling average required to declare liftoff and enter
 * powered flight state.
 */
#ifdef GROUND_TEST
    #define LIFTOFF_ACCEL_TRIGGER_MPSSQ 1
#else
    #define LIFTOFF_ACCEL_TRIGGER_MPSSQ 2 * Photic::EARTH_SLGRAV_MPSSQ
#endif

#endif
