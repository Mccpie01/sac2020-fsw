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
 * @file      sac2020_state.h
 * @purpose   Flight computer state representation objects.
 * @author    Stefan deBruyn
 * @updated   3/1/2020
 */

#ifndef SAC2020_STATE_H
#define SAC2020_STATE_H

#ifdef DECODE
    #include <cstdint>
#endif

/**
 * All possible vehicle states, used in state machine management.
 */
typedef enum VehicleState : uint8_t
{
    PRELTOFF, // Pre-liftoff; sitting on the pad.
    PWFLIGHT, // Powered flight; motor burning.
    CRUISING, // Motor spent, still ascending.
    CRSCANRD, // Cruising with canards deployed.
    FALLDROG, // Falling under drogue parachute.
    FALLMAIN, // Falling under main parachute.
    CONCLUDE  // Flight over, with rocket likely grounded.
} VehicleState_t;

/**
 * State vector for main flight computer. Must be <= TELEM_PACKET_SIZE bytes in
 * length.
 */
typedef struct MainStateVector
{
    // Timestamp.
    float time;

    // Rocket state as estimated by nav filter.
    float altitude;
    float velocity;
    float acceleration;
    float accel_vertical;

    // Sensor values of interest.
    float pressure;
    float temperature;
    float baro_altitude;
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float quat_w;
    float quat_x;
    float quat_y;
    float quat_z;

    // Estimated launchpad altitude.
    float launchpad_altitude;

    // Current state of the flight computer.
    VehicleState_t state;

    // Temperature read by IMU. Placed here to minimize padding.
    int8_t imu_temp;

} MainStateVector_t;

#endif
