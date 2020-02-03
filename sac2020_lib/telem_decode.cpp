/**
 * Script for decoding telemetry dump from aux computer.
 * Usage: make clean; make decode; ./decode TELEM.DAT
 */

#include <stdio.h>
#include <stdlib.h>

#include "sac2020_state.h"

char* state_name(uint8_t state)
{
    switch (state)
    {
        case VehicleState_t::PRELTOFF: return "PRELTOFF";
        case VehicleState_t::PWFLIGHT: return "PWFLIGHT";
        case VehicleState_t::CRUISING: return "CRUISING";
        case VehicleState_t::CRSCANRD: return "CRSCANRD";
        case VehicleState_t::FALLDROG: return "FALLDROG";
        case VehicleState_t::FALLMAIN: return "FALLMAIN";
        case VehicleState_t::CONCLUDE: return "CONCLUDE";
        default:                       return "UNKNOWN";
    };
}

int main(int ac, char** av)
{
    FILE* fptr = NULL;
    fptr = fopen(av[1], "rb");

    MainStateVector_t vec;
    uint32_t packet_count = 0;
    while (fread(&vec, sizeof(vec), 1, fptr) > 0)
    {
        packet_count++;

        // Timestamp and state.
        printf("[%06.2f//%s] ", vec.time, state_name(vec.state));

        // Kalman filter state estimate.
        printf("kfalt=%09.2f kfvel=%09.2f kfacc=%09.2f   ",
               vec.altitude, vec.velocity, vec.acceleration);

        // Sensor readings.
        printf("q=%09.2f t=%09.2f balt=%09.2f   ",
               vec.pressure, vec.temperature, vec.baro_altitude);
        printf("accx=%09.2f accy=%09.2f accz=%09.2f accv=%09.2f   ",
               vec.accel_x, vec.accel_y, vec.accel_z, vec.accel_vertical);
        printf("r=%09.2f p=%09.2f y=%09.2f",
               vec.gyro_r, vec.gyro_p, vec.gyro_y);

        printf("\n");
    }

    fclose(fptr);
    printf("Decoded %d telemetry packets\n", packet_count);
}
