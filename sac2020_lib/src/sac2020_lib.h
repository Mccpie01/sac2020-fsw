#ifndef SAC2020_LIB_H
#define SAC2020_LIB_H

#include <TeensyThreads.h>
#include <vector>

/**
 * Altitude of Truth or Consequences, NM.
 */
#define LAUNCHPAD_ALTITUDE 1293.876
/**
 * Toggles debug prints to Serial.
 */
#define DEBUG_SERIAL Serial

/**
 * Flight network communication line (serial line between main and aux FCs).
 */
#define FNW_SERIAL Serial1

/**
 * Baud rate for FNW_SERIAL.
 */
#define FNW_BAUD 115200

/**
 * Whether or not we are using the flight computer network. If defined, the
 * flight computers will attempt to handshake on setup() and block until
 * consensus is established.
 */
#define USING_FNW

/********************************* TYPEDEFS ***********************************/

/**
 * Denotes the status of a particular system component.
 */
typedef enum Status : uint8_t
{
    OFFLINE, // Uninitialized.
    ONLINE,  // Initialized and running.
    FAULT    // Encountered an issue.
} Status_t;

/**
 * Configuration for LED status pulses.
 */
typedef struct PulseLEDs
{
    uint8_t pulses;            // Number of pulses (flashes) per LED.
    std::vector<uint8_t> pins; // LED pins to pulse.
} PulseLEDs_t;

/**
 * State vector for main flight computer.
 */
typedef struct MainStateVector
{
    // Timestamp.
    float time;

    // Rocket state as estimated by nav filter.
    float altitude;
    float velocity;
    float acceleration;

    // Sensor values of interest.
    float pressure;
    float temperature;
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_r;
    float gyro_p;
    float gyro_y;

    // Event flags.
    bool liftoff;
    bool burnout;
    bool canards_deployed;
    bool apogee;
    bool drogue_deployed;
    bool main_deployed;

    // Current state of the flight computer.
    uint8_t state;

} MainStateVector_t;

/**
 * State vector for auxiliary flight computer.
 */
typedef struct AuxStateVector
{

} AuxStateVector_t;

/**
 * All possible vehicle states, used in state machine management.
 */
typedef enum VehicleState : uint8_t
{
    PRELTOFF, // Pre-liftoff; sitting on the pad.
    PWFLIGHT, // Powered flight; motor burning.
    CRUISING, // Motor spent, still ascending.
    FALLDROG, // Falling under drogue parachute.
    FALLMAIN, // Falling under main parachute.
    CONCLUDE  // Flight over, with rocket likely grounded.
} VehicleState_t;

/**
 * Tokens are 1-byte pieces of metadata sent between flight computers.
 */
typedef uint8_t token_t;

/********************************* GLOBALS ************************************/

/**
 * Number of LED pulses that indicate successful initialization of a component.
 */
const uint8_t SYS_ONLINE_LED_PULSES = 3;

/**
 * Lock for synchronizing digital IO.
 */
extern Threads::Mutex g_dio_lock;

/**
 * Tokens sent between flight computers.
 */
const token_t FNW_TOKEN_NIL = 0x00; // Null token.
const token_t FNW_TOKEN_AOK = 0xAA; // Something went OK.
const token_t FNW_TOKEN_ERR = 0xBB; // Something went wrong.
const token_t FNW_TOKEN_VEC = 0xCC; // State vector payload follows.

/**
 * LED pulse configuration for component LEDs. Written once by main thread after
 * system startup, read many times by LED pulse thread.
 */
extern PulseLEDs_t g_led_pulse_conf;

/******************************** PROTOTYPES **********************************/

/**
 * Raises a flight computer fault for a particular component.
 *
 * @param   k_pin  Fault indicator LED pin.
 * @param   k_msg  Fault message, printed to DEBUG_SERIAL if defined.
 * @param   k_stat Global status for component to be set to Status_t::FAULT.
 */
void fault(uint8_t k_pin, const char* k_msg, Status_t& k_stat);

/**
 * Gets the time according to Arduino's millis(), reinterpreted as seconds.
 *
 * @ret     Current system time in seconds.
 * @return [description]
 */
inline double time_s();

/**
 * Pulses LEDs to indicate system status. In general, the LEDs being pulsed will
 * correspond to system components which successfully initialized.
 *
 * @param   k_conf Pulse configuration.
 */
void pulse_leds(const PulseLEDs_t k_conf);

#endif
