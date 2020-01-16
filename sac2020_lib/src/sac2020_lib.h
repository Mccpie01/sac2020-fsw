#ifndef SAC2020_LIB_H
#define SAC2020_LIB_H

#include <vector>

#ifndef FF
    #include <TeensyThreads.h>

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

    /**
     * Size of the telemetry packets sent from main to aux. The size of the
     * Arduino serial rx/tx buffers must be >= this number. The buffer size is
     * 64 by default and may need to be modified in the Serial source code.
     */
    #define TELEM_PACKET_SIZE 128
#else
    #include <cstdint>
#endif

/**
 * Cause of a state machine transition, used in debugging.
 */
typedef enum Reason : uint8_t
{
    CONDITION,
    TIMEOUT
} Reason_t;

/**
 * Determines where time stands relative to an event window. Should be followed
 * by an EVENT_WINDOW_EVAL().
 *
 * @param   k_t      Current time.
 * @param   k_t_low  Lower event window bound. <= 0 indicates no bound.
 * @param   k_t_high Upper event window bound. <= 0 indicates no bound.
 */
#define EVENT_WINDOW_INIT(k_t, k_t_low, k_t_high)                              \
    float _t_since_liftoff = k_t;                                              \
    bool _grace_period_over = _t_since_liftoff >= k_t_low;                     \
    bool _timed_out = k_t_high > 0 && _t_since_liftoff >= k_t_high;            \
    Reason_t _reason;

/**
 * Evaluates whether or not an event should trigger. Should be preceeded by an
 * EVENT_WINDOW_INIT().
 *
 * @param   k_conds_ok If time-invariant trigger conditions are met, i.e. those
 *                     conditions not related to the detection window.
 *
 * @ret     If the event should trigger.
 */
#define EVENT_WINDOW_EVAL(k_conds_ok)                                          \
    ((_reason = (_timed_out ? Reason_t::TIMEOUT : Reason_t::CONDITION)) ||     \
     (_grace_period_over && (_timed_out || k_conds_ok)))

/**
 * Gets a string representation of an event detection reason. Should be
 * preceeded by an EVENT_WINDOW_EVAL() that evaluated to true.
 */
#define EVENT_WINDOW_REASON                                                    \
    (_reason == Reason_t::CONDITION ? "CONDITION" : "TIMEOUT")

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

    // Timestamps for flight events.
    float t_liftoff;
    float t_burnout;
    float t_canards;
    float t_drogue;
    float t_main;

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
    float gyro_r;
    float gyro_p;
    float gyro_y;

    // Current state of the flight computer.
    VehicleState_t state;

} MainStateVector_t;

/**
 * State vector for auxiliary flight computer.
 */
typedef struct AuxStateVector
{

} AuxStateVector_t;

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
#ifndef FF
    extern Threads::Mutex g_dio_lock;
#endif

/**
 * Tokens sent between flight computers.
 */
const token_t FNW_TOKEN_NIL = 0x00; // Null token.
const token_t FNW_TOKEN_AOK = 0xAA; // Something went OK.
const token_t FNW_TOKEN_ERR = 0xBB; // Something went wrong.

/**
 * LED pulse configuration for component LEDs. Written once by main thread after
 * system startup, read many times by LED pulse thread.
 */
extern PulseLEDs_t g_led_pulse_conf;

/**
 * ID of LED pulse thread created during startup.
 */
extern int32_t pulse_thread_id;

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
 */
double time_s();

/**
 * Pulses LEDs to indicate system status. In general, the LEDs being pulsed will
 * correspond to system components which successfully initialized.
 *
 * @param   k_conf Pulse configuration.
 */
void pulse_leds(const PulseLEDs_t k_conf);

#endif
