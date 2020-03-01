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
 * @file      sac2020_lib.h
 * @purpose   Supporting code that is common to both main and auxiliary flight
 *            computer nodes.
 * @author    Stefan deBruyn
 * @updated   2/22/2020
 */

#ifndef SAC2020_LIB_H
#define SAC2020_LIB_H

#include <vector>

#include "sac2020_state.h"

#ifndef FF
    /**
     * Whether or not computer is being tested on the ground, inert, outside
     * of a rocket. THIS SHOULD BE COMMENTED OUT ON LAUNCH DAY.
     */
    // #define GROUND_TEST

    /**
     * Toggles debug prints to Serial.
     */
    #define DEBUG_SERIAL Serial

    /**
     * Flight network communication line (serial line between main and aux FCs).
     */
    #define FNW_SERIAL Serial1

    /**
     * Serial line used to communicate with Bluetooth module.
     */
    #define BLE_SERIAL Serial3

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
    #define FNW_PACKET_SIZE 127

    /**
     * Maximum time to wait for a handshake when establishing FNW connection.
     */
    #define FNW_CONN_TIMEOUT 5

    /**
     * Whether or not a packet was received over FNW.
     */
    #define FNW_PACKET_AVAILABLE (FNW_SERIAL.available() == FNW_PACKET_SIZE)
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
 * Tokens are 1-byte pieces of metadata sent between flight computers.
 */
typedef uint8_t token_t;

/********************************* GLOBALS ************************************/

/**
 * Tokens sent between flight computers.
 */
const token_t FNW_TOKEN_NIL = 0x00; // Null token.
const token_t FNW_TOKEN_AOK = 0xAA; // Something went OK.
const token_t FNW_TOKEN_ERR = 0xBB; // Something went wrong.
const token_t FNW_TOKEN_HSH = 0xCC; // Handshake payload follows.
const token_t FNW_TOKEN_VEC = 0xDD; // State vector payload follows.

/******************************** PROTOTYPES **********************************/

/**
 * Controls the status LEDs on the flight computer board.
 */
class LEDController final
{
public:
    /**
     * Configures a new LED controller.
     *
     * @param   k_pins List of LED pin numbers. These will be set to OUTPUT
     *                 mode, and stay solid by default.
     */
    LEDController(std::vector<uint8_t> k_pins);

    /**
     * Makes a pin flash while the controller is run.
     *
     * @param   k_pin Pin number.
     */
    void flash(uint8_t k_pin);

    /**
     * Makes a pin solid while the controller is run.
     *
     * @param   k_pin Pin number.
     */
    void solid(uint8_t k_pin);

    /**
     * Runs the controller by blinking LEDs as appropriate. Should be called
     * at a relatively high frequency.
     *
     * @param   k_t   Current time in seconds.
     * @param   k_pin Pin to run. If -1, all pins run.
     */
    void run(float k_t, int32_t k_pin=-1);

    /**
     * Lowers all LEDs.
     */
    void lower_all();

    /**
     * Raises all LEDs.
     */
    void raise_all();

private:
    /**
     * Number of state changes (either HIGH -> LOW or LOW -> HIGH) to make on
     * a flashing LED per second.
     */
    static constexpr uint8_t PULSES = 3;

    /**
     * State of a single pin.
     */
    typedef struct Pin
    {
        uint8_t num;        // Pin number.
        bool high;          // Whether or not LED is on.
        bool flash;         // True if flashing, false if solid.
        float t_flash_last; // Time of last flash. -1 if none.
    } Pin_t;

    /**
     * Pin states, updated as the controller is run.
     */
    std::vector<Pin_t> m_pins;
};

/**
 * Raises a flight computer fault for a particular component.
 *
 * @param   k_pin  Fault indicator LED pin.
 * @param   k_msg  Fault message, printed to DEBUG_SERIAL if defined.
 * @param   k_stat Global status for component to be set to Status_t::FAULT.
 * @param   k_ledc LED controller.
 */
void fault(uint8_t k_pin, const char* k_msg, Status_t& k_stat,
           LEDController* k_ledc);

/**
 * Gets the time according to Arduino's millis(), reinterpreted as seconds.
 *
 * @ret     Current system time in seconds.
 */
double time_s();

/**
 * Sanitizes a string literal with FF format codes in it. Operates on the same
 * pointer and so destroys the results of previous calls. Supports a maximum
 * string size of 128 characters (including null terminator).
 *
 * @param   k_str String to sanitize.
 * @param   k_len String length.
 *
 * @ret     Sanitized string.
 */
char* sanitize_ff_fmt(const char k_str[], uint32_t k_len);

#endif
