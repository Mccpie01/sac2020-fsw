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
 * @file      sac2020_main.ino
 * @purpose   Arduino sketch for the main flight computer node, which manages
 *            the rocket's state, navigation, and recovery.
 * @author    Stefan deBruyn
 * @updated   3/1/2020
 */

#ifdef FF
    #include "ff_arduino_harness.hpp"
    #include <string.h>
#else
    #include <SPI.h>
    #include <Servo.h>
    #include <TeensyThreads.h>
    #include <Wire.h>

    #include "Adafruit_BLE.h"
    #include "Adafruit_BluefruitLE_UART.h"
    #include "sac2020_anthem.h"
    #include "sac2020_baro.h"
    #include "sac2020_imu.h"
#endif

#include <math.h>
#include "photic.h"
#include "sac2020_lib.h"

#include "sac2020_main_pins.h"
#include "sac2020_profile.h"

/**
 * Telemetry output. This output goes to the terminal in FF (and supports FF
 * formatting) and goes to DEBUG_SERIAL in Teensyduino. FF formatting
 * metacharacters are automatically cleaned from debug strings if not in FF.
 */
#ifdef FF
    #define TELEM(data, ...)                                                   \
    {                                                                          \
        ff::out("[#b$msonder#r] ");                                            \
        ff::out(data, ##__VA_ARGS__);                                          \
        ff::out("\n");                                                         \
    }
#else
    #define TELEM(data, ...)                                                   \
    {                                                                          \
        const char data_raw[] = data;                                          \
        char* data_san = sanitize_ff_fmt(data_raw, sizeof(data_raw));          \
        DEBUG_SERIAL.printf(data_san, ##__VA_ARGS__);                          \
        DEBUG_SERIAL.printf("\n");                                             \
        if (g_ble_active)                                                      \
        {                                                                      \
            BLE_SERIAL.printf(data_san, ##__VA_ARGS__);                        \
            BLE_SERIAL.printf("\n");                                           \
        }                                                                      \
    }
#endif

/**
 * Flashes the LED controller some number of times to indicate something to
 * the flight computer operator.
 *
 * @param   k Number of times to flash.
 */
#define EVENT_FLASH(k)                                                         \
{                                                                              \
    for (uint32_t i = 0; i < k; i++)                                           \
    {                                                                          \
        g_ledc->raise_all();                                                   \
        delay(125);                                                            \
        g_ledc->lower_all();                                                   \
        delay(125);                                                            \
    }                                                                          \
}

/******************************** CONFIGURATION ********************************/

/**
 * Depth of Kalman gain calculation.
 */
#define KGAIN_CALC_DEPTH 50

/**
 * Number of seconds that must elapse before detecting liftoff via
 * accelerometer.
 *
 * Note: this is always 0 since the grace period was made Bluetooth-commanded.
 */
#define NO_LIFTOFF_GRACE_PERIOD_S 0

/**
 * Number of pressure readings taken on startup to estimate launchpad altitude.
 */
#define LAUNCHPAD_ALTITUDE_EST_READINGS 1000

/**
 * The index of imu::Vector<3> corresponding to the vertical direction. In the
 * case of a BNO055 IMU calibrated flat on a table, this is the positive z axis.
 */
#define VERTICAL_AXIS_VECTOR_IDX 2

/**
 * Command received over BLE that prompts FC startup.
 */
#define CMD_STARTUP "go"

/**
 * Command received over BLE that enables liftoff detection.
 */
#define CMD_GOTIME "321"

/********************************* STATE MACROS *******************************/

/**
 * Shortcuts to members of the state vector.
 */
#define SV_TIME        g_statevec.time
#define SV_ALTITUDE    g_statevec.altitude
#define SV_VELOCITY    g_statevec.velocity
#define SV_ACCEL       g_statevec.acceleration
#define SV_ACCEL_VERT  g_statevec.accel_vertical
#define SV_PRESSURE    g_statevec.pressure
#define SV_TEMPERATURE g_statevec.temperature
#define SV_BARO_ALT    g_statevec.baro_altitude
#define SV_ACCEL_X     g_statevec.accel_x
#define SV_ACCEL_Y     g_statevec.accel_y
#define SV_ACCEL_Z     g_statevec.accel_z
#define SV_GYRO_X      g_statevec.gyro_x
#define SV_GYRO_Y      g_statevec.gyro_y
#define SV_GYRO_Z      g_statevec.gyro_z
#define SV_QUAT_W      g_statevec.quat_w
#define SV_QUAT_X      g_statevec.quat_x
#define SV_QUAT_Y      g_statevec.quat_y
#define SV_QUAT_Z      g_statevec.quat_z
#define SV_LP_ALT      g_statevec.launchpad_altitude
#define SV_IMU_TEMP    g_statevec.imu_temp
#define SV_STATE       g_statevec.state

/*********************************** GLOBALS **********************************/

/**
 * Hardware wrappers, configured during startup.
 */
photic::Imu* g_imu;
photic::Barometer* g_baro;
#ifndef FF
    Servo g_servo1;
    Servo g_servo2;
    Adafruit_BluefruitLE_UART g_ble(Serial3, PIN_BLE_MOD, PIN_BLE_CTS,
                                    PIN_BLE_RTS);
#endif

/**
 * Component statuses, set during startup.
 */
Status_t g_baro_status  = Status_t::OFFLINE; // BMP085 barometer.
Status_t g_imu_status   = Status_t::OFFLINE; // BNO055 IMU.
Status_t g_ble_status   = Status_t::OFFLINE; // Bluetooth module.
Status_t g_pyro1_status = Status_t::OFFLINE; // Pyro 1.
Status_t g_pyro2_status = Status_t::OFFLINE; // Pyro 2.
Status_t g_fnw_status   = Status_t::OFFLINE; // Flight computer network.

/**
 * Kalman filter for state estimation and metronome controlling its frequency.
 */
photic::KalmanFilter g_kf;
photic::Metronome g_mtr_kf(10);

/**
 * Whether or not the final state vector indicating the vehicle is in
 * VehicleState_t::CONCLUDE has been sent to the aux computer.
 */
bool g_sent_conclude_msg = false;
/**
 * Variance in altitude and acceleration readings, sampled during startup.
 */
float g_pos_variance;
float g_acc_variance;

/**
 * History for liftoff detection and metronome controlling its frequency.
 * Liftoff is defined as a 1-second rolling average of vertical acceleration
 * readings meeting or exceeding LIFTOFF_ACCEL_TRIGGER_MPSSQ.
 */
photic::history<float> g_hist_lodet(10);
photic::Metronome g_mtr_lodet(10);

/**
 * History for burnout detection and metronome controlling its frequency.
 * Burnout is defined as a 1-second rolling average of vertical acceleration
 * readings within BURNOUT_ACCEL_TRIGGER_NEGL_MPSSQ of 1 G.
 */
photic::history<float> g_hist_bodet(10);
photic::Metronome g_mtr_bodet(10);

/**
 * History for apogee detection and metronome controlling its frequency. Apogee
 * is defined as a 1-second rolling average of vertical velocity estimates that
 * are negative.
 */
photic::history<float> g_hist_apdet(10);
photic::Metronome g_mtr_apdet(10);

/**
 * Vehicle state vector. Zeroed on startup. Contains the symbolic state of the
 * vehicle, i.e. if it's in powered flight, falling, etc.
 */
MainStateVector_t g_statevec;

/**
 * Controller for status LEDs.
 */
LEDController* g_ledc = nullptr;

/**
 * Whether or not the BLE is currently being used.
 */
bool g_ble_active = false;

/**
 * Whether or not I am waiting for an ack from the aux computer before sending
 * another telemetry packet.
 */
bool g_aux_ack_pending = false;

/**
 * ID of anthem thread.
 */
int32_t g_anthem_thread_id = -1;

/**
 * Time of liftoff detection.
 */
float g_t_liftoff = -1;

/********************************* FUNCTIONS **********************************/

/**
 * Receives a string of some expected size from the BLE.
 *
 * Note: BLE recv buffer is flushed after receipt.
 *
 * @param   k_exp Expected string.
 * @param   k_len Size of expected string. Method blocks until this many
                  bytes are received from the BLE.
 *
 * @ret     Whether or not the received string matched the expected.
 */
bool ble_recv(const char k_exp[], size_t k_len)
{
    size_t recv_len = 0;
    char recv_buf[k_len];
    memset(recv_buf, 0, k_len);

    // Read in data until the expected amount is received.
    while (recv_len < k_len)
    {
        g_ledc->run(time_s());
        if (g_ble.available())
        {
            recv_buf[recv_len++] = (char) g_ble.read();
        }
    }

    delay(100);
    g_ble.flush();

    // Return if the data matched the expected. Expected string is terminated
    // by a null sentinel, but received string is terminated by a newline, so
    // exclude the last character from the comparison.
    return memcmp(recv_buf, k_exp, k_len - 1) == 0;
}

/**
 * Initializes the BMP085 barometer.
 */
void init_baro()
{
    // Create barometer device wrapper based on runtime environment.
    g_baro =
    #ifdef FF
        new VirtualBarometer();
    #else
        new Sac2020Barometer();
    #endif

    // Attempt contact with barometer. Abort on failure.
    TELEM("Contacting barometer...");
    if (!g_baro->init())
    {
        fault(PIN_LED_BARO_FAULT, "ERROR :: BMP085 INIT FAILED", g_baro_status,
              g_ledc);
        return;
    }

    // Signal operator sampling is about to start.
    EVENT_FLASH(8);
    TELEM("Sampling launchpad altitude...");

    // Barometer status LED flashes during sampling.
    g_ledc->flash(PIN_LED_BARO_FAULT);

    // Estimate initial altitude via barometer. Measure variance in altitude
    // readings for computing a Kalman gain.
    photic::history<float> alts(LAUNCHPAD_ALTITUDE_EST_READINGS);
    for (std::size_t i = 0; i < LAUNCHPAD_ALTITUDE_EST_READINGS; i++)
    {
        g_baro->update();
        alts.add(g_baro->data().altitude);
        g_ledc->run(time_s(), PIN_LED_BARO_FAULT);
    }

    // Compute variance and set launchpad altitude.
    g_pos_variance = alts.stdev() * alts.stdev();
    SV_ALTITUDE = alts.mean();
    SV_LP_ALT = SV_ALTITUDE;

    // Bring up barometer LED again and reset all LEDs.
    g_ledc->solid(PIN_LED_BARO_FAULT);
    g_ledc->lower_all();

    // Barometer is good to go.
    TELEM("Launchpad altitude set to $y%.2f#r m", SV_ALTITUDE);
    TELEM("Sampled altimeter variance: $y%.4f", g_pos_variance);
    g_baro_status = Status_t::ONLINE;
}

/**
 * Initializes the BNO055 IMU.
 */
void init_imu()
{
    // Create IMU device wrapper based on runtime environment.
    g_imu =
    #ifdef FF
        new VirtualImu();
    #else
        new Sac2020Imu();
    #endif

    // Attempt contact with IMU. Abort on failure.
    TELEM("Contacting IMU...");
    if (!g_imu->init())
    {
        fault(PIN_LED_IMU_FAULT, "ERROR :: BNO055 INIT FAILED", g_imu_status,
              g_ledc);
        return;
    }

#ifndef FF
    // Perform IMU sensor calibration.
    TELEM("Beginning IMU calibration...");

    // Last status read for each sensor.
    uint8_t system_status = 0;
    uint8_t gyro_status   = 0;
    uint8_t accel_status  = 0;
    uint8_t mag_status    = 0;

    // Whether or not each sensor is fully calibrated.
    bool system_calib = false;
    bool gyro_calib   = false;
    bool accel_calib  = false;
    bool mag_calib    = false;

    // Status returned by BNO055 when component is fully calibrated.
    static const uint8_t FULL_CALIB_STATUS = 3;

    // Pins used to convey calibration events.
    const std::vector<uint8_t> calib_pins =
    {
        PIN_LED_PYRO1_FAULT, // IMU system.
        PIN_LED_PYRO2_FAULT, // Gyroscope.
        PIN_LED_BLE_FAULT,   // Accelerometer.
        PIN_LED_FNW_FAULT    // Magnetometer.
    };

    // While calibration is not complete...
    do
    {
        // Get calibration statuses.
        Sac2020Imu* imu_cast = (Sac2020Imu*) g_imu;
        imu_cast->get_calib(&system_status, &gyro_status, &accel_status,
                            &mag_status);
        // Mark system as calibrated if not.
        if (system_status == FULL_CALIB_STATUS && !system_calib)
        {
            TELEM("IMU system calibrated");
            digitalWrite(calib_pins[0], HIGH);
            system_calib = true;
        }
        // Mark gyroscope as calibrated if not.
        if (gyro_status == FULL_CALIB_STATUS && !gyro_calib)
        {
            TELEM("IMU gyroscope calibrated");
            digitalWrite(calib_pins[1], HIGH);
            gyro_calib = true;
        }
        // Mark accelerometer as calibrated if not.
        if (accel_status == FULL_CALIB_STATUS && !accel_calib)
        {
            TELEM("IMU accelerometer calibrated");
            digitalWrite(calib_pins[2], HIGH);
            accel_calib = true;
        }
        // Mark magnetometer as calibrated if not.
        if (mag_status == FULL_CALIB_STATUS && !mag_calib)
        {
            TELEM("IMU magnetometer calibrated");
            digitalWrite(calib_pins[3], HIGH);
            mag_calib = true;
        }
    } while (!system_calib || !gyro_calib || !accel_calib || !mag_calib);

    // Signal to operator that calibration is complete and give them a moment
    // to set the computer down before variance sampling starts.
    EVENT_FLASH(8);
    TELEM("IMU calibration complete. Variance profile will begin shortly...");
    delay(5000);
#endif

    // Flash IMU LED during sampling.
    TELEM("Sampling accelerometer variance...");
    g_ledc->flash(PIN_LED_IMU_FAULT);

    // Measure variance in acceleration readings for later computing a Kalman
    // gain. This is the variance in a single axis, as only a (scalar) component
    // of the measured acceleration vector enters the filter.
    photic::history<float> accs(LAUNCHPAD_ALTITUDE_EST_READINGS);
    for (std::size_t i = 0; i < LAUNCHPAD_ALTITUDE_EST_READINGS; i++)
    {
        g_imu->update();
        // For a BNO055 laying flat on a table, Z will be up, which is the
        // approximate direction of travel and should represent the most
        // significant component of net acceleration.
        accs.add(g_imu->data().accel_z);
        g_ledc->run(time_s(), PIN_LED_IMU_FAULT);
    }
    g_acc_variance = accs.stdev() * accs.stdev();

    // Bring IMU LED back up and reset all LEDs.
    g_ledc->solid(PIN_LED_IMU_FAULT);
    g_ledc->lower_all();

    // IMU is good to go.
    TELEM("Sampled accelerometer variance: $y%.4f", g_acc_variance);
    g_imu_status = Status_t::ONLINE;
}

/**
 * Initializes the canard fin servos.
 */
void init_servos()
{
#ifndef FF
    g_servo1.attach(PIN_SERVO1_PWM);
    g_servo2.attach(PIN_SERVO2_PWM);
#endif
}

/**
 * Initializes the Bluetooth module.
 */
void init_ble()
{
    g_ble_active = true;

    // Attempt contact with BLE. Abort on failure.
    if (!g_ble.begin(false, false))
    {
        fault(PIN_LED_BLE_FAULT, "ERROR :: BLUETOOTH INIT FAILED", g_ble_status,
              g_ledc);
        return;
    }
    g_ble.setMode(BLUEFRUIT_MODE_DATA);

    // Wait for startup command.
    if (!ble_recv(CMD_STARTUP, sizeof(CMD_STARTUP)))
    {
        fault(PIN_LED_BLE_FAULT, "ERROR :: MALFORMED STARTUP COMMAND",
              g_ble_status, g_ledc);
        return;
    }

    // BLE is good to go.
    g_ledc->solid(PIN_LED_BLE_FAULT);
    g_ble_status = Status_t::ONLINE;
}

/**
 * Initializes the pyros. TODO
 */
void init_pyros()
{
    g_ledc->flash(PIN_LED_PYRO1_FAULT);
    g_ledc->flash(PIN_LED_PYRO2_FAULT);
}

/**
 * Updates all sensors and dumps their readings into the state vector.
 */
void update_sensors()
{
    // Run device wrappers.
    g_imu->update();
    g_baro->update();

    // Dump sensor readings into state vector.
    SV_PRESSURE = g_baro->data().pressure;
    SV_TEMPERATURE = g_baro->data().temperature;
    SV_BARO_ALT = g_baro->data().altitude;
    SV_ACCEL_X = g_imu->data().accel_x;
    SV_ACCEL_Y = g_imu->data().accel_y;
    SV_ACCEL_Z = g_imu->data().accel_z;
    SV_GYRO_X = g_imu->data().gyro_z;
    SV_GYRO_Y = g_imu->data().gyro_y;
    SV_GYRO_Z = g_imu->data().gyro_x;

#ifndef FF
    // Read IMU temperature.
    Sac2020Imu* imu_cast = (Sac2020Imu*) g_imu;
    SV_IMU_TEMP = imu_cast->get_temp();

    // Determine vertical component of acceleration relative to launchpad
    // using sensed orientation.
    imu::Quaternion orientation = imu_cast->quat();
    imu::Vector<3> accel_rocket(SV_ACCEL_X, SV_ACCEL_Y, SV_ACCEL_Z);
    imu::Vector<3> accel_world = orientation.rotateVector(accel_rocket);
    SV_ACCEL_VERT = accel_world[VERTICAL_AXIS_VECTOR_IDX];

    // Put quaternion orientation into state vector.
    SV_QUAT_W = orientation.w();
    SV_QUAT_X = orientation.x();
    SV_QUAT_Y = orientation.y();
    SV_QUAT_Z = orientation.z();
#else
    // In Flight Factory, just use the vertical reading, since the flight model
    // is only 1 DoF.
    SV_ACCEL_VERT = SV_ACCEL_Z;
#endif
}

/**
 * Deploys forward canard fins. TODO
 */
void deploy_canards()
{

}

/**
 * Deploy the drogue parachute. TODO
 */
void deploy_drogue()
{

}

/**
 * Deploy the main parachute. TODO
 */
void deploy_main()
{

}

/**
 * Gets the seconds elapsed since liftoff. If liftoff has not been detected,
 * behavior is undefined.
 */
inline double time_liftoff_s()
{
    return time_s() - g_t_liftoff;
}

/**
 * Gets the magnitude of the currently sensed linear acceleration vector.
 */
inline float accel_magnitude()
{
    return sqrt(SV_ACCEL_X * SV_ACCEL_X +
                SV_ACCEL_Y * SV_ACCEL_Y +
                SV_ACCEL_Z * SV_ACCEL_Z);
}

/*********************************** SETUP ************************************/

void setup()
{
    // In case we are debugging over serial, give operator a moment to open
    // serial monitor.
    DEBUG_SERIAL.begin(115200);
    BLE_SERIAL.begin(115200);
    delay(3000);

    // Zero the state vector.
    memset(&g_statevec, 0, sizeof(MainStateVector_t));
    SV_STATE = VehicleState_t::PRELTOFF;

    // Set up status LED controller.
    g_ledc = new LEDController({PIN_LED_SYS_FAULT,
                                PIN_LED_IMU_FAULT,
                                PIN_LED_BLE_FAULT,
                                PIN_LED_PYRO1_FAULT,
                                PIN_LED_PYRO2_FAULT,
                                PIN_LED_FNW_FAULT,
                                PIN_LED_BARO_FAULT});

    // Signal to operator that computer is in startup.
    EVENT_FLASH(8);

    // Begin by initializing the BLE and waiting for the startup command
    // from the field operator via Bluefruit app.
    init_ble();
    TELEM("Vehicle is in startup...");

#ifdef USING_FNW
    TELEM("Contacting aux node...");

    // Send handshake packet to aux.
    FNW_SERIAL.begin(FNW_BAUD);
    uint8_t packet_tx[FNW_PACKET_SIZE];
    packet_tx[0] = FNW_TOKEN_HSH;
    FNW_SERIAL.write(packet_tx, FNW_PACKET_SIZE);

    // Wait for echo from aux.
    float t_handshake_start = time_s();
    while (!FNW_PACKET_AVAILABLE &&
           time_s() - t_handshake_start < FNW_CONN_TIMEOUT);

    if (FNW_PACKET_AVAILABLE)
    {
        // Read in response packet and check tokens.
        uint8_t packet_rx[FNW_PACKET_SIZE];
        memset(packet_rx, FNW_TOKEN_ERR, FNW_PACKET_SIZE);
        FNW_SERIAL.readBytes(packet_rx, FNW_PACKET_SIZE);
        bool ok = true;

        for (size_t i = 0; i < FNW_PACKET_SIZE; i++)
        {
            // If there is a byte mismatch in the echoed packet, something
            // went wrong.
            if (packet_rx[i] != packet_tx[i])
            {
                ok = false;
                fault(PIN_LED_FNW_FAULT, "ERROR :: BAD HANDSHAKE WITH AUX",
                      g_fnw_status, g_ledc);
                break;
            }
        }

        // Otherwise, all is well.
        if (ok)
        {
            TELEM("Handshake exchanged with aux!");
            g_fnw_status = Status_t::ONLINE;
        }
    }
    else
    {
        // Handshake timed out--something is wrong.
        fault(PIN_LED_FNW_FAULT, "ERROR :: FAILED TO CONTACT AUX", g_fnw_status,
              g_ledc);
    }
#endif

    // Initialize subsystems.
    init_pyros();
    init_imu();
    init_baro();
    init_servos();

    // Set up Kalman filter.
    g_kf.set_delta_t(g_mtr_kf.period());
    g_kf.set_sensor_variance(g_pos_variance, g_acc_variance);
    g_kf.set_initial_estimate(SV_LP_ALT, 0, 0);
    g_kf.compute_kg(KGAIN_CALC_DEPTH);

    // Determine if everything initialized correctly.
    bool ok = g_baro_status  == Status_t::ONLINE &&
              g_imu_status   == Status_t::ONLINE &&
              g_pyro1_status == Status_t::ONLINE &&
              g_pyro2_status == Status_t::ONLINE;
    if (!ok)
    {
        g_ledc->flash(PIN_LED_SYS_FAULT);
    }

    // Raise all LEDs to flip internal flags in controller.
    g_ledc->raise_all();

#ifndef FF
    // ввысь, товарищ.
    g_anthem_thread_id = threads.addThread(anthem, nullptr);
#endif

    // Wait for operator signal to proceed to liftoff detection.
    TELEM("Setup complete. Enter \"%s\" to allow liftoff detection...",
          CMD_GOTIME);
    while (!ble_recv(CMD_GOTIME, sizeof(CMD_GOTIME)));
    TELEM("Let's jam!");
    delay(1000);

    // End connection with BLE.
    g_ble.end();

#ifndef FF
    // Kill the anthem thread, which doesn't place nice with the IMU.
    threads.kill(g_anthem_thread_id);
    delay(100); // Thread dies on the next time slice, so wait a bit for that.
#endif

    // Flag that we no longer want to pipe telemetry to the BLE.
    // It was observed that continuing to write to its serial after
    // ending the session would cause anomalous crashes.
    g_ble_active = false;

#ifdef FF
    // If using FF, open telemetry pipes.
    ff::topen("gt_alt");
    ff::topen("kf_alt");
    ff::topen("gt_vel");
    ff::topen("kf_vel");
#endif
}

/******************************** STATE MACHINE *******************************/

/**
 * Updates the state machine for the entire mission.
 */
void run_state_machine()
{
    // If in pre-liftoff state, monitor acceleration for a spike characteristic
    // of liftoff. We do this with repeated returns in loop() as opposed to a
    // blocking loop in setup() for compliance with Flight Factory.
    if (SV_STATE == VehicleState_t::PRELTOFF)
    {
        // Add vertical acceleration to the rolling average.
        if (g_mtr_lodet.poll(SV_TIME))
        {
            g_hist_lodet.add(SV_ACCEL_VERT);
        }

        // Run status LED controller.
        g_ledc->run(SV_TIME);

        // Evaluate transition conditions.
        EVENT_WINDOW_INIT(SV_TIME, NO_LIFTOFF_GRACE_PERIOD_S, -1);
        bool liftoff_conds_met =
                g_hist_lodet.at_capacity() &&
                g_hist_lodet.mean() >= LIFTOFF_ACCEL_TRIGGER_MPSSQ;

        // Evaluate transition conditions.
        if (EVENT_WINDOW_EVAL(liftoff_conds_met))
        {
            TELEM("Event $b%-7s#r at t+$y%06.2f#r by $r%-9s#r; acc=$y%.2f#r",
                  "LIFTOFF", SV_TIME, EVENT_WINDOW_REASON, g_hist_lodet.mean());
            g_t_liftoff = SV_TIME;
            SV_STATE = VehicleState_t::PWFLIGHT;

            // Lower all LEDs to conserve power.
            g_ledc->lower_all();
        }
    }
    // If in powered flight, we're waiting for a timeout or sufficiently low
    // sustained acceleration to transition to cruising state.
    else if (SV_STATE == VehicleState_t::PWFLIGHT)
    {
        // Add vertical acceleration to the rolling average.
        if (g_mtr_bodet.poll(SV_TIME))
        {
            g_hist_bodet.add(SV_ACCEL_VERT);
        }

        // Evaluate transition conditions.
        EVENT_WINDOW_INIT(time_liftoff_s(), EVENT_BURNOUT_T_LOW_S,
                          EVENT_BURNOUT_T_HIGH_S);
        bool burnout_conds_met = g_hist_bodet.at_capacity() &&
                                 g_hist_bodet.mean() < 0;
        if (EVENT_WINDOW_EVAL(burnout_conds_met))
        {
            TELEM("Event $b%-7s#r at t+$y%06.2f#r by $r%-9s#r; acc=$y%.2f#r",
                  "BURNOUT", SV_TIME, EVENT_WINDOW_REASON, g_hist_bodet.mean());
            SV_STATE = VehicleState_t::CRUISING;
        }
    }
    // If cruising without canards deployed, we're waiting for a timeout or
    // sufficiently high altitude estimate to trigger deployment.
    else if (SV_STATE == VehicleState_t::CRUISING)
    {
        // Evaluate transition conditions.
        EVENT_WINDOW_INIT(time_liftoff_s(), EVENT_CANARDS_T_LOW_S,
                          EVENT_CANARDS_T_HIGH_S);
        bool canard_conds_met =
                (SV_ALTITUDE - SV_LP_ALT) >= CANARD_DEPLOYMENT_ALTITUDE_M;
        if (EVENT_WINDOW_EVAL(canard_conds_met))
        {
            TELEM("Event $b%-7s#r at t+$y%06.2f#r by $r%-9s#r; hgt=$y%.2f#r",
                  "CANARDS", SV_TIME, EVENT_WINDOW_REASON,
                  SV_ALTITUDE - SV_LP_ALT);
            SV_STATE = VehicleState_t::CRSCANRD;
            deploy_canards();
        }
    }
    // If cruising with canards deployed, we're waiting for a timeout or
    // sustained negative velocity estimate to trigger apogee/drogue deployment.
    else if (SV_STATE == VehicleState::CRSCANRD)
    {
        // Add velocity to the rolling average.
        if (g_mtr_apdet.poll(SV_TIME))
        {
            g_hist_apdet.add(SV_VELOCITY);
        }

        // Evaluate transition conditions.
        EVENT_WINDOW_INIT(time_liftoff_s(), EVENT_APOGEE_T_LOW_S,
                          EVENT_APOGEE_T_HIGH_S);
        bool apogee_conds_met = g_hist_apdet.at_capacity() &&
                                g_hist_apdet.mean() < 0;
        if (EVENT_WINDOW_EVAL(apogee_conds_met))
        {
            TELEM("Event $b%-7s#r at t+$y%06.2f#r by $r%-9s#r; vel=$y%.2f#r",
                  "DROGUE", SV_TIME, EVENT_WINDOW_REASON, g_hist_apdet.mean());
            SV_STATE = VehicleState_t::FALLDROG;
            deploy_drogue();
        }
    }
    // If falling with the drogue deployed, we're waiting for a timeout or
    // sufficiently low altitude estimate to trigger main deployment.
    else if (SV_STATE == VehicleState_t::FALLDROG)
    {
        // Evaluate transition conditions.
        EVENT_WINDOW_INIT(time_liftoff_s(), EVENT_MAIN_T_LOW_S,
                          EVENT_MAIN_T_HIGH_S);
        bool main_conds_met =
                (SV_ALTITUDE - SV_LP_ALT) <= MAIN_DEPLOYMENT_ALTITUDE_M;
        if (EVENT_WINDOW_EVAL(main_conds_met))
        {
            TELEM("Event $b%-7s#r at t+$y%06.2f#r by $r%-9s#r; hgt=$y%.2f#r",
                  "MAIN", SV_TIME, EVENT_WINDOW_REASON,
                  SV_ALTITUDE - SV_LP_ALT);
            SV_STATE = VehicleState_t::FALLMAIN;
            deploy_main();
        }
    }
    // If falling with the main deployed, we're waiting for
    // EVENT_CONCLUDE_T_HIGH_S seconds since liftoff to pass before concluding
    // the mission and ceasing meaningful operation.
    else if (SV_STATE == VehicleState::FALLMAIN)
    {
        // Evaluate transition conditions.
        if (time_liftoff_s() >= EVENT_CONCLUDE_T_HIGH_S)
        {
            TELEM("Event $%-7s#r at t+$y%06.2f#r", "CONCLUDE", SV_TIME);
            SV_STATE = VehicleState::CONCLUDE;

            // Raise all LEDs as indication to recovery team.
            g_ledc->raise_all();
        }
    }
    // Otherwise, we're in VehicleState_t::CONCLUDE.
}

/************************************ LOOP ************************************/

void loop()
{
    // If the conclude message has been sent to the aux computer, the mission is
    // over.
    if (g_sent_conclude_msg)
    {
        return;
    }

    // Timestamp this system iteration.
    SV_TIME = time_s();

    // Update sensors and dump their readings into the state vector.
    update_sensors();

    // Perform state transitions.
    run_state_machine();

    // The Kalman filter updates our estimate of the rocket's kinetic state.
    // This runs in every state except pre-liftoff.
    bool do_kf = SV_STATE != VehicleState_t::PRELTOFF;
    if (do_kf && g_mtr_kf.poll(SV_TIME))
    {
        // Huge drops in the barometer's altitude reading have been noticed in
        // the moments following liftoff. We believe this to be caused by the
        // mass of inert air in the avionics bay hitting the sensor as it
        // suddenly accelerates. To combat this, we floor the altitude
        // observation at the estimated launchpad altitude so that the filter
        // does not think the rocket is traveling in the direction opposite of
        // the measured acceleration.
        float alt_obs = SV_BARO_ALT < SV_LP_ALT ? SV_LP_ALT : SV_BARO_ALT;

        // Run the filter and place the new estimate into the state vector.
        photic::matrix kinst = g_kf.filter(alt_obs, SV_ACCEL_VERT);
        SV_ALTITUDE = kinst[0][0];
        SV_VELOCITY = kinst[1][0];
        SV_ACCEL = kinst[2][0];

    #ifdef FF
        // Log flight data in FF for analysis.
        aimbot::state_t true_state = SIM.get_rocket_state();
        ff::tout("gt_alt", SV_TIME, true_state.altitude);
        ff::tout("kf_alt", SV_TIME, SV_ALTITUDE);
        ff::tout("gt_vel", SV_TIME, true_state.velocity);
        ff::tout("kf_vel", SV_TIME, SV_VELOCITY);
    #endif
    }

    // Transmit state vector to the aux computer over FNW_SERIAL for SD backup
    // and transmission to ground station. This runs in every state except
    // pre-liftoff.
    bool do_telemtx = SV_STATE != VehicleState_t::PRELTOFF;
    if (do_telemtx)
    {
    #ifdef USING_FNW
        // OK to send another packet.
        if (!g_aux_ack_pending)
        {
            // Pack metadata token and state vector into a buffer and send to
            // aux.
            uint8_t packet[FNW_PACKET_SIZE];
            packet[0] = FNW_TOKEN_VEC;
            memcpy(packet + 1, &g_statevec, sizeof(g_statevec));
            // Transmit state vector.
            FNW_SERIAL.write(packet, FNW_PACKET_SIZE);

            // If the mission is over, this telemetry transmission becomes the
            // last action the main computer performs before ceasing to loop.
            if (SV_STATE == VehicleState_t::CONCLUDE)
            {
                g_sent_conclude_msg = true;
                return;
            }

            // Block further packet TXs until aux confirms receipt.
            g_aux_ack_pending = true;
        }
        // Waiting for confirmation of last packet receipt--check for ack.
        else
        {
            // If a packet is available, it's probably the ack. Currently, the
            // ack is just an echo because aux has no information that main
            // could possibly want. For the fastest exchange possible, just
            // empty the RX buffer w/o checking the contents.
            if (FNW_PACKET_AVAILABLE)
            {
                uint8_t packet[FNW_PACKET_SIZE];
                FNW_SERIAL.readBytes(packet, FNW_PACKET_SIZE);
                g_aux_ack_pending = false;
            }
        }
    #endif
    }
}
