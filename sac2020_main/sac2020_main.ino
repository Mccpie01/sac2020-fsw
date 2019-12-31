#include <photic.h>
#include <SPI.h>
#include <Servo.h>
#include <Wire.h>

#include "sac2020_baro.h"
#include "sac2020_imu.h"
#include "sac2020_main_pins.h"
#include "sac2020_lib.h"

/******************************** CONFIGURATION ********************************/

/**
 * Depth of Kalman gain calculation.
 */
#define KGAIN_CALC_DEPTH 100
/**
 * Variance in altimeter readings determined offboard. TODO
 */
#define POS_VARIANCE -111111111
/**
 * Variance in accelerometer readings determined offboard. TODO
 */
#define ACC_VARIANCE -222222222
/**
 * Number of seconds that must elapse before detecting liftoff via
 * accelerometer.
 */
#define NO_LIFTOFF_GRACE_PERIOD_S 10 * 60
/**
 * Minimum acceleration rolling average required to declare liftoff and enter
 * powered flight state.
 */
#define LIFTOFF_ACCEL_TRIGGER_MPSSQ 3 * photic::EARTH_SLGRAV_MPSSQ
/**
 * Number of seconds that must elapse after liftoff before acceleration readings
 * may be used to detect burnout.
 */
#define NO_BURNOUT_GRACE_PERIOD_S 5
/**
 * Maximum number of seconds that may elapse after liftoff before automatically
 * declaring burnout, regardless of acceleration readings.
 */
#define BURNOUT_TRIGGER_TIMEOUT_S 8
/**
 * Negligence in burnout detection. In the allotted detection window, if the
 * absolute difference between a rolling average of acceleration readings and
 * 1 G is less than or equal to this number, we exit powered flight state.
 */
#define BURNOUT_ACCEL_TRIGGER_NEGL_MPSSQ 0.15
/**
 * Number of pressure readings taken on startup to estimate launchpad altitude.
 */
#define LAUNCHPAD_ALTITUDE_EST_READINGS 1000
/**
 * Altitude relative to launchpad at which to deploy canards. TODO
 */
#define CANARD_DEPLOYMENT_ALTITUDE_M 100
/**
 * Minimum time that must elapse before altitude-based canard deployments.
 */
#define NO_CANARDS_GRACE_PERIOD_S 10
/**
 * Time after liftoff at which to automatically deploy canards. TODO
 */
#define CANARD_DEPLOYMENT_TIMEOUT_S 20

/********************************* STATE MACROS *******************************/

/**
 * Shortcuts to members of the state vector.
 */
#define SV_TIME         g_statevec.time
#define SV_T_LIFTOFF    g_statevec.t_liftoff
#define SV_T_BURNOUT    g_statevec.t_burnout
#define SV_T_CANARDS    g_statevec.t_canards
#define SV_ALTITUDE     g_statevec.altitude
#define SV_VELOCITY     g_statevec.velocity
#define SV_ACCEL        g_statevec.acceleration
#define SV_PRESSURE     g_statevec.pressure
#define SV_TEMPERATURE  g_statevec.temperature
#define SV_ACCEL_X      g_statevec.accel_x
#define SV_ACCEL_Y      g_statevec.accel_y
#define SV_ACCEL_Z      g_statevec.accel_z
#define SV_GYRO_R       g_statevec.gyro_r
#define SV_GYRO_P       g_statevec.gyro_p
#define SV_GYRO_Y       g_statevec.gyro_y
#define SV_CANARDSDEP   g_statevec.canards_deployed
#define SV_STATE        g_statevec.state
/**
 * Member of state vector containing most recent vertical acceleration reading.
 */
#define VERTICAL_ACCEL  g_statevec.accel_x

/*********************************** GLOBALS **********************************/

/**
 * Hardware wrappers, configured during startup.
 */
Sac2020Imu g_imu;
Sac2020Barometer g_baro;
Servo g_servo1;
Servo g_servo2;

/**
 * Component statuses, set during startup.
 */
Status_t g_baro_status  = Status_t::OFFLINE; // BMP85 barometer.
Status_t g_imu_status   = Status_t::OFFLINE; // BNO055 IMU.
Status_t g_pyro1_status = Status_t::OFFLINE; // Pyro 1.
Status_t g_pyro2_status = Status_t::OFFLINE; // Pyro 2.
Status_t g_fnw_status   = Status_t::OFFLINE; // Flight computer network.

/**
 * Kalman filter for state estimation and metronome controlling its frequency.
 */
photic::KalmanFilter g_kf;
photic::Metronome g_mtr_kf(10);

/**
 * Launchpad altitude, estimated during startup.
 */
double g_x0;

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
 * Metronome controlling the frequency at which state vectors are sent to
 * the aux computer for SD backup and transmission to ground.
 */
photic::Metronome g_mtr_telemtx(10);

/********************************* FUNCTIONS **********************************/

/**
 * Gets the time elapsed since the program began in seconds.
 *
 * @ret     Time since epoch in seconds.
 */
float time()
{
    return millis() / 1000.0;
}

/**
 * Initializes the BMP085 barometer.
 */
void init_baro()
{
    if (!g_baro.init())
    {
        fault(PIN_LED_BARO_FAULT, "ERROR :: BMP085 INIT FAILED", g_baro_status);
        return;
    }

    g_baro_status = Status_t::ONLINE;
}

/**
 * Initializes the BNO055 IMU.
 */
void init_imu()
{
    if (!g_imu.init())
    {
        fault(PIN_LED_IMU_FAULT, "ERROR :: BNO055 INIT FAILED", g_imu_status);
        return;
    }

    g_imu_status = Status_t::ONLINE;
}

/**
 * Initializes the canard fin servos.
 */
void init_servos()
{
    g_servo1.attach(PIN_SERVO1_PWM);
    g_servo2.attach(PIN_SERVO2_PWM);
}

/**
 * Performs pyro continuity checks.
 */
void check_pyros()
{

}

/**
 * Function run by the LED pulse thread during startup and countdown.
 */
void pulse_thread_func()
{
    while (true)
    {
        pulse_leds(g_led_pulse_conf);
    }
}

/**
 * Lower all status LEDs.
 */
inline void lower_leds()
{
    digitalWrite(PIN_LED_BARO_FAULT, LOW);
    digitalWrite(PIN_LED_IMU_FAULT, LOW);
    digitalWrite(PIN_LED_PYRO1_FAULT, LOW);
    digitalWrite(PIN_LED_PYRO2_FAULT, LOW);
}

/**
 * Updates all sensors and dumps their readings into the state vector.
 */
void update_sensors()
{
    g_imu.update();
    g_baro.update();

    SV_PRESSURE = g_baro.data().pressure;
    SV_TEMPERATURE = g_baro.data().temperature;
    SV_ACCEL_X = g_imu.data().accel_x;
    SV_ACCEL_Y = g_imu.data().accel_y;
    SV_ACCEL_Z = g_imu.data().accel_z;
    SV_GYRO_R = g_imu.data().gyro_r;
    SV_GYRO_P = g_imu.data().gyro_p;
    SV_GYRO_Y = g_imu.data().gyro_y;
}

/**
 * Deploys forward canard fins. TODO
 */
void deploy_canards()
{

}

/**
 * Deploy the drogue parachute.
 */
void deploy_drogue()
{

}

/**
 * Deploy the main parachute.
 */
void deploy_main()
{

}

/*********************************** SETUP ************************************/

void setup()
{
#ifdef DEBUG_SERIAL
    DEBUG_SERIAL.begin(115200);
    while (!DEBUG_SERIAL);
#endif

    // Zero the state vector.
    memset(&g_statevec, 0, sizeof(MainStateVector_t));
    SV_STATE = VehicleState_t::PRELTOFF;

    // Initialize subsystems.
    init_baro();
    init_imu();
    init_servos();
    check_pyros();

    // Estimate initial altitude via barometer.
    g_x0 = 0;
    for (std::size_t i = 0; i < LAUNCHPAD_ALTITUDE_EST_READINGS; i++)
    {
        g_baro.update();
        g_x0 += g_baro.data().altitude;
    }
    g_x0 /= LAUNCHPAD_ALTITUDE_EST_READINGS;
    SV_ALTITUDE = g_x0;

    // Set up Kalman filter.
    g_kf.set_delta_t(g_mtr_kf.period());
    g_kf.set_sensor_variance(POS_VARIANCE, ACC_VARIANCE);
    g_kf.set_initial_estimate(g_x0, 0, 0);
    g_kf.compute_kg(KGAIN_CALC_DEPTH);

    // Build status LED pulse configuration.
    g_led_pulse_conf.pulses = SYS_ONLINE_LED_PULSES;
    if (g_baro_status == Status::ONLINE)
    {
        g_led_pulse_conf.pins.push_back(PIN_LED_BARO_FAULT);
    }
    if (g_imu_status == Status::ONLINE)
    {
        g_led_pulse_conf.pins.push_back(PIN_LED_IMU_FAULT);
    }
    if (g_pyro1_status == Status::ONLINE)
    {
        g_led_pulse_conf.pins.push_back(PIN_LED_PYRO1_FAULT);
    }
    if (g_pyro2_status == Status::ONLINE)
    {
        g_led_pulse_conf.pins.push_back(PIN_LED_PYRO2_FAULT);
    }

    // Determine if everything initialized correctly.
    bool ok = g_baro_status  == Status_t::ONLINE &&
              g_imu_status   == Status_t::ONLINE &&
              g_pyro1_status == Status_t::ONLINE &&
              g_pyro2_status == Status_t::ONLINE;

#ifdef USING_FNW
    // Send the aux computer my status.
    token_t tok_status = ok ? FNW_TOKEN_AOK : FNW_TOKEN_ERR;
    FNW_SERIAL.begin(FNW_BAUD);
    FNW_SERIAL.write(&tok_status, sizeof(token_t));

    // Wait for confirmation from aux computer.
    while (FNW_SERIAL.peek() == -1);
    token_t tok = FNW_TOKEN_NIL;
    FNW_SERIAL.readBytes(&tok, sizeof(token_t));

    if (tok != FNW_TOKEN_AOK)
    {
        fault(PIN_LED_FNW_FAULT, "ERROR :: FAILED TO HANDSHAKE WITH AUX",
              g_fnw_status);
    }
    else
    {
        g_fnw_status = Status_t::ONLINE;
        g_led_pulse_conf.pins.push_back(PIN_LED_FNW_FAULT);
    }
#endif

    // Dispatch LED pulse thread.
    int32_t pulse_thread_id = threads.addThread(pulse_thread_func);
    if (pulse_thread_id == -1)
    {
    #ifdef DEBUG_SERIAL
        DEBUG_SERIAL.println("ERROR :: FAILED TO CREATE LED PULSE THREAD");
    #endif
        exit(1);
    }

    // Wait for liftoff while the grace period is active and the proper
    // acceleration has not yet been sensed.
    while (time() < NO_LIFTOFF_GRACE_PERIOD_S ||
           g_hist_lodet.mean() > LIFTOFF_ACCEL_TRIGGER_MPSSQ);
    {
        // Add to the liftoff detection history according to the metronome.
        if (g_mtr_lodet.poll(time()))
        {
            update_sensors();
            g_hist_lodet.add(VERTICAL_ACCEL);
        }
    }

    // You can't take the sky from me.
    SV_T_LIFTOFF = time();
    SV_STATE = VehicleState_t::PWFLIGHT;

    // Join the LED pulse thread to ensure we have full command over DIO during
    // flight without the need to synchronize.
    threads.kill(pulse_thread_id);
    threads.wait(pulse_thread_id, 1000); // Cap wait time at 1000 ms.

    // Lower all LEDs to conserve power.
    lower_leds();
}

/**
 * Updates the state machine for the entire mission.
 */
void run_state_machine()
{
    // If in powered flight, we're waiting for a timeout or sufficiently low
    // sustained acceleration to transition to cruising state.
    if (SV_STATE == VehicleState_t::PWFLIGHT)
    {
        // Update vertical acceleration history.
        if (g_mtr_bodet.poll(SV_TIME))
        {
            g_hist_bodet.add(VERTICAL_ACCEL);
        }

        // Evaluate all conditions involved in burnout detection.
        float t_since_liftoff = SV_TIME - SV_T_LIFTOFF;
        bool grace_period_over = t_since_liftoff >= NO_BURNOUT_GRACE_PERIOD_S;
        bool timed_out = t_since_liftoff >= BURNOUT_TRIGGER_TIMEOUT_S;
        bool burnout_conds_met =
                fabs(g_hist_bodet.mean() - photic::EARTH_SLGRAV_MPSSQ)
                <= BURNOUT_ACCEL_TRIGGER_NEGL_MPSSQ;

        // If the grace period is over and either the trigger has timed out
        // or the acceleration conditions are met, rocket is cruising.
        if (grace_period_over && (timed_out || burnout_conds_met))
        {
            SV_STATE = VehicleState_t::CRUISING;
            SV_T_BURNOUT = SV_TIME;
        }
    }
    // If cruising without canards deployed, we're waiting for a timeout or
    // sufficiently high altitude estimate to trigger deployment.
    else if (SV_STATE == VehicleState_t::CRUISING)
    {
        // Evaluate all conditions involved in canard deployment.
        float t_since_liftoff = SV_TIME - SV_T_LIFTOFF;
        bool grace_period_over = t_since_liftoff >= NO_CANARDS_GRACE_PERIOD_S;
        bool timed_out = t_since_liftoff >= CANARD_DEPLOYMENT_TIMEOUT_S;
        bool canard_conds_met =
                (SV_ALTITUDE - g_x0) >= CANARD_DEPLOYMENT_ALTITUDE_M;

        // If the grace period is over and either the deployment has timed out
        // or the altitude conditions are met, rocket deploys canards.
        if (grace_period_over && (timed_out || canard_conds_met))
        {
            SV_STATE = VehicleState_t::CRSCANRD;
            SV_T_CANARDS = SV_TIME;
            deploy_canards();
        }
    }
    // If cruising with canards deployed, we're waiting for a timeout or
    // sustained negative velocity estimate to trigger apogee/drogue deployment.
    // else if (SV_STATE == VehicleState::CRSCANRD)
    // {
    //     if (g_mtr_apdet.poll(SV_TIME))
    //     {
    //         g_hist_apdet.add(SV_VELOCITY);
    //     }
    //
    //     // Evaluate all conditions involved in apogee detection.
    //     float t_since_liftoff = SV_TIME - SV_T_LIFTOFF;
    //     bool grace_period_Over = t_since_liftoff >= NO_APOGEE_GRACE_PERIOD_S;
    //     bool timed_out = time_since_liftoff >= APOGEE_DETECTION_TIMEOUT_S;
    //     bool apogee_conds_met = g_hist_apdet.mean() < 0;
    //
    //     // If the grace period is over and either detection has timed out or the
    //     // velocity conditions are met, rocket assumes it is falling and deploys
    //     // the drogue parachute.
    //     if (grace_period_over && (timed_out || apogee_conds_met))
    //     {
    //         SV_STATE = VehicleState_t::FALLDROG;
    //         deploy_drogue();
    //     }
    // }
}

/************************************ LOOP ************************************/

void loop()
{
    // Fetch a timestamp for this system iteration.
    SV_TIME = time();

    // Update sensors and dump their readings into the state vector.
    update_sensors();

    // KINETIC STATE ESTIMATION. The Kalman filter updates our estimate of the
    // rocket's kinetic state. This runs in every state except pre-liftoff.
    bool do_kf = SV_STATE != VehicleState_t::PRELTOFF;
    if (do_kf && g_mtr_kf.poll(SV_TIME))
    {
        float pos_observed = SV_ALTITUDE;
        float acc_observed = VERTICAL_ACCEL;

        // Run the filter and place the new estimate into the state vector.
        photic::matrix kinstate = g_kf.filter(pos_observed, acc_observed);
        SV_ALTITUDE = kinstate[0][0];
        SV_VELOCITY = kinstate[1][0];
        SV_ACCEL = kinstate[2][0];
    }

    // STATE MACHINE UPDATE. Perform state transitions.
    run_state_machine();

    // TELEMETRY TRANSMISSION. Transmit state vector to the aux computer over
    // FNW_SERIAL for SD backup and transmission to ground station. This runs in
    // every state.
    if (g_mtr_telemtx.poll(SV_TIME))
    {
        // TODO
    }
}
