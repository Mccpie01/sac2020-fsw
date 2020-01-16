#include <Adafruit_GPS.h>
#include <RH_RF95.h>
#include <SD.h>
#include <SPI.h>
#include <TeensyThreads.h>

#include "sac2020_aux_pins.h"
#include "sac2020_lib.h"

/**
 * RF module frequency.
 */
#define RFM_FREQ 434
/**
 * Name of telemetry heap file on SD card.
 */
#define TELEM_FNAME "TELEM.DAT"

/*********************************** GLOBALS **********************************/

/**
 * Hardware wrappers, configured during startup.
 */
Adafruit_GPS g_gps(&Serial3);
RH_RF95      g_rfm(PIN_RFM_CHIPSEL, PIN_RFM_INTR);

/**
 * Component statuses, set during startup.
 */
Status_t g_sd_status  = Status_t::OFFLINE; // SD card.
Status_t g_gps_status = Status_t::OFFLINE; // GPS.
Status_t g_rfm_status = Status_t::OFFLINE; // RF module.
Status_t g_fnw_status = Status_t::OFFLINE; // Flight computer network.

/**
 * Whether or not rocket has entered flight. This is flipped on the first
 * receipt of a telemetry packet from main.
 */
bool g_liftoff = false;

/********************************* FUNCTIONS **********************************/

/**
 * Initializes the SD card library and validates file operations. Updates the
 * global status appropriately.
 */
void init_sd()
{
    static const char TEST_FNAME[] = "SDVALIDATE";
    static const char TEST_BUF[]   = "Rage against the dying of the light.";

    // Attempt to initialize library.
    if (!SD.begin())
    {
        fault(PIN_LED_SD_FAULT, "ERROR :: SD INIT FAILED", g_sd_status);
        return;
    }

    // Validate write operation.
    File out = SD.open(TEST_FNAME, FILE_WRITE);
    size_t bytes_written = out.write(TEST_BUF, sizeof TEST_BUF);
    out.close();
    if (bytes_written != sizeof TEST_BUF)
    {
        fault(PIN_LED_SD_FAULT, "ERROR :: SD TEST WRITE FAILED", g_sd_status);
        return;
    }

    // Validate read operation.
    File in = SD.open(TEST_FNAME, FILE_READ);
    char buf[sizeof TEST_BUF];
    in.read(buf, sizeof TEST_BUF);
    in.close();
    if (strcmp(buf, TEST_BUF))
    {
        fault(PIN_LED_SD_FAULT, "ERROR :: SD TEST READ FAILED", g_sd_status);
        return;
    }

    // Validate remove operation.
    SD.remove(TEST_FNAME);
    if (SD.exists(TEST_FNAME))
    {
        fault(PIN_LED_SD_FAULT, "ERROR :: SD TEST REMOVE FAILED", g_sd_status);
        return;
    }

    g_sd_status = Status_t::ONLINE;
}

/**
 * Initializes the GPS and updates the global status accordingly.
 */
void init_gps()
{
    if (!g_gps.begin(9600))
    {
        fault(PIN_LED_GPS_FAULT, "ERROR :: GPS INIT FAILED", g_gps_status);
        return;
    }

    g_gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    g_gps.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);

    g_gps_status = Status_t::ONLINE;
}

/**
 * Initializes the RF module and updates the global status accordingly.
 */
void init_rfm()
{
    pinMode(PIN_RFM_RESET, OUTPUT);
    digitalWrite(PIN_RFM_RESET, HIGH);

    digitalWrite(PIN_RFM_RESET, LOW);
    delay(10);
    digitalWrite(PIN_RFM_RESET, HIGH);
    delay(10);

    if (!g_rfm.init())
    {
        fault(PIN_LED_RFM_FAULT, "ERROR :: RFM INIT FAILED", g_rfm_status);
        return;
    }

    if (!g_rfm.setFrequency(RFM_FREQ))
    {
        fault(PIN_LED_RFM_FAULT, "ERROR :: FAILED TO SET RFM FREQ",
              g_rfm_status);
        return;
    }

    g_rfm.setTxPower(23, true);

    g_rfm_status = Status_t::ONLINE;
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
    digitalWrite(PIN_LED_SD_FAULT, LOW);
    digitalWrite(PIN_LED_GPS_FAULT, LOW);
    digitalWrite(PIN_LED_RFM_FAULT, LOW);
    digitalWrite(PIN_LED_FNW_FAULT, LOW);
}

/*********************************** SETUP ************************************/

void setup()
{
#ifdef DEBUG_SERIAL
    DEBUG_SERIAL.begin(115200);
    while (!DEBUG_SERIAL);
#endif

    // Initialize subsystems.
    init_sd();
    init_gps();
    init_rfm();

    // Build status LED pulse configuration.
    g_led_pulse_conf.pulses = SYS_ONLINE_LED_PULSES;
    if (g_sd_status == Status::ONLINE)
    {
        g_led_pulse_conf.pins.push_back(PIN_LED_SD_FAULT);
    }
    if (g_gps_status == Status::ONLINE)
    {
        g_led_pulse_conf.pins.push_back(PIN_LED_GPS_FAULT);
    }
    if (g_rfm_status == Status::ONLINE)
    {
        g_led_pulse_conf.pins.push_back(PIN_LED_RFM_FAULT);
    }

#ifdef USING_FNW
    // Initialize flight network line and wait for status token from main FC.
    FNW_SERIAL.begin(FNW_BAUD);
    while (FNW_SERIAL.available() == 0);
    token_t tok = FNW_TOKEN_NIL;
    FNW_SERIAL.readBytes(&tok, sizeof(token_t));

    // Send acknowledgement of receipt.
    token_t tok_ack = FNW_TOKEN_AOK;
    FNW_SERIAL.write(&tok_ack, sizeof(token_t));

    if (tok != FNW_TOKEN_AOK)
    {
        fault(PIN_LED_FNW_FAULT, "ERROR :: FAILED TO HANDSHAKE WITH MAIN",
              g_fnw_status);
    }
    else
    {
        g_fnw_status = Status_t::ONLINE;
        g_led_pulse_conf.pins.push_back(PIN_LED_FNW_FAULT);
    }
#endif

    // Dispatch LED pulse thread.
    pulse_thread_id = threads.addThread(pulse_thread_func);
    if (pulse_thread_id == -1)
    {
    #ifdef DEBUG_SERIAL
        DEBUG_SERIAL.println("ERROR :: FAILED TO CREATE LED PULSE THREAD");
    #endif
        exit(1);
    }

    // Determine if everything initialized correctly.
    bool ok = g_sd_status  == Status_t::ONLINE
           && g_gps_status == Status_t::ONLINE
           && g_rfm_status == Status_t::ONLINE
        #ifdef USING_FNW
           && g_fnw_status == Status_t::ONLINE
        #endif
              ;
}

/************************************ LOOP ************************************/

void loop()
{
#ifdef USING_FNW
    // Check for state vector receipt from main. Main should only start sending
    // these after liftoff.
    if (FNW_SERIAL.available() >= TELEM_PACKET_SIZE)
    {
        // If this is the first telemetry packet received, the rocket has
        // entered flight--kill the LEDs.
        if (!g_liftoff)
        {
            g_liftoff = true;
            threads.kill(pulse_thread_id);
            threads.wait(pulse_thread_id, 1000); // Cap wait time at 1000 ms.
            lower_leds();
        }

        // Read entire packet into memory and dump to SD card.
        MainStateVector_t vec;
        FNW_SERIAL.readBytes((uint8_t*) (&vec), sizeof(vec));
        File out = SD.open(TELEM_FNAME, FILE_WRITE);
        out.write((uint8_t*) (&vec), sizeof(vec));
        out.close();

        // Transmit to ground station via RF module.
        g_rfm.send((uint8_t) (&vec), sizeof(vec));
        g_rfm.waitPacketSent();
    }
#endif
}
