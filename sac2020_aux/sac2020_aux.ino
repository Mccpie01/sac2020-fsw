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

/**
 * Whether or not FNW indicator LED is lit.
 */
bool g_telemtx_led = false;

/**
 * Controller for status LEDs.
 */
LEDController* g_ledc = nullptr;

/********************************* FUNCTIONS **********************************/

/**
 * Initializes the SD card library and validates file operations. Updates the
 * global status appropriately.
 */
void init_sd()
{
    static const char TEST_FNAME[] = "SDVAL";
    static const char TEST_BUF[]   = "Rage against the dying of the light.";

    // Attempt to initialize library.
    if (!SD.begin(PIN_SDCARD_CHIPSEL))
    {
        fault(PIN_LED_SD_FAULT, "ERROR :: SD INIT FAILED", g_sd_status, g_ledc);
        return;
    }

    // Validate write operation.
    File out = SD.open(TEST_FNAME, FILE_WRITE);
    size_t bytes_written = out.write(TEST_BUF, sizeof TEST_BUF);
    out.close();
    if (bytes_written != sizeof TEST_BUF)
    {
        fault(PIN_LED_SD_FAULT, "ERROR :: SD TEST WRITE FAILED", g_sd_status,
              g_ledc);
        return;
    }

    // Validate read operation.
    File in = SD.open(TEST_FNAME, FILE_READ);
    char buf[sizeof TEST_BUF];
    in.read(buf, sizeof TEST_BUF);
    in.close();
    if (strcmp(buf, TEST_BUF))
    {
        fault(PIN_LED_SD_FAULT, "ERROR :: SD TEST READ FAILED", g_sd_status,
              g_ledc);
        return;
    }

    // Validate remove operation.
    SD.remove(TEST_FNAME);
    if (SD.exists(TEST_FNAME))
    {
        fault(PIN_LED_SD_FAULT, "ERROR :: SD TEST REMOVE FAILED", g_sd_status,
              g_ledc);
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
        fault(PIN_LED_GPS_FAULT, "ERROR :: GPS INIT FAILED", g_gps_status,
              g_ledc);
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
        fault(PIN_LED_RFM_FAULT, "ERROR :: RFM INIT FAILED", g_rfm_status,
              g_ledc);
        return;
    }

    if (!g_rfm.setFrequency(RFM_FREQ))
    {
        fault(PIN_LED_RFM_FAULT, "ERROR :: FAILED TO SET RFM FREQ",
              g_rfm_status, g_ledc);
        return;
    }

    g_rfm.setTxPower(23, true);

    g_rfm_status = Status_t::ONLINE;
}

/*********************************** SETUP ************************************/

void setup()
{
#ifdef DEBUG_SERIAL
    DEBUG_SERIAL.begin(115200);
    while (!DEBUG_SERIAL);
#endif

    FNW_SERIAL.begin(FNW_BAUD);

    // Set up status LED controller.
    g_ledc = new LEDController({PIN_LED_SD_FAULT,
                                PIN_LED_RFM_FAULT,
                                PIN_LED_GPS_FAULT,
                                PIN_LED_FNW_FAULT});

    // Initialize subsystems.
    init_sd();
    init_gps();
    init_rfm();

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
    // Flash LEDs while on pad.
    if (!g_liftoff)
    {
        g_ledc->run(time_s());
    }

#ifdef USING_FNW
    // Check for state vector receipt from main. Main should only start sending
    // these after liftoff.
    if (FNW_PACKET_AVAILABLE)
    {
        uint8_t packet[FNW_PACKET_SIZE];
        FNW_SERIAL.readBytes(packet, FNW_PACKET_SIZE);

        // If leading token indicates handshake, echo the packet back without
        // saving.
        if (packet[0] == FNW_TOKEN_HSH)
        {
            FNW_SERIAL.write(packet, FNW_PACKET_SIZE);
            return;
        }

        // Otherwise, it's a state vector, so we have entered flight.
        if (!g_liftoff)
        {
            g_liftoff = true;
            g_ledc->lower_all();
        }

        // Copy into struct and write to SD.
        MainStateVector_t vec;
        memcpy(&vec, packet + 1, sizeof(vec));
        File out = SD.open(TELEM_FNAME, FILE_WRITE);
        out.write((uint8_t*) (&vec), sizeof(vec));
        out.close();

        // If the state was VehicleState_t::CONCLUDE, the flight is over--raise
        // all LEDs as indication to recovery team.
        if (vec.state == VehicleState_t::CONCLUDE)
        {
            g_ledc->raise_all();
        }
        // Otherwise, flash LED to indicate telemetry activity.
        else
        {
            g_telemtx_led = !g_telemtx_led;
            digitalWrite(PIN_LED_FNW_FAULT, g_telemtx_led ? HIGH : LOW);
        }

        // Transmit to ground station via RF module.
        // g_rfm.send((uint8_t) (&vec), sizeof(vec));
        // g_rfm.waitPacketSent();
    }
#endif
}
