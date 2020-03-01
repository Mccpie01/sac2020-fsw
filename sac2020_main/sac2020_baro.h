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
 * @file      sac2020_baro.h
 * @purpose   Device wrapper for the flight computer's barometer.
 * @author    Stefan deBruyn
 * @updated   2/2/2020
 */

#ifndef SAC2020_BARO_H
#define SAC2020_BARO_H

#include <Adafruit_BMP085.h>
#include <photic.h>

class Sac2020Barometer final : public photic::Barometer
{
public:
    /**
     * Initializes the BMP085.
     *
     * @ret     True if successful, false otherwise.
     */
    bool init()
    {
        return m_bmp085.begin(0x00);
    }

    /**
     * Reads in all data from the BMP085.
     *
     * @ret     Always returns true.
     */
    bool update()
    {
        m_data.pressure = m_bmp085.readPressure();
        m_data.temperature = m_bmp085.readTemperature();
        m_data.altitude = m_bmp085.readAltitude();

        return true;
    }

private:
    /**
     * BMP085 driver.
     */
    Adafruit_BMP085 m_bmp085;
};

#endif
