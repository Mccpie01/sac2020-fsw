#ifndef SAC2020_IMU_H
#define SAC2020_IMU_H

#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <photic.h>

class Sac2020Imu final : public photic::Imu
{
public:
    /**
     * Initializes the BNO055.
     *
     * @ret     True if successful, false otherwise.
     */
    bool init()
    {
        bool success = m_bno055.begin();

        if (success)
        {
            m_bno055.setExtCrystalUse(true);
        }

        return success;
    }

    /**
     * Updates orientation and acceleration data with the BNO055's Euler angles
     * and linear accleration readings. Magnetometer data is disregarded.
     *
     * @ret     Always returns true.
     */
    bool update()
    {
        // Read in sensor events from device.
        sensors_event_t orient, accel;
        m_bno055.getEvent(&orient, Adafruit_BNO055::VECTOR_EULER);
        m_bno055.getEvent(&accel, Adafruit_BNO055::VECTOR_LINEARACCEL);

        // Populate orientation data.
        m_data.gyro_r = orient.orientation.x;
        m_data.gyro_p = orient.orientation.y;
        m_data.gyro_y = orient.orientation.z;

        // Populate acceleration data.
        m_data.accel_x = accel.acceleration.x;
        m_data.accel_y = accel.acceleration.y;
        m_data.accel_z = accel.acceleration.z;

        return true;
    }

private:
    /**
     * BNO055 driver.
     */
    Adafruit_BNO055 m_bno055;
};

#endif
