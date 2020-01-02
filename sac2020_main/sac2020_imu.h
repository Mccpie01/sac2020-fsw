#ifndef SAC2020_IMU_H
#define SAC2020_IMU_H

#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <photic.h>

#include "utility/imumaths.h"

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
     * Updates orientation and acceleration data with the BNO055's Euler angle
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

        // Populate orientation data. Currently this maps x, y, z to roll,
        // pitch, yaw, but I have no idea if this is correct.
        m_data.gyro_r = orient.orientation.x;
        m_data.gyro_p = orient.orientation.y;
        m_data.gyro_y = orient.orientation.z;

        // Populate acceleration data.
        m_data.accel_x = accel.acceleration.x;
        m_data.accel_y = accel.acceleration.y;
        m_data.accel_z = accel.acceleration.z;

        // Populate quaternion orientation. This does not go into the data
        // struct because it uses a proprietary class. Access with
        // Sac2020Imu::quat().
        m_quat = m_bno055.getQuat();

        return true;
    }

    /**
     * Gets quaternion orientation.
     *
     * @ret     Quaternion orientation relative to startup orientation.
     */
    imu::Quaternion quat()
    {
        return m_quat;
    }

private:
    /**
     * BNO055 driver.
     */
    Adafruit_BNO055 m_bno055;
    /**
     * Last sensed quaternion orientation.
     */
    imu::Quaternion m_quat;
};

#endif
