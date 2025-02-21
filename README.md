# sac2020-computer

2024-01-03 Fork Created and looking to Fix compile error and Move the code to a Teensy 4.1 (Unified into 1 Teensy 4.1)
- Todo
  - Unified code of Main and Auxiliary Computer to run on 1 teensy 4.1
  - Add support for AltIMU-10 v6 Gyro, Accelerometer, Compass, and Altimeter (LSM6DSO, LIS3MDL, and LPS22DF Carrier)
  - Add support for more GPS
  - Add config file for Hardware available (select hardware to use)
  - Maybe add a Web page to see the rocket status
  - Add support to display rocket status on an Oled or ePaper ( When running Bench test )
  - Add support for 6 pyros (Primer and backup)
  - Add support for 10 Servos (For Canard fins, Moterize Parashut deployment, Air break Deployment, and more.)
  - Add More Documentation
  - Add support for the UNC RC 1.1 Flight computer (or Newer)
  - Add a Deploy Package ( Future Project to deploy an edf rocket in the form of a MARS or MOON Lander)

> Which socioeconomic ideology got a man into orbit first?

---

## Computer Schema

**Main Computer**

* Teensy 3.2, 90 MHz single-core, 2 kB ROM
* Tracks vehicle state with BNO055 IMU and BMP085 barometer
* Deploys forward canards
* Fires recovery pyros
* Sends periodic telemetry packets to an auxiliary computer


**Auxiliary Computer**

* Teensy 3.2, 90 MHz single-core, 2 kB ROM
* Operates Adafruit Ultimate GPS
* Operates RadioHead RF95 module communicating with ground station
* Stores telemetry received from main computer on onboard SD card

## Arduino Dependencies

* [Adafruit BNO055 driver](https://github.com/adafruit/Adafruit_BNO055)
* [Adafruit BMP085 driver](https://github.com/adafruit/Adafruit-BMP085-Library)
* [Adafruit GPS driver](https://github.com/adafruit/Adafruit_GPS)
* [Adafruit Unified Sensor driver](https://github.com/adafruit/Adafruit_Sensor)
* sac2020_lib (included in repository)
* [Photic](https://github.com/longhorn-rocketry/photic)
