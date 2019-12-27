# sac2020-computer

> Which socioeconomic ideology got a man into orbit first?

---

## Computer Schema

**Main Computer**

* Teensy 3.2, 90 MHz single-core, 2 kB ROM
* Tracks vehicle state with BNO055 IMU and BMP085 barometer
* Deploys forward canards
* Fires recovery pyros
* Sends periodic telemetry packets to auxiliary computer


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
