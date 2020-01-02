#ifndef FF
    #include <Wire.h>
#else
    #include "ff_arduino_harness.hpp"
#endif

#include "sac2020_lib.h"

// STL redefinitions that are required for std::vector.
namespace std
{
    void __throw_bad_alloc() {}

    void __throw_length_error(const char* e) {}
}

#ifndef FF
    Threads::Mutex g_dio_lock;
#endif

PulseLEDs_t g_led_pulse_conf;

void fault(uint8_t k_pin, const char* k_msg, Status_t& k_stat)
{
    digitalWrite(k_pin, HIGH);
#ifdef DEBUG_SERIAL
    DEBUG_SERIAL.println(k_msg);
#endif
    k_stat = Status_t::FAULT;
}

double time_s()
{
    return millis() / 1000.0;
}

#ifndef FF
void pulse_leds(const PulseLEDs_t k_conf)
{
    // For each LED pulse...
    for (uint8_t i = 0; i < k_conf.pulses; i++)
    {
        // Raise all LEDs.
        g_dio_lock.lock();
        for (size_t j = 0; j < k_conf.pins.size(); j++)
        {
            digitalWrite(k_conf.pins[j], HIGH);
        }
        g_dio_lock.unlock();

        // Sleep for a bit.
        threads.delay(100);

        // Lower all LEDs.
        g_dio_lock.lock();
        for (size_t j = 0; j < k_conf.pins.size(); j++)
        {
            digitalWrite(k_conf.pins[j], LOW);
        }
        g_dio_lock.unlock();

        // Sleep for another bit if not the last pulse.
        if (i != k_conf.pulses - 1)
        {
            threads.delay(100);
        }
    }

    // Big delay between pulses.
    threads.delay(900);
}
#endif
