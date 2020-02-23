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

void fault(uint8_t k_pin, const char* k_msg, Status_t& k_stat,
           LEDController* k_ledc)
{
#ifdef DEBUG_SERIAL
    DEBUG_SERIAL.println(k_msg);
#endif
#ifdef BLE_SERIAL
    BLE_SERIAL.println(k_msg);
#endif
    k_stat = Status_t::FAULT;
    k_ledc->flash(k_pin);
}

double time_s()
{
    return millis() / 1000.0;
}

#ifndef FF
LEDController::LEDController(std::vector<uint8_t> k_pins)
{
    for (uint8_t& pin : k_pins)
    {
        Pin_t pin_conf = {pin, true, false, -1};
        m_pins.push_back(pin_conf);
        pinMode(pin, OUTPUT);
    }
}

void LEDController::flash(uint8_t k_pin)
{
    for (Pin_t& pin : m_pins)
    {
        if (pin.num == k_pin)
        {
            pin.flash = true;
            return;
        }
    }
}

void LEDController::solid(uint8_t k_pin)
{
    for (Pin_t& pin : m_pins)
    {
        if (pin.num == k_pin)
        {
            pin.flash = false;
            pin.high = true;
            return;
        }
    }
}

void LEDController::run(float k_t, int32_t k_pin)
{
    for (Pin_t& pin : m_pins)
    {
        if (k_pin != -1 && pin.num != k_pin)
        {
            continue;
        }

        if (pin.flash)
        {
            if (pin.t_flash_last == -1 ||
                k_t - pin.t_flash_last > 1.0 / LEDController::PULSES)
            {
                pin.high = !pin.high;
                pin.t_flash_last = k_t;
            }
        }

        digitalWrite(pin.num, pin.high ? HIGH : LOW);
    }
}

void LEDController::lower_all()
{
    for (Pin_t& pin : m_pins)
    {
        pin.high = false;
        digitalWrite(pin.num, LOW);
    }
}

void LEDController::raise_all()
{
    for (Pin_t& pin : m_pins)
    {
        pin.high = true;
        digitalWrite(pin.num, HIGH);
    }
}
#endif

char* sanitize_ff_fmt(const char k_str[], uint32_t k_len)
{
    static char san[128];

    uint32_t i_str = 0;
    uint32_t i_san = 0;
    while (i_str < k_len)
    {
        if (k_str[i_str] == '$' || k_str[i_str] == '#')
        {
            i_str += 2;
            continue;
        }
        san[i_san] = k_str[i_str];
        i_san++;
        i_str++;
    }

    return san;
}
