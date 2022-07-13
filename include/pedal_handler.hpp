#ifndef PEDAL_HANDLER_HPP
#define PEDAL_HANDLER_HPP
#include <Arduino.h>
#include <ADC_SPI.h>
#include <SPI.h>
#include <Metro.h>
#include <stdint.h>

#include "pedal_readings.hpp"
#include "parameters.hpp"
#include "KS2eVCUgpios.hpp"
#include "FlexCAN_util.hpp"

// check that the pedals are reading within 10% of each other
// sum of the two readings should be within 10% of the average travel
// T.4.2.4

// BSE check
// EV.5.6
// FSAE T.4.3.4

// FSAE EV.5.7
// APPS/Brake Pedal Plausability Check

class PedalHandler
{
private:
    Metro *timer_debug_raw_torque;
    Metro *pedal_out;
    bool brake_is_active_;
   
    ADC_SPI pedal_ADC;
    float accel1_, accel2_, brake1_, brake2_; 
    float accel1LIMITLO_,accel1LIMITHI_,accel2LIMITLO_,accel2LIMITHI_,brake1LIMITHI_,a1Range,a2Range;//start and end ranges for the sensors
    double ratioA1,ratioA2;
    bool implausibility_occured_;
    float calculateADCVolts(float adcReading);
public:
    PedalHandler(Metro *pedal_debug_tim, Metro *deb) : timer_debug_raw_torque(pedal_debug_tim), pedal_out(deb){};
    void init_pedal_handler();
    MCU_pedal_readings VCUPedalReadings;
    bool is_accel_pedal_plausible();
    bool is_brake_pedal_plausible();
    int calculate_torque(int16_t &motor_speed, int &max_torque);
    void verify_pedals(bool &accel_is_plausible, bool &brake_is_plausible, bool &accel_and_brake_plausible, bool &impl_occ);
    bool read_pedal_values();
    void set_sensor_ranges(float accel1limitlo, float accel1limithi, float accel2limitlo, float accel2limithi, float brakelimit1hi);

};

#endif