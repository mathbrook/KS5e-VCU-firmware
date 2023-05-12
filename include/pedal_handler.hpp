#ifndef PEDAL_HANDLER_HPP
#define PEDAL_HANDLER_HPP
#include <Arduino.h>
#include <ADC_SPI.h>
#include <SPI.h>
#include <Metro.h>
#include <stdint.h>
#include <AutoPID.h>
#include <FreqMeasureMulti.h>

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
    uint16_t accel1_{}, accel2_{}, brake1_{}, steering_angle_;
    bool implausibility_occured_;

    AutoPID *pid_;
    double *current_;
    double *set_;
    double *throttle_;

    //Wheel speed measuring class objects
    FreqMeasureMulti *wsfl_;
    FreqMeasureMulti *wsfr_;
    float current_rpm = 0;
    float prev_rpm;
    unsigned long current_rpm_change_time;
    int sum = 0;
    int count = 0;
    unsigned long time1;
    float current_rpm2 = 0;
    float prev_rpm2;
    unsigned long current_rpm_change_time2;
    int sum2 = 0;
    int count2 = 0;
    unsigned long time2;
    int regen_command;


public:
    PedalHandler(Metro *pedal_debug_tim, Metro *deb, AutoPID *pid, double *current, double *set, double *throttle,FreqMeasureMulti *wsfl, FreqMeasureMulti *wsfr) : timer_debug_raw_torque(pedal_debug_tim), pedal_out(deb), pid_(pid), current_(current), set_(set), throttle_(throttle),wsfl_(wsfl),wsfr_(wsfr){};
    void init_pedal_handler();
    MCU_pedal_readings VCUPedalReadings;
    bool is_accel_pedal_plausible();
    bool is_brake_pedal_plausible();
    int calculate_torque(int16_t &motor_speed, int &max_torque, bool regen_button);
    void verify_pedals(bool &accel_is_plausible, bool &brake_is_plausible, bool &accel_and_brake_plausible, bool &impl_occ);
    bool read_pedal_values();
    void get_ws();
    double get_wsfl();
    double get_wsfr();
};

#endif