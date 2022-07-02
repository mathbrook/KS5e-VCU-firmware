#ifndef STATE_MACHINE_HPP
#define STATE_MACHINE_HPP
#include <Arduino.h>
#include "MCU_status.h"
#include "inverter.hpp"
#include <Metro.h>

// check that the pedals are reading within 10% of each other
// sum of the two readings should be within 10% of the average travel
// T.4.2.4


// BSE check
// EV.5.6
// FSAE T.4.3.4

// FSAE EV.5.7
// APPS/Brake Pedal Plausability Check

class PedalHandler {
  public:             
    PedalHandler pedal_handler() {};

    void init_pedal_handler();
    
    void read_pedal_values();

    bool is_accel_pedal_plausible();
    bool is_brake_pedal_plausible();


  private:
    PM100Info::MCU_pedal_readings VCUPedalReadings;
    
    

};

#endif