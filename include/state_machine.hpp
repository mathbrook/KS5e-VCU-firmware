#ifndef STATE_MACHINE_HPP
#define STATE_MACHINE_HPP
#include <Arduino.h>
#include "MCU_status.h"
#include "inverter.hpp"
#include <Metro.h>

class StateMachine {
  public:             
    StateMachine state_machine() {};

    void init_state_machine();
    void handle_state_machine(MCU_STATE& mcu_stat, );

  private:
    Inverter * pm100;

};

#endif