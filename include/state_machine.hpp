#ifndef STATE_MACHINE_HPP
#define STATE_MACHINE_HPP
#include <Arduino.h>
#include "MCU_status.h"
#include "inverter.hpp"
#include "accumulator.hpp"
#include "pedal_handler.hpp"
#include <Metro.h>

class StateMachine {
  public:             
    StateMachine(Inverter * inv, Accumulator * acc, Metro * rs_tim) : pm100(inv), accumulator(acc), timer_ready_sound(rs_tim) {};

    void init_state_machine(MCU_status &mcu_status);
    void handle_state_machine(MCU_status &mcu_status);
    

  private:
    void set_state(MCU_status &mcu_status, MCU_STATE new_state);
    void sendPrechargeStartMsg();
    Metro * timer_ready_sound; // Time to play RTD sound
    Inverter * pm100;
    Accumulator * accumulator;
    PedalHandler * pedals;
};

#endif