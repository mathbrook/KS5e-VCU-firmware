#ifndef STATE_MACHINE_HPP
#define STATE_MACHINE_HPP
#include <Arduino.h>
#include "MCU_status.hpp"
#include "inverter.hpp"
#include "accumulator.hpp"
#include "pedal_handler.hpp"
#include "dashboard.hpp"
#include <Metro.h>

class StateMachine
{
private:
    Inverter *pm100;
    Accumulator *accumulator;
    Metro *timer_ready_sound; // Time to play RTD sound
    Dashboard *dash_;
    Metro *debug_;
    PedalHandler *pedals;
    void set_state(MCU_status &mcu_status, MCU_STATE new_state);
    void sendPrechargeStartMsg();

public:
    StateMachine(Inverter *inv, Accumulator *acc, Metro *rs_tim, Dashboard *dash, Metro *debug, PedalHandler *pedals) : pm100(inv), accumulator(acc), timer_ready_sound(rs_tim), dash_(dash), debug_(debug), pedals(pedals){};

    void init_state_machine(MCU_status &mcu_status);
    void handle_state_machine(MCU_status &mcu_status);
};

#endif