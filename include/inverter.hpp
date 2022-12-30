#ifndef INVERTER_HPP
#define INVERTER_HPP

#include <Arduino.h>
#include <Metro.h>

#include "parameters.hpp"
#include "inverter/mc_fault_codes.hpp"
#include "inverter/mc_internal_states.hpp"
#include "inverter/mc_motor_position_information.hpp"
#include "inverter/mc_temperatures.hpp"
#include "inverter/mc_voltage_information.hpp"
#include "inverter/mc_command_message.hpp"
class Inverter
{

private:
    Metro *mcTim;
    Metro *timer_inverter_enable;
    Metro *timer_motor_controller_send;

    void writeControldisableWithZeros();
    void writeEnableNoTorque();

    uint8_t disableWithZeros[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // The message to disable the controller/cancel lockout
    uint8_t enableNoTorque[8] = {0, 0, 0, 0, 1, 1, 0, 0};   // The message to enable the motor with zero torque
    MC_command_message pm100Msg{};
    MC_internal_states pm100State{};
    MC_motor_position_information pm100Speed{};
    MC_voltage_information pm100Voltage{};
    MC_temperatures_1 pm100temp1{};
    MC_temperatures_2 pm100temp2{};
    MC_temperatures_3 pm100temp3{};
    MC_fault_codes pm100Faults{};

public:
    // this is a member init list: https://www.youtube.com/watch?v=1nfuYMXjZsA
    Inverter(Metro *mc_kick_timer, Metro *en_tim, Metro *comm_timer) : mcTim(mc_kick_timer), timer_inverter_enable(en_tim), timer_motor_controller_send(comm_timer){};

    void inverter_init();
    void doStartup();
    void inverter_kick(bool enable);
    void forceMCdischarge();
    void updateInverterCAN();
    void reset_inverter();
    int getmcBusVoltage();
    int getmcMotorRPM();
    bool check_TS_active();
    bool check_inverter_disabled();
    bool command_torque(int torque);
    void tryToClearMcFault();
    void enable_inverter();
    bool check_inverter_ready();
    bool check_inverter_enable_timeout();
    void debug_print();
    void commandRegen();
};

#endif