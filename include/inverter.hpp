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
#include "dashboard.hpp"
#include "FlexCAN_util.hpp"
class Inverter
{

private:
    can_obj_ksu_dbc_h_t *inverter_ksu_can;
    Metro *mc_kick_tim;
    Metro *timer_inverter_enable;
    Metro *timer_motor_controller_send;
    Metro *timer_current_limit;
    Dashboard *dash;
    void writeControldisableWithZeros();
    void writeEnableNoTorque();
    can_0x202_BMS_Current_Limit_t inverter_current_limit_message;
    uint8_t disableWithZeros[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // The message to disable the controller/cancel lockout
    uint8_t enableNoTorque[8] = {0, 0, 0, 0, 1, 1, 0, 0};   // The message to enable the motor with zero torque
    MC_command_message pm100Msg{};
    can_0x0c0_M192_Command_Message_t inverter_command_message;
    MC_internal_states pm100State{};
    can_0x0a0_M160_Temperature_Set_1_t inverter_temperature_set_1;
    can_0x0a1_M161_Temperature_Set_2_t inverter_temperature_set_2;
    can_0x0a2_M162_Temperature_Set_3_t inverter_temperature_set_3;
    can_0x0a3_M163_Analog_Input_Voltages_t inverter_analoginput_info;
    can_0x0a4_M164_Digital_Input_Status_t inverter_digitalinput_info;
    can_0x0a5_M165_Motor_Position_Info_t inverter_speed_info;
    can_0x0a6_M166_Current_Info_t inverter_current_info;
    can_0x0a7_M167_Voltage_Info_t inverter_voltage_info;
    can_0x0a8_M168_Flux_ID_IQ_Info_t inverter_flux_info;
    can_0x0a9_M169_Internal_Voltages_t inverter_internal_voltage_info;
    can_0x0aa_M170_Internal_States_t inverter_internal_states_info;
    can_0x0ab_M171_Fault_Codes_t inverter_fault_info;
    can_0x0ac_M172_Torque_And_Timer_Info_t inverter_torque_info;
    MC_motor_position_information pm100Speed{};
    MC_voltage_information pm100Voltage{};
    MC_temperatures_1 pm100temp1{};
    MC_temperatures_2 pm100temp2{};
    MC_temperatures_3 pm100temp3{};
    MC_fault_codes pm100Faults{};

public:
    // this is a member init list: https://www.youtube.com/watch?v=1nfuYMXjZsA
    Inverter(can_obj_ksu_dbc_h_t *ksu_can_,Metro *mc_kick_timer, Metro *en_tim, Metro *comm_timer, Metro *current_lim_tim, Dashboard *dash_) : inverter_ksu_can(ksu_can_),mc_kick_tim(mc_kick_timer), timer_inverter_enable(en_tim), timer_motor_controller_send(comm_timer),timer_current_limit(current_lim_tim),dash(dash_){};

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
    bool calc_and_send_current_limit(uint16_t pack_voltage, uint32_t discharge_power_limit,uint32_t charge_power_limit);
};

#endif