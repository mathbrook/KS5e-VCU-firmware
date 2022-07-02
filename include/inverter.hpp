#ifndef INVERTER_HPP
#define INVERTER_HPP

#include <Arduino.h>
#include <Metro.h>

#include "inverter/mc_fault_codes.hpp"
#include "inverter/mc_internal_states.hpp"
#include "inverter/mc_motor_position_information.hpp"
#include "inverter/mc_temperatures.hpp"
#include "inverter/mc_voltage_information.hpp"

uint8_t disableWithZeros[] = {0, 0, 0, 0, 0, 0, 0, 0}; // The message to disable the controller/cancel lockout
uint8_t enableNoTorque[] = {0, 0, 0, 0, 1, 1, 0, 0};   // The message to enable the motor with zero torque

class Inverter
{
public:
  // this is a member init list: https://www.youtube.com/watch?v=1nfuYMXjZsA
  Inverter(Metro *mcTimer) : mcTim(mcTimer) {}

  void inverter_init();
  void doStartup();
  void keepInverterAlive(bool enable);
  void forceMCdischarge();
  void updateInverterCAN();

private:
  void writeControldisableWithZeros();
  void writeEnableNoTorque();
  void tryToClearMcFault();

  MC_internal_states pm100State{};
  MC_motor_position_information pm100Speed{};
  MC_voltage_information pm100Voltage{};
  MC_temperatures_1 pm100temp1{};
  MC_temperatures_2 pm100temp2{};
  MC_temperatures_3 pm100temp3{};
  MC_fault_codes pm100Faults{};

  Metro *mcTim;
};

#endif