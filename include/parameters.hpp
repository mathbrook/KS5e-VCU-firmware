#ifndef PARAMETERS_HPP
#define PARAMETERS_HPP

#define HT_DEBUG_EN
// #define DEBUG


#define USE_INVERTER true
// TODO may wanna do this another way

// Pedalbox stuff

//CRUISE CONTROL
#define SLIP 1.1 //slip target if in TC mode
#define SET_RPM 1624 //rpm target if in cruise control mode
#define D_KP 1.5
#define D_KI 0.3
#define D_KD 0.5
#define D_OUTPUT_MIN 0.0
#define D_OUTPUT_MAX 2400
#define BANGBANG_RANGE 1000.0
#define PID_TIMESTEP 100.0
#define PID_MODE false //enable cruise control
#define PID_TC_MODE false //enable traction control
#define EXP_TORQUE_CURVE false //set to TRUE for kustom pedal curve

//Longer RPM Timeout means we can read slower RPM
//Shouldnt hurt to have it long. Should help for things like
//Traction control
#define RPM_TIMEOUT 1000
#define BRAKE_ACTIVE 3000             // Threshold for brake pedal active
#define MIN_ACCELERATOR_PEDAL_1 0    // Low accelerator implausibility threshold
#define START_ACCELERATOR_PEDAL_1 50  // Position to start acceleration
#define END_ACCELERATOR_PEDAL_1 1600    // Position to max out acceleration
#define MAX_ACCELERATOR_PEDAL_1 2000    // High accelerator implausibility threshold
#define MIN_ACCELERATOR_PEDAL_2 0   // Low accelerator implausibility threshold
#define START_ACCELERATOR_PEDAL_2 50  // Position to start acceleration
#define END_ACCELERATOR_PEDAL_2 2550    // Position to max out acceleration
#define MAX_ACCELERATOR_PEDAL_2 3000    // High accelerator implausibility threshold
#define HALF_ACCELERATOR_PEDAL_1 ((START_ACCELERATOR_PEDAL_1 + END_ACCELERATOR_PEDAL_1) / 2)
#define HALF_ACCELERATOR_PEDAL_2 ((START_ACCELERATOR_PEDAL_2 + END_ACCELERATOR_PEDAL_2) / 2)
#define ALPHA 0.9772
#define REGEN_NM 120 
#define BSPD_OK_HIGH_THRESHOLD 500 // ADC reading when BSPD is Latched (OK state)
const int accumulator_cell_count = 72;
const float accumulator_cell_nominal_voltage = 3.6;
const float bspd_current_high_threshold = 5000/(accumulator_cell_count * accumulator_cell_nominal_voltage); // Current value where BSPD current detection should be high (5kw at nominal voltage)
#define MIN_HV_VOLTAGE 600 // apparently this is divided by ten? yes maybe, bc getmcbusvoltage returns a can packet which is the bus voltage*10? idk
#define DISCHARGE_POWER_LIM 75000
#define CHARGE_POWER_LIM 9000

// #define HT_DEBUG_EN
// Torque Calculation Defines
#define ALPHA 0.9772
// Note that the variable max_torque is uin8_t
// So it will overflow past a value of 255
const uint8_t torque_1 = 60;
const uint8_t torque_2 = 120;
const uint8_t torque_3 = 180;
const uint8_t torque_4 = 240;
const int torque_mode_list[]={torque_1,torque_2,torque_3,torque_4};

#endif