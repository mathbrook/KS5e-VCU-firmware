#ifndef PARAMETERS_HPP
#define PARAMETERS_HPP

#define DEBUG false


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
#define WHEELSPEED_TOOTH_COUNT 18
//Longer RPM Timeout means we can read slower RPM
//Shouldnt hurt to have it long. Should help for things like
//Traction control
#define RPM_TIMEOUT 1000
#define MIN_BRAKE_PEDAL 400           // ~0.5v, set on 2-29-2024
#define START_BRAKE_PEDAL 1300        // 1.58V, set on 2-29-2024
#define BRAKE_ACTIVE 2293             // Threshold for brake pedal active (set to be doable by hand)
#define END_BRAKE_PEDAL 3358          // ~4.1V, approximately maxed out brake pedal, set on 2-29-2024
#define MAX_BRAKE_PEDAL 3850

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

#define REGEN_NM 60 
#define BSPD_OK_HIGH_THRESHOLD 500 // ADC reading when BSPD is Latched (OK state)
const uint16_t accumulator_max_discharge_current = 280;
const uint16_t accumulator_max_charge_current = 32;
const int accumulator_cell_count = 72;
const float accumulator_cell_minimum_voltage = 2.5;
const float accumulator_cell_nominal_voltage = 3.6;
const float accumulator_cell_maximum_voltage = 4.2;
const float bspd_current_high_threshold = 5000/(accumulator_cell_count * accumulator_cell_nominal_voltage); // Current value where BSPD current detection should be high (5kw at nominal voltage)
#define MIN_HV_VOLTAGE 600 // apparently this is divided by ten? yes maybe, bc getmcbusvoltage returns a can packet which is the bus voltage*10? idk
#define DISCHARGE_POWER_LIM 75000
#define CHARGE_POWER_LIM 9000

// Torque Calculation Defines
#define ALPHA 0.9 // This is the coefficient for exponential smoothing
// Note that the variable max_torque is uin8_t
// So it will overflow past a value of 255
const uint8_t TORQUE_1 = 60; // 1st Torque setting
const uint8_t TORQUE_2 = 120; //2nd torque seting
const uint8_t TORQUE_3 = 180; //3rd torque setting
const uint8_t TORQUE_4 = 240; //4th torque setting
const int torque_mode_list[]={TORQUE_1,TORQUE_2,TORQUE_3,TORQUE_4};

#endif