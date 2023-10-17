#ifndef PARAMETERS_HPP
#define PARAMETERS_HPP

#define HT_DEBUG_EN
#define DEBUG


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
#define D_OUTPUT_MAX (TORQUE_1*10)
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
#define regen_nm 120

#define MIN_HV_VOLTAGE 600 // apparently this is divided by ten? yes maybe, bc getmcbusvoltage returns a can packet which is the bus voltage*10? idk

// #define HT_DEBUG_EN
// Torque Calculation Defines
#define ALPHA 0.9772
#define TORQUE_1 60
#define TORQUE_2 120
#define TORQUE_3 180
#define TORQUE_4 240
#define TORQUE_5 320

// Pump speed
#define PUMP_SPEED 3400

// neo-pixel specific
#define RED    0xFF0000
#define GREEN  0x00FF00
#define BLUE   0x0000FF
#define YELLOW 0xFFFF00
#define PINK   0x10EF0078
#define ORANGE 0xE05800
#define WHITE  0xFF000000
#define BLACK  0x0
#define BRIGHTNESS 64

#endif