#ifndef PARAMETERS_HPP
#define PARAMETERS_HPP

#define HT_DEBUG_EN
#define DEBUG 0
// TODO may wanna do this another way

// Pedalbox stuff

//CRUISE CONTROL
#define SET_RPM 1000
#define D_KP 0.08;
#define D_KI 0.1;
#define D_KD 0.6;
#define D_OUTPUT_MIN 0.0;
#define D_OUTPUT_MAX 500.0;


#define BRAKE_ACTIVE 2000             // Threshold for brake pedal active
#define MIN_ACCELERATOR_PEDAL_1 200    // Low accelerator implausibility threshold
#define START_ACCELERATOR_PEDAL_1 2200  // Position to start acceleration
#define END_ACCELERATOR_PEDAL_1 2670    // Position to max out acceleration
#define MAX_ACCELERATOR_PEDAL_1 4000    // High accelerator implausibility threshold
#define MIN_ACCELERATOR_PEDAL_2 200    // Low accelerator implausibility threshold
#define START_ACCELERATOR_PEDAL_2 1390  // Position to start acceleration
#define END_ACCELERATOR_PEDAL_2 1724    // Position to max out acceleration
#define MAX_ACCELERATOR_PEDAL_2 4000    // High accelerator implausibility threshold
#define HALF_ACCELERATOR_PEDAL_1 ((START_ACCELERATOR_PEDAL_1 + END_ACCELERATOR_PEDAL_1) / 2)
#define HALF_ACCELERATOR_PEDAL_2 ((START_ACCELERATOR_PEDAL_2 + END_ACCELERATOR_PEDAL_2) / 2)


#define MIN_HV_VOLTAGE 600 // apparently this is divided by ten?

// #define HT_DEBUG_EN
// Torque Calculation Defines
#define ALPHA 0.9772
#define TORQUE_1 50
#define TORQUE_2 50
// Pump speed
#define PUMP_SPEED 2048

// neo-pixel specific
#define RED    0xFF0000
#define GREEN  0x00FF00
#define BLUE   0x0000FF
#define YELLOW 0xFFFF00
#define PINK   0x10EF0078
#define ORANGE 0xE05800
#define WHITE  0xFF000000
#define BLACK  0x0
#define BRIGHTNESS 255

#endif