#ifndef PARAMETERS_HPP
#define PARAMETERS_HPP

// #define HT_DEBUG_EN
#define DEBUG 0
// TODO may wanna do this another way
// Sensor	Hi	    Lo  	V Hi	    V Lo	    Range	    RATIO
// Accel1	1830	940	    2.233886719	1.147460938	1.086425781	0.651685393
// Accel2	1230	650 	1.501464844	0.793457031	0.708007813	0.66

// Pedalbox stuff

#define BRAKE_ACTIVE 2100             // Threshold for brake pedal active
#define MIN_ACCELERATOR_PEDAL_1 200    // Low accelerator implausibility threshold
#define START_ACCELERATOR_PEDAL_1 940  // Position to start acceleration
#define END_ACCELERATOR_PEDAL_1 1830    // Position to max out acceleration
#define MAX_ACCELERATOR_PEDAL_1 2500    // High accelerator implausibility threshold
#define MIN_ACCELERATOR_PEDAL_2 200    // Low accelerator implausibility threshold
#define START_ACCELERATOR_PEDAL_2 650  // Position to start acceleration
#define END_ACCELERATOR_PEDAL_2 1230    // Position to max out acceleration
#define MAX_ACCELERATOR_PEDAL_2 2000    // High accelerator implausibility threshold
#define HALF_ACCELERATOR_PEDAL_1 ((START_ACCELERATOR_PEDAL_1 + END_ACCELERATOR_PEDAL_1) / 2)
#define HALF_ACCELERATOR_PEDAL_2 ((START_ACCELERATOR_PEDAL_2 + END_ACCELERATOR_PEDAL_2) / 2)
//defines for ACTUAL pedal readings (voltages) will start here
#define SENSOR_LO 0.25
#define SENSOR_HI 4.75

#define MIN_HV_VOLTAGE 600 // apparently this is divided by ten?

// #define HT_DEBUG_EN
// Torque Calculation Defines
#define ALPHA 0.9772 //what the fuck is alpha
#define TORQUE_1 160
#define TORQUE_2 240
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