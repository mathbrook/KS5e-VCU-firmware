#pragma once

#include "drivers.h"

// #ifdef DRIVER
//     #if DRIVER == Matthew
        #define BRAKE_ACTIVE 25000               // Threshold for brake pedal active  

        #define MIN_ACCELERATOR_PEDAL_1 21100    // Low accelerator implausibility threshold
        #define START_ACCELERATOR_PEDAL_1 21700  // Position to start acceleration
        #define END_ACCELERATOR_PEDAL_1 26000    // Position to max out acceleration
        #define MAX_ACCELERATOR_PEDAL_1 26949    // High accelerator implausibility threshold

        #define MIN_ACCELERATOR_PEDAL_2 13700    // Low accelerator implausibility threshold
        #define START_ACCELERATOR_PEDAL_2 14200  // Position to start acceleration
        #define END_ACCELERATOR_PEDAL_2 17250    // Position to max out acceleration
        #define MAX_ACCELERATOR_PEDAL_2 17565    // High accelerator implausibility threshold
//    #else
//         #error "Bad driver definition"
//     #endif
// #else
//     #error "Driver required"
// #endif

#define HALF_ACCELERATOR_PEDAL_1 ((START_ACCELERATOR_PEDAL_1 + END_ACCELERATOR_PEDAL_1)/2)
#define HALF_ACCELERATOR_PEDAL_2 ((START_ACCELERATOR_PEDAL_2 + END_ACCELERATOR_PEDAL_2)/2)
