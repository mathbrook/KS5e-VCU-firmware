#ifndef KS2EVCUGPIOS_HPP
#define KS2EVCUGPIOS_HPP

#define WSFL 28 //Input Pullup
#define WSFR 29 //Input Pullup
#define SDCVSENSE 39 //analog
#define SDCISENSE 20 //analog
#define BSPDSENSE 16 //analog
#define BUZZER 4 //Output
#define GLV_VSENSE 41 //analog
#define GLV_ISENSE 38 //analog
#define _5V_VSENSE 40 //analog
#define A9 27
#define A10 26

#define LOWSIDE1 5
#define LOWSIDE2 6
/*
 * ADC pin definitions
 */
#define ADC_BRAKE_1_CHANNEL 2
#define ADC_HALL_CHANNEL 3 //currently offline because i cooked one of the filter hcips
#define ADC_ACCEL_1_CHANNEL 0
#define ADC_ACCEL_2_CHANNEL 1
#define CS_ADC 10
#endif