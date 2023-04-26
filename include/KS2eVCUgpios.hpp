#ifndef KS2EVCUGPIOS_HPP
#define KS2EVCUGPIOS_HPP

#define WSFL 28 //Input Pullup
#define WSFR 28 //Input Pullup
#define SDCSENSE 20 //Input ADC
#define MC_RELAY 14 //Output
#define RTDbutton 34 //Input Pullup
#define BUZZER 15 //Output
#define TORQUEMODE 33 //Input Pullup
#define LAUNCHCONTROL 35 //Input Pullup
#define NEOPIXELPIN 17 //lol

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