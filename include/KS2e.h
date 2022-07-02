#ifndef __KS2e_H__
#define __KS2e_H__
#include <Arduino.h>
#include <ADC_SPI.h>
#include <FlexCAN_T4.h>
#include <Metro.h>
#include <MCU_status.h>
#include <SPI.h>
#include <string.h>
#include <stdint.h>
#include <KS2eCAN.h>
#include <KS2eVCUgpios.h>
#include <VCUNeoPixelBullshitLMFAO.h>
#include <Adafruit_MCP4725.h>
#include "Adafruit_LEDBackpack.h"

// #include <drivers.h>
// Pedalbox stuff

#define BRAKE_ACTIVE 2300             // Threshold for brake pedal active
#define MIN_ACCELERATOR_PEDAL_1 200   // Low accelerator implausibility threshold
#define START_ACCELERATOR_PEDAL_1 904 // Position to start acceleration
#define END_ACCELERATOR_PEDAL_1 1820  // Position to max out acceleration
#define MAX_ACCELERATOR_PEDAL_1 2500  // High accelerator implausibility threshold
#define MIN_ACCELERATOR_PEDAL_2 200   // Low accelerator implausibility threshold
#define START_ACCELERATOR_PEDAL_2 620 // Position to start acceleration
#define END_ACCELERATOR_PEDAL_2 1250  // Position to max out acceleration
#define MAX_ACCELERATOR_PEDAL_2 2000  // High accelerator implausibility threshold
#define HALF_ACCELERATOR_PEDAL_1 ((START_ACCELERATOR_PEDAL_1 + END_ACCELERATOR_PEDAL_1) / 2)
#define HALF_ACCELERATOR_PEDAL_2 ((START_ACCELERATOR_PEDAL_2 + END_ACCELERATOR_PEDAL_2) / 2)


#define MIN_HV_VOLTAGE 600

#define HT_DEBUG_EN
// Torque Calculation Defines
#define ALPHA 0.9772
#define TORQUE_1 100
#define TORQUE_2 160
// Pump speed
#define PUMP_SPEED 2048
float accel1{}, accel2{}, brake1{}, brake2{};
Adafruit_7segment DashDisplay = Adafruit_7segment();
ADC_SPI ADC(DEFAULT_SPI_CS, DEFAULT_SPI_SPEED);
Metro mcControlTimer = Metro(50);
Adafruit_MCP4725 dac;

#endif