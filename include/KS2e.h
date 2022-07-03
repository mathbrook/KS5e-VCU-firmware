#ifndef __KS2e_H__
#define __KS2e_H__
#include <Arduino.h>

#include <FlexCAN_T4.h>
#include <Metro.h>
#include <MCU_status.h>

#include <string.h>
#include <stdint.h>
#include <KS2eCAN.h>
#include <KS2eVCUgpios.hpp>
#include <VCUNeoPixelBullshitLMFAO.h>
#include <Adafruit_MCP4725.h>
#include "Adafruit_LEDBackpack.h"

// #include <drivers.h>


Adafruit_7segment DashDisplay = Adafruit_7segment();
Metro mcControlTimer = Metro(50);
Adafruit_MCP4725 dac;

#endif