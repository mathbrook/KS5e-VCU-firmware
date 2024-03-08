#ifndef __ADC_SPI_H__
#define __ADC_SPI_H__

#include <stdint.h>
#include <stddef.h>

#define DEFAULT_SPI_CS 10
#define DEFAULT_SPI_SPEED 1000000

// MCP3204
// maybe also mcp3208

#define NUM_ADC_CHANNELS 4
class ADC_SPI
{
public:
    ADC_SPI();
    ADC_SPI(int CS);
    ADC_SPI(int CS, unsigned int SPIspeed);
    void init(int CS, unsigned int SPIspeed);
    uint16_t read_adc(int channel);
    void update_readings(const double ADC_ALPHA);
    uint16_t get_reading(int channel);
private:
    
    int ADC_SPI_CS;
    unsigned int SPI_SPEED;
    uint16_t readings[NUM_ADC_CHANNELS] = {0,0,0,0};
};

#endif // !<ADC_SPI.H>
#pragma once
