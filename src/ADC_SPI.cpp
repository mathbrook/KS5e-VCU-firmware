#include "ADC_SPI.h"
#include <SPI.h>

/*
 * This library is used to communicate with
 * the Microchip MCP3208 12-bit ADC using
 * the Teensy/Arduino SPI library
 */

/*
 * Initialize ADC SPI using default CS pin
 */
ADC_SPI::ADC_SPI()
{
	// init(DEFAULT_SPI_CS, DEFAULT_SPI_SPEED);
}

/*
 * Initialize ADC SPI using custom CS pin
 * param CS Pin to use for Chip Select
 */
ADC_SPI::ADC_SPI(int CS)
{
	init(CS, DEFAULT_SPI_SPEED);
}

ADC_SPI::ADC_SPI(int CS, unsigned int SPIspeed)
{
	init(CS, SPIspeed);
}

/*
 * Initialization helper
 */
void ADC_SPI::init(int CS, unsigned int SPIspeed)
{
	ADC_SPI_CS = CS;
	SPI_SPEED = SPIspeed;

	pinMode(ADC_SPI_CS, OUTPUT);
	digitalWrite(ADC_SPI_CS, HIGH);
	// Initialize SPI:
	SPI.begin();
}

/*
 * Measure an ADC channel
 * param channel MCP3208 channel to read
 * return 0-5V measurement scaled to 0-4095
 */
uint16_t ADC_SPI::read_adc(int channel)
{
	// Gain control of the SPI port
	// and configure settings
	SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE0));

	// Take the SS pin low to select the chip:
	digitalWrite(ADC_SPI_CS, LOW);

	// Set up channel
	byte b = B01100000;
	b |= ((channel << 2));

	// Send in the channel via SPI:
	SPI.transfer(b);

	// Read data from SPI
	byte result1 = SPI.transfer(0);
	byte result2 = SPI.transfer(0);

	// Take the SS pin high to de-select the chip:
	digitalWrite(ADC_SPI_CS, HIGH);

	// Release control of the SPI port
	SPI.endTransaction();

	return (result1 << 4) | (result2 >> 4);
}

uint16_t ADC_SPI::get_reading(int channel)
{
	if (channel > NUM_ADC_CHANNELS || channel < 0)
	{
		return 0;
	}
	return readings[channel];
}
// Will read from ADC and apply exponential smooth
void ADC_SPI::update_readings(const double ADC_ALPHA)
{
	for (unsigned int i = 0; i < (sizeof(readings) / sizeof(readings[0])); i++)
	{
		// exponential smoothing https://chat.openai.com/share/cf98f2ea-d87c-4d25-a365-e398ffebf968
		readings[i] = ADC_ALPHA * readings[i] + (1-ADC_ALPHA) * read_adc(i);
	}
}
