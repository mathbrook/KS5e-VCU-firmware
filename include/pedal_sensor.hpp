#ifndef PEDAL_SENSOR_HPP
#define PEDAL_SENSOR_HPP
#include <stdint.h>
#include <Arduino.h>
struct sensor_range_t
{
    const uint16_t lowOutOfRangeThreshold;
    const uint16_t highOutOfRangeThreshold;
    const uint16_t startTravelThreshold;
    const uint16_t endTravelThreshold;

    sensor_range_t(uint16_t low, uint16_t high, uint16_t start, uint16_t end)
        : lowOutOfRangeThreshold(low),
          highOutOfRangeThreshold(high),
          startTravelThreshold(start),
          endTravelThreshold(end) {}
};


class pedalSensor
{
private:
    uint16_t &adc_reading;
    uint16_t lowOutOfRangeThreshold;
    uint16_t highOutOfRangeThreshold;
    uint16_t startTravelThreshold;
    uint16_t endTravelThreshold;
    float travelRatio;
    float voltage_reading;
    bool isBelowThreshold;
    bool isAboveThreshold;
    float scaling_factor;
    void setTravelRatio(float travel);
    void setVoltage(float voltage);

public:

    pedalSensor(uint16_t &adc_reading,
                uint16_t lowThreshold,
                uint16_t highThreshold,
                uint16_t startThreshold,
                uint16_t endThreshold,
                float ratio,
                float scale = 1.0f,
                float voltage = 0.0f)
        : adc_reading(adc_reading),
          lowOutOfRangeThreshold(lowThreshold),
          highOutOfRangeThreshold(highThreshold),
          startTravelThreshold(startThreshold),
          endTravelThreshold(endThreshold),
          travelRatio(ratio),
          voltage_reading(voltage),
          isBelowThreshold(false),
          isAboveThreshold(false),
          scaling_factor(scale) {}

    void updateOutOfRangeFlags();

    float calculateTravel();
    float getTravelRatio() const;

    bool isBelowRange() const;

    bool isAboveRange() const;

    float calculateVoltage();

    float getVoltage() const;

    void printValues() const;

    void sensor_run();
};
#endif