#include "pedal_sensor.hpp"
// pedalSensor::pedalSensor(){

// };

// void pedalSensor::init(uint16_t adc_reading, uint16_t lowThreshold, uint16_t highThreshold, uint16_t startThreshold, uint16_t endThreshold, float ratio, float scale, float voltage)
// {
//     this->adc_reading = adc_reading;
//     this->lowOutOfRangeThreshold=lowThreshold
// }
void pedalSensor::setTravelRatio(float travel)
{
    this->travelRatio = travel;
}

float pedalSensor::getTravelRatio() const
{
    return this->travelRatio;
}

bool pedalSensor::isBelowRange() const
{
    return this->isBelowThreshold;
}

bool pedalSensor::isAboveRange() const
{
    return this->isAboveThreshold;
}

float pedalSensor::calculateVoltage()
{
    return static_cast<float>(adc_reading) * this->scaling_factor;
}

void pedalSensor::setVoltage(float voltage)
{
    this->voltage_reading = voltage;
}

float pedalSensor::getVoltage() const
{
    return this->voltage_reading;
}

void pedalSensor::updateOutOfRangeFlags()
{
    if (adc_reading < lowOutOfRangeThreshold)
    {
        this->isBelowThreshold = true;
        this->isAboveThreshold = false;
    }
    else if (adc_reading > highOutOfRangeThreshold)
    {
        this->isBelowThreshold = false;
        this->isAboveThreshold = true;
    }
    else
    {
        this->isBelowThreshold = false;
        this->isAboveThreshold = false;
    }
}

float pedalSensor::calculateTravel()
{
    this->updateOutOfRangeFlags();

    if (this->isBelowThreshold || this->isAboveThreshold)
    {
        return -1.0f; // Sensor is out of range
    }

    if (adc_reading < this->startTravelThreshold)
    {
        return 0.0f;
    }
    else if (adc_reading > this->endTravelThreshold)
    {
        return 1.0f;
    }
    else
    {
        return static_cast<float>(adc_reading - this->startTravelThreshold) /
               (this->endTravelThreshold - this->startTravelThreshold);
    }
}

void pedalSensor::printValues() const
{
    Serial.printf("Current ADC Value: %d\n",this->adc_reading);
    Serial.printf("lowOutOfRangeThreshold: %d\n", this->lowOutOfRangeThreshold);
    Serial.printf("highOutOfRangeThreshold: %d\n", this->highOutOfRangeThreshold);
    Serial.printf("startTravelThreshold: %d\n", this->startTravelThreshold);
    Serial.printf("endTravelThreshold: %d\n", this->endTravelThreshold);
    Serial.printf("travelRatio: %f\n", this->travelRatio);
    Serial.printf("scaling_factor: %f\n", this->scaling_factor);
}

void pedalSensor::sensor_run()
{
    this->setTravelRatio(this->calculateTravel());
    this->setVoltage(this->calculateVoltage());
#if DEBUG
    // this->printValues();
#endif
}