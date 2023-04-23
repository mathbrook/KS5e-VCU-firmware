
#include "FlexCAN_util.hpp"
#include "dashboard.hpp"


uint8_t counter = 0;
uint8_t   digit = 0;


void Dashboard::updateDashCAN()
{
    CAN_message_t rxMsg;

    if(ReadInverterCAN(rxMsg))
    {
        if(rxMsg.id == ID_DASH_BUTTONS)
        {
            button_states = rxMsg.buf[0];
        }
    }
}


void Dashboard::ByteEachDigit(int num)
{
    if (num >= 10)
    {
        ByteEachDigit(num / 10);
    }
    else if (counter == 4)
    {
        counter = 0;
    }

    digit = num % 10;

    this->BusVolt_ByteEachDigit[counter] = digit;

    counter++;
}


uint8_t *Dashboard::getBusVoltage()
{
    return this->BusVolt_ByteEachDigit;
}