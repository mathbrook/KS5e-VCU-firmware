#include "FlexCAN_util.hpp"
#include "dashboard.hpp"



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


uint8_t *Dashboard::ByteEachDigit(int num)
{
    if (num >= 10)
    {
        ByteEachDigit(num / 10);
    }
        
    int digit = num % 10;

    this->BusVolt_ByteEachDigit[this->counter] = digit;

    this->counter++;

    if (this->counter > sizeof(BusVolt_ByteEachDigit) - 1)
    {
        this->counter = 0;
        return this->BusVolt_ByteEachDigit;
    }
}