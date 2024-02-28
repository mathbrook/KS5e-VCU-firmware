
#include "FlexCAN_util.hpp"
#include "dashboard.hpp"



elapsedMillis Dashboard::get_torque_mode_last_pressed_time(){
    return torque_mode_last_pressed;
}
void Dashboard::set_torque_mode_last_pressed_time(uint8_t setpoint)
{
    torque_mode_last_pressed=setpoint;
}

void Dashboard::updateDashCAN()
{
    CAN_message_t rxMsg;

    if (ReadInverterCAN(rxMsg))
    {
        if (rxMsg.id == ID_DASH_BUTTONS)
        {   
            float timestamp = millis() / float(1000);
            #if DEBUG
            Serial.printf("Dash last received interval: %f\n",(timestamp-last_received_timestamp));
            #endif
            last_received_timestamp=timestamp;
            // WriteToDaqCAN(rxMsg);
            button_states = rxMsg.buf[0];
        }
    }
}

