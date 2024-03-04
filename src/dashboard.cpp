
#include "FlexCAN_util.hpp"
#include "dashboard.hpp"

elapsedMillis Dashboard::get_button_last_pressed_time(uint8_t buttonNumber)
{
    return button_last_pressed_time[buttonNumber - 1];
}
void Dashboard::set_button_last_pressed_time(uint8_t setpoint, uint8_t buttonNumber)
{
    button_last_pressed_time[buttonNumber - 1] = setpoint;
}

// Reset all button timers
void Dashboard::reset_all_button_timers()
{
    for (uint8_t i = 0; i < sizeof(button_last_pressed_time) / sizeof(button_last_pressed_time[0]); i++)
    {
        set_button_last_pressed_time(0, i);
    }
}


void Dashboard::update_dash(uint8_t input)
{
    float timestamp = millis() / float(1000);
    Serial.printf("Dash last received interval: %f\n", (timestamp - (this->last_received_timestamp)));
    this->last_received_timestamp = timestamp;
    for (int i = 0; i < 6; i++)
    {
        uint8_t bit = (0x1 << i);
        bool new_val = input & bit;
        bool old_val = (this->get_buttons() & bit);
        if (new_val != old_val)
        {
            Serial.printf("Button number %d changed from %d to %d", i + 1, old_val, new_val);
            this->set_button_last_pressed_time(0, i);
        }
    }
    this->set_buttons(input);
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
            Serial.printf("Dash last received interval: %f\n", (timestamp - last_received_timestamp));
#endif
            last_received_timestamp = timestamp;
            // WriteToDaqCAN(rxMsg);
            button_states = rxMsg.buf[0];
        }
    }
}
