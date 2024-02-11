#pragma once

#include <string.h>
#include <stdint.h>

#pragma pack(push,1)

class Dashboard 
{

public:
    Dashboard() = default;
    inline uint8_t get_buttons()  const {return button_states;}

    inline bool get_button1()  const {return !(button_states&0x01);} // RTD BUTTON
    inline bool get_button2()  const {return !(button_states&0x02);} // REGEN GREEN LMAO
    inline bool get_button3()  const {return !(button_states&0x04);}
    inline bool get_button4()  const {return !(button_states&0x08);}
    inline bool get_button5()  const {return !(button_states&0x10);}
    inline bool get_button6()  const {return !(button_states&0x20);}

    inline void set_buttons(uint8_t inputs)        { button_states = inputs; }

    void updateDashCAN();
    void set_dash_msg_rx(bool set_to);
    bool get_dash_msg_rx();
    elapsedMillis get_torque_mode_last_pressed();
    void set_torque_mode_last_pressed(uint8_t setpoint);

    bool new_dash_msg_received;
    float last_received_timestamp = 0;
private:
    uint8_t button_states;
    elapsedMillis torque_mode_last_pressed;

};

#pragma pack(pop)
