#include "FlexCAN_util.hpp"
#include "dashboard.hpp"
void Dashboard::updateDashCAN(){
    CAN_message_t rxMsg;
    if(ReadInverterCAN(rxMsg)){
        if(rxMsg.id == ID_DASH_BUTTONS){
            button_states=rxMsg.buf[0];
        }
    }
}