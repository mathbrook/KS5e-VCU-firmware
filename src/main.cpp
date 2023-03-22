// library includes
#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <Metro.h>
#include <string.h>
#include <stdint.h>
#include <FreqMeasureMulti.h>

// our includes
#include "MCU_status.hpp"
#include "KS2eCAN.hpp"
#include "KS2eVCUgpios.hpp"
#include "FlexCAN_util.hpp"
#include "parameters.hpp"
#include "inverter.hpp"
#include "accumulator.hpp"
#include "state_machine.hpp"
#include "FlexCAN_util.hpp"

// Metro timers for inverter:
Metro timer_mc_kick_timer = Metro(50, 1);
Metro timer_inverter_enable = Metro(2000, 1); // Timeout failed inverter enable
Metro timer_motor_controller_send = Metro(10, 1);

// timers for the accumulator:
Metro pchgMsgTimer = Metro(1000);
// Metro pchgTimeout = Metro(500);

// timers for the pedals:

Metro timer_debug_pedals_raw = Metro(100, 1);
Metro pedal_debug = Metro(100, 1);
Metro pedal_check = Metro(40, 1);

// timers for the dashboard:
Metro pm100speedInspection = Metro(500, 1);

// timers for the state machine:
Metro timer_ready_sound = Metro(1000); // Time to play RTD sound
Metro debug_tim = Metro(100, 1);
int temporarydisplaytime = 0;
//PID shit
volatile double current_rpm, set_rpm, throttle_out;
double KP = D_KP;
double KI = D_KI;
double KD = D_KD;
double OUTPUT_MIN = D_OUTPUT_MIN;
double OUTPUT_MAX = D_OUTPUT_MAX;
AutoPID speedPID(&current_rpm, &set_rpm, &throttle_out, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);
// timers for VCU state out:
Metro timer_can_update = Metro(100, 1);
//Wheel speed shit 
FreqMeasureMulti wsfl;
FreqMeasureMulti wsfr;

// objects
Dashboard dash;
Inverter pm100(&timer_mc_kick_timer, &timer_inverter_enable, &timer_motor_controller_send);
Accumulator accum(&pchgMsgTimer);
PedalHandler pedals(&timer_debug_pedals_raw, &pedal_debug, &speedPID, &current_rpm, &set_rpm, &throttle_out,&wsfl,&wsfr);
StateMachine state_machine(&pm100, &accum, &timer_ready_sound, &dash, &debug_tim, &temporarydisplaytime, &pedals, &pedal_check);
MCU_status mcu_status = MCU_status();

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup()
{
    delay(100);
    Serial.println("setup");
    InitCAN();
    mcu_status.set_max_torque(0); // no torque on startup
    mcu_status.set_torque_mode(0);
    Serial.println("initted mcu status");
    pinMode(BUZZER, OUTPUT); //TODO write gpio initialization function
    digitalWrite(BUZZER, LOW);
    pinMode(LOWSIDE1, OUTPUT);
    pinMode(LOWSIDE2, OUTPUT);
    pinMode(WSFL, INPUT_PULLUP);
    pinMode(WSFR, INPUT_PULLUP);
    mcu_status.set_inverter_powered(true); //this means nothing anymore
    mcu_status.set_max_torque(TORQUE_1);
    state_machine.init_state_machine(mcu_status);
}

void loop()
{

    state_machine.handle_state_machine(mcu_status);

    if (timer_can_update.check())
    {
        // Send Main Control Unit status message
        CAN_message_t tx_msg;
        mcu_status.write(tx_msg.buf);
        tx_msg.id = ID_VCU_STATUS;
        tx_msg.len = sizeof(mcu_status);
        WriteCANToInverter(tx_msg);
    }
//   if(Serial.available()){
//     String message=(Serial.readString());
//     int userreq=message.toInt();
//     Serial.println(userreq);
//     //Validation for inputs being int between 0-90 degrees
//         pump_dac.setVoltage(userreq, false);
//     }
}
