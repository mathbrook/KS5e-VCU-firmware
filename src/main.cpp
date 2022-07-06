// library includes
#include <WS2812Serial.h>
#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <Metro.h>
#include <string.h>
#include <stdint.h>
#include <Adafruit_MCP4725.h>

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
Metro timer_inverter_enable = Metro(10000, 1); // Timeout failed inverter enable
Metro timer_motor_controller_send = Metro(50, 1);

// timers for the accumulator:
Metro pchgMsgTimer = Metro(1000);
//Metro pchgTimeout = Metro(500);

// timers for the pedals:
Metro timer_debug_pedals_raw = Metro(1000, 1);
Metro pedal_debug = Metro(100, 1);
Metro pedal_check = Metro(40, 1);

// timers for the dashboard:
Metro pm100speedInspection = Metro(500, 1);

// timers for the state machine:
Metro timer_ready_sound = Metro(1000); // Time to play RTD sound
Metro debug_tim = Metro(100, 1);


// timers for VCU state out:
Metro timer_can_update = Metro(100, 1);

// dashboard led handling
// TODO unfuck this
const int numled=3+6;
const int numledOnDash=6;
byte drawingMemory[numled*3];         //  3 bytes per LED
DMAMEM byte displayMemory[numled*12]; // 12 bytes per LED
byte DashdrawingMemory[numledOnDash*3];         //  3 bytes per LED
DMAMEM byte DashdisplayMemory[numledOnDash*12]; // 12 bytes per LED
WS2812Serial leds(numled, displayMemory, drawingMemory, 17, WS2812_GRB);

// objects
Inverter pm100(&timer_mc_kick_timer, &timer_inverter_enable, &timer_motor_controller_send);
Accumulator accum(&pchgMsgTimer);
PedalHandler pedals(&timer_debug_pedals_raw, &pedal_debug);
Dashboard dash(&leds, &pm100speedInspection);
StateMachine state_machine(&pm100, &accum, &timer_ready_sound, &dash, &debug_tim, &pedals, &pedal_check);
Adafruit_MCP4725 pump_dac; 
MCU_status mcu_status = MCU_status();

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup()
{

    Serial.begin(115200);
    delay(1000);
    
    Serial.println("setup");
    InitCAN();

    mcu_status.set_max_torque(0); // no torque on startup
    mcu_status.set_torque_mode(0);
    Serial.println("initted mcu status");
    
    pinMode(RTDbutton, INPUT_PULLUP);
    pinMode(BUZZER, OUTPUT);
    digitalWrite(BUZZER, LOW);
    pinMode(TORQUEMODE, INPUT);
    pinMode(LAUNCHCONTROL, INPUT_PULLUP);
    pinMode(MC_RELAY, OUTPUT);
    pinMode(WSFL, INPUT_PULLUP);
    pinMode(WSFR, INPUT_PULLUP);
    // if (!pump_dac.begin())
    // {
    //     Serial.println("L pump_dac");
    // }
    // pump_dac.setVoltage(PUMP_SPEED, false);
    digitalWrite(MC_RELAY, HIGH);
    mcu_status.set_inverter_powered(true);
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
        WriteToDaqCAN(tx_msg);
    }
}
