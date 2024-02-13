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
#include "device_status.h"

#define NEBUG

// Metro timers for inverter:
Metro timer_mc_kick_timer = Metro(50, 1);
Metro timer_inverter_enable = Metro(2000, 1); // Timeout failed inverter enable
Metro timer_motor_controller_send = Metro(100, 1);
Metro timer_current_limit_send = Metro(500, 1);

// timers for the accumulator:
Metro pchgMsgTimer = Metro(1000, 0);
// Metro pchgTimeout = Metro(500);

// timers for the pedals:

Metro timer_debug_pedals_raw = Metro(100, 1);
Metro pedal_out = Metro(50, 1);
Metro pedal_check = Metro(40, 1);

// timers for the dashboard:
Metro pm100speedInspection = Metro(500, 1);

// timers for the state machine:
Metro timer_ready_sound = Metro(1000); // Time to play RTD sound
Metro debug_tim = Metro(200, 1);

// PID shit
volatile double current_rpm, set_rpm, throttle_out;
const double KP = D_KP;
const double KI = D_KI;
const double KD = D_KD;
const double OUTPUT_MIN = D_OUTPUT_MIN;
const double OUTPUT_MAX = D_OUTPUT_MAX;

AutoPID speedPID(&current_rpm, &set_rpm, &throttle_out, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);

// timers for VCU state out:
Metro timer_can_update = Metro(1000, 1);

// Wheel speed shit
FreqMeasureMulti wsfl;
FreqMeasureMulti wsfr;

// objects
Dashboard dash;
Inverter pm100(&timer_mc_kick_timer, &timer_inverter_enable, &timer_motor_controller_send, &timer_current_limit_send, &dash);
Accumulator accum(&pchgMsgTimer);
PedalHandler pedals(&timer_debug_pedals_raw, &pedal_out, &speedPID, &current_rpm, &set_rpm, &throttle_out, &wsfl, &wsfr);
StateMachine state_machine(&pm100, &accum, &timer_ready_sound, &dash, &debug_tim, &pedals, &pedal_check);
MCU_status mcu_status = MCU_status();

static CAN_message_t mcu_status_msg;
static CAN_message_t fw_hash_msg;
static CAN_message_t pedal_thresholds_msg;
device_status_t vcu_status_t;
pedal_thresholds_t vcu_pedal_thresholds_t;

void gpio_init();

//----------------------------------------------------------------------------------------------------------------------------------------

void setup()
{
    Serial.begin(57600);
    delay(100);

    InitCAN();
    gpio_init();
    mcu_status.set_max_torque(0); // no torque on startup
    mcu_status.set_torque_mode(0);
    // build up fw git hash message
    fw_hash_msg.id = ID_VCU_FW_VERSION;
    fw_hash_msg.len = sizeof(vcu_status_t) / sizeof(uint8_t);
#if DEBUG
    Serial.printf("FW git hash: %lu, IS_DIRTY: %d IS_MAIN: %d\n", AUTO_VERSION, FW_PROJECT_IS_DIRTY, FW_PROJECT_IS_MAIN_OR_MASTER);
    Serial.printf("FW git hash in the struct: %lu\n", vcu_status_t.firmware_version);
#endif
    vcu_status_t.on_time_seconds = millis() / 1000;
    memcpy(fw_hash_msg.buf, &vcu_status_t, sizeof(vcu_status_t));

    mcu_status.set_inverter_powered(true); // note VCU does not control inverter power on rev3
    mcu_status.set_torque_mode(1);         // TODO torque modes should be an enum
    mcu_status.set_max_torque(torque_1);   // TORQUE_1=60nm, 2=120nm, 3=180nm, 4=240nm
    state_machine.init_state_machine(mcu_status);
}

void loop()
{
    state_machine.handle_state_machine(mcu_status);

    if (timer_can_update.check())
    {
        // Send Main Control Unit status message
        mcu_status.write(mcu_status_msg.buf);
        mcu_status_msg.id = ID_VCU_STATUS;
        mcu_status_msg.len = sizeof(mcu_status);
        WriteCANToInverter(mcu_status_msg);

        // broadcast firmware git hash
        vcu_status_t.on_time_seconds = millis() / 1000;
        memcpy(fw_hash_msg.buf, &vcu_status_t, sizeof(vcu_status_t));
        WriteCANToInverter(fw_hash_msg);

        // broadcast pedal thresholds information
        pedal_thresholds_msg.id = ID_VCU_PEDAL_THRESHOLD_SETTINGS;
        pedal_thresholds_msg.len = 7;
        pedal_thresholds_msg.buf[0]=0;
        memcpy(&pedal_thresholds_msg.buf[1], &vcu_pedal_thresholds_t.pedal_thresholds_0, sizeof(vcu_pedal_thresholds_t.pedal_thresholds_0));
        WriteCANToInverter(pedal_thresholds_msg);

        pedal_thresholds_msg.buf[0]=1;
        memcpy(&pedal_thresholds_msg.buf[1], &vcu_pedal_thresholds_t.pedal_thresholds_1, sizeof(vcu_pedal_thresholds_t.pedal_thresholds_1));
        WriteCANToInverter(pedal_thresholds_msg);

        pedal_thresholds_msg.buf[0]=2;
        memcpy(&pedal_thresholds_msg.buf[1], &vcu_pedal_thresholds_t.pedal_thresholds_2, sizeof(vcu_pedal_thresholds_t.pedal_thresholds_2));
        WriteCANToInverter(pedal_thresholds_msg);
    }
}

void gpio_init()
{
    pinMode(BUZZER, OUTPUT);
    digitalWrite(BUZZER, LOW);
    pinMode(LOWSIDE1, OUTPUT);
    pinMode(LOWSIDE2, OUTPUT);
    pinMode(WSFL, INPUT_PULLUP);
    pinMode(WSFR, INPUT_PULLUP);
}