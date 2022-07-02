#include "state_machine.hpp"

void StateMachine::sendPrechargeStartMsg()
{
    uint8_t prechargeCmd[] = {0, 0, 0, 0, 0, 0, 0, 0};
    CAN_message_t ctrlMsg;
    ctrlMsg.len = 8;
    ctrlMsg.id = 420;
    memcpy(ctrlMsg.buf, prechargeCmd, sizeof(ctrlMsg.buf));
    AccumulatorCAN.write(ctrlMsg);
}

void StateMachine::

/* Handle changes in state */
void set_state(MCU_STATE new_state)
{
    if (mcu_status.get_state() == new_state)
    {
        return;
    }

    // exit logic
    switch (mcu_status.get_state())
    {
    case MCU_STATE::STARTUP:
        break;
    case MCU_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE:
        tryToClearMcFault();
        break;
    case MCU_STATE::TRACTIVE_SYSTEM_ACTIVE:
        break;
    case MCU_STATE::ENABLING_INVERTER:
        break;
    case MCU_STATE::WAITING_READY_TO_DRIVE_SOUND:
        // make dashboard stop buzzer
        mcu_status.set_activate_buzzer(false);
        digitalWrite(BUZZER, LOW);
        break;
    case MCU_STATE::READY_TO_DRIVE:
        break;
    }

    mcu_status.set_state(new_state);

    // entry logic
    switch (new_state)
    {
    case MCU_STATE::STARTUP:
        break;
    case MCU_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE:
    {
        uint8_t TSNA[] = {3, 3, 3, 3, 3, 3};
        memcpy(PixelColorz, TSNA, sizeof(PixelColorz));
        pixelColor = GREEN;
        break;
    }
    case MCU_STATE::TRACTIVE_SYSTEM_ACTIVE:
        pixelColor = RED;
        {
            uint8_t TSA[] = {0, 0, 0, 0, 0, 0};
            memcpy(PixelColorz, TSA, sizeof(PixelColorz));
        }
        break;
    case MCU_STATE::ENABLING_INVERTER:
    {
        {
            uint8_t ENINV[] = {5, 5, 5, 5, 5, 5};
            memcpy(PixelColorz, ENINV, sizeof(PixelColorz));
        }
        pixelColor = YELLOW;
        tryToClearMcFault();
        doStartup();
        Serial.println("MCU Sent enable command");
        timer_inverter_enable.reset();
        break;
    }
    case MCU_STATE::WAITING_READY_TO_DRIVE_SOUND:
        // make dashboard sound buzzer
        {
            uint8_t ENINV[] = {2, 1, 2, 1, 2, 1};
            memcpy(PixelColorz, ENINV, sizeof(PixelColorz));
        }
        pixelColor = ORANGE;
        mcu_status.set_activate_buzzer(true);
        digitalWrite(BUZZER, HIGH);
        timer_ready_sound.reset();
        Serial.println("RTDS enabled");
        break;
    case MCU_STATE::READY_TO_DRIVE:
        pixelColor = PINK;
        {
            uint8_t RTD[] = {3, 3, 3, 3, 3, 3};
            memcpy(PixelColorz, RTD, sizeof(PixelColorz));
        }
        Serial.println("Ready to drive");
        break;
    }
}

void StateMachine::handle_state_machine(MCU_status &mcu_status, )
{
    switch (mcu_status.get_state())
    {
    case MCU_STATE::STARTUP:
        break;
    case MCU_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE:

        if (pchgMsgTimer.check())
        {
            sendPrechargeStartMsg();
        }

#if DEBUG
        if (miscDebugTimer.check())
        {
            Serial.println("TS NOT ACTIVE");
        }
#endif
        // if TS is above HV threshold, move to Tractive System Active
        if ((pm100Voltage.get_dc_bus_voltage() >= MIN_HV_VOLTAGE) /* && precharge_success==true*/)
        {
#if DEBUG
            Serial.println("Setting state to TS Active from TS Not Active");
#endif
            set_state(MCU_STATE::TRACTIVE_SYSTEM_ACTIVE);
        }
        break;

    case MCU_STATE::TRACTIVE_SYSTEM_ACTIVE:
        check_TS_active();
        keepInverterAlive(0);

        // if start button has been pressed and brake pedal is held down, transition to the next state
        if (digitalRead(RTDbutton) == 0 && mcu_status.get_brake_pedal_active())
        {
#if DEBUG
            Serial.println("Setting state to Enabling Inverter");
#endif
            set_state(MCU_STATE::ENABLING_INVERTER);
        }
        break;

    case MCU_STATE::ENABLING_INVERTER:
        check_TS_active();
        keepInverterAlive(1);

        // inverter enabling timed out
        if (timer_inverter_enable.check())
        {
#if DEBUG
            Serial.println("Setting state to TS Active from Enabling Inverter");
#endif
            set_state(MCU_STATE::TRACTIVE_SYSTEM_ACTIVE);
        }
        // motor controller indicates that inverter has enabled within timeout period
        if (pm100State.get_inverter_enable_state())
        {
#if DEBUG
            Serial.println("Setting state to Waiting Ready to Drive Sound");
#endif
            set_state(MCU_STATE::WAITING_READY_TO_DRIVE_SOUND);
        }
        break;

    case MCU_STATE::WAITING_READY_TO_DRIVE_SOUND:
        check_TS_active();
        check_inverter_disabled();
        keepInverterAlive(1);

        // if the ready to drive sound has been playing for long enough, move to ready to drive mode
        if (timer_ready_sound.check())
        {
#if DEBUG
            Serial.println("Setting state to Ready to Drive");
#endif
            set_state(MCU_STATE::READY_TO_DRIVE);
        }
        break;

    case MCU_STATE::READY_TO_DRIVE:
        check_TS_active();
        check_inverter_disabled();

        // update_coulomb_count();
        if (timer_motor_controller_send.check())
        {
            // FSAE EV.5.5
            // FSAE T.4.2.10
            int calculated_torque = 0;
            calculated_torque = calculate_torque();
            if (accel1 < MIN_ACCELERATOR_PEDAL_1 || accel1 > MAX_ACCELERATOR_PEDAL_1)
            {
                mcu_status.set_no_accel_implausability(false);
#if DEBUG
                Serial.println("T.4.2.10 1");
                Serial.println(accel1);
#endif
            }
            else if (accel2 < MIN_ACCELERATOR_PEDAL_2 || accel2 > MAX_ACCELERATOR_PEDAL_2)
            {
                mcu_status.set_no_accel_implausability(false);
#if DEBUG
                Serial.println("T.4.2.10 2");
                Serial.println(accel2);
#endif
            }
            // check that the pedals are reading within 10% of each other
            // sum of the two readings should be within 10% of the average travel
            // T.4.2.4
            else if ((accel1 - (4096 - accel2)) >
                     (END_ACCELERATOR_PEDAL_1 - START_ACCELERATOR_PEDAL_1 + START_ACCELERATOR_PEDAL_2 - END_ACCELERATOR_PEDAL_2) / 20)
            {
#if DEBUG
                Serial.println("T.4.2.4");
                Serial.printf("computed - %f\n", accel1 - (4096 - accel2));
                Serial.printf("standard - %d\n", (END_ACCELERATOR_PEDAL_1 - START_ACCELERATOR_PEDAL_1 + START_ACCELERATOR_PEDAL_2 - END_ACCELERATOR_PEDAL_2) / 20);
#endif
                mcu_status.set_no_accel_implausability(false);
            }
            else
            {
#if DEBUG
                Serial.println("T.4.2.4");
                Serial.printf("computed - %f\n", accel1 - (4096 - accel2));
                Serial.printf("standard - %d\n", (END_ACCELERATOR_PEDAL_1 - START_ACCELERATOR_PEDAL_1 + START_ACCELERATOR_PEDAL_2 - END_ACCELERATOR_PEDAL_2) / 20);
#endif
                mcu_status.set_no_accel_implausability(true);
            }

            // BSE check
            // EV.5.6
            // FSAE T.4.3.4
            if (brake1 < 200 || brake1 > 4000)
            {
                mcu_status.set_no_brake_implausability(false);
            }
            else
            {
                mcu_status.set_no_brake_implausability(true);
            }

            // FSAE EV.5.7
            // APPS/Brake Pedal Plausability Check
            if (
                (
                    (accel1 > ((END_ACCELERATOR_PEDAL_1 - START_ACCELERATOR_PEDAL_1) / 4 + START_ACCELERATOR_PEDAL_1)) ||
                    (accel2 > ((END_ACCELERATOR_PEDAL_2 - START_ACCELERATOR_PEDAL_2) / 4 + START_ACCELERATOR_PEDAL_2))) &&
                mcu_status.get_brake_pedal_active())
            {
                mcu_status.set_no_accel_brake_implausability(false);
            }
            else if (
                (accel1 < ((END_ACCELERATOR_PEDAL_1 - START_ACCELERATOR_PEDAL_1) / 20 + START_ACCELERATOR_PEDAL_1)) &&
                (accel2 < ((END_ACCELERATOR_PEDAL_2 - START_ACCELERATOR_PEDAL_2) / 20 + START_ACCELERATOR_PEDAL_2)))
            {
                mcu_status.set_no_accel_brake_implausability(true);
            }
            calculated_torque = 0;

            if (
                mcu_status.get_no_brake_implausability() &&
                mcu_status.get_no_accel_implausability() &&
                mcu_status.get_no_accel_brake_implausability())
            {
                calculated_torque = calculate_torque();
            }
            else
            {
                Serial.println("not calculating torque");
                Serial.printf("no brake implausibility: %d\n", mcu_status.get_no_brake_implausability());
                Serial.printf("no accel implausibility: %d\n", mcu_status.get_no_accel_implausability());
                Serial.printf("no accel brake implausibility: %d\n", mcu_status.get_no_accel_brake_implausability());
            }
            // Implausibility exists, command 0 torque

#if DEBUG
            // if (timer_debug_torque.check()) {
            /* Serial.print("MCU REQUESTED TORQUE: ");
             Serial.println(calculated_torque);
             Serial.print("MCU NO IMPLAUS ACCEL: ");
             Serial.println(mcu_status.get_no_accel_implausability());
             Serial.print("MCU NO IMPLAUS BRAKE: ");
             Serial.println(mcu_status.get_no_brake_implausability());
             Serial.print("MCU NO IMPLAUS ACCEL BRAKE: ");
             Serial.println(mcu_status.get_no_accel_brake_implausability());*/
            /* Serial.printf("ssok: %d\n", mcu_status.get_software_is_ok());
             Serial.printf("bms: %d\n", mcu_status.get_bms_ok_high());
             Serial.printf("imd: %d\n", mcu_status.get_imd_ok_high());*/
//}
#endif
            uint8_t torquePart1 = calculated_torque % 256;
            uint8_t torquePart2 = calculated_torque / 256;
            uint8_t angularVelocity1 = 0, angularVelocity2 = 0;
            bool emraxDirection = true; // forward
            bool inverterEnable = true; // go brrr
            uint8_t torqueCommand[] = {torquePart1, torquePart2, angularVelocity1, angularVelocity2, emraxDirection, inverterEnable, 0, 0};
            CAN_message_t ctrlMsg;
            ctrlMsg.len = 8;
            ctrlMsg.id = ID_MC_COMMAND_MESSAGE;
            memcpy(ctrlMsg.buf, torqueCommand, sizeof(ctrlMsg.buf));
            CAN.write(ctrlMsg);
            DaqCAN.write(ctrlMsg);
        }
        break;
    }
}