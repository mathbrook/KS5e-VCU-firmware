#include "inverter.hpp"
#include "FlexCAN_util.hpp"
#include "pedal_handler.hpp"
// inverter has got to be crunk up before yeeting
void Inverter::doStartup()
{
    writeEnableNoTorque();
    writeControldisableWithZeros();
    writeEnableNoTorque();

    timer_inverter_enable->reset();
}

void Inverter::updateInverterCAN()
{

    CAN_message_t rxMsg;

    if (ReadInverterCAN(rxMsg))
    {
        WriteToDaqCAN(rxMsg);
        switch (rxMsg.id)
        {
        case (ID_MC_INTERNAL_STATES):
        {
            pm100State.load(rxMsg.buf);
            break;
        }
        case (ID_MC_FAULT_CODES):
        {
            pm100Faults.load(rxMsg.buf);
            break;
        }
        case (ID_MC_VOLTAGE_INFORMATION):
        {
            pm100Voltage.load(rxMsg.buf);

            break;
        }
        case (ID_MC_MOTOR_POSITION_INFORMATION):
        {

            pm100Speed.load(rxMsg.buf);
            break;
        }
        case (ID_MC_TEMPERATURES_1):
        {
            pm100temp1.load(rxMsg.buf);
            break;
        }
        case (ID_MC_TEMPERATURES_2):
        {
            pm100temp2.load(rxMsg.buf);
            break;
        }
        case (ID_MC_TEMPERATURES_3):
        {
            pm100temp3.load(rxMsg.buf);
            break;
        }
        case (ID_DASH_BUTTONS):
        {
            float timestamp = millis() / float(1000);
            Serial.printf("Dash last received interval: %f\n", (timestamp - (dash->last_received_timestamp)));
            dash->last_received_timestamp = timestamp;
            dash->set_buttons(rxMsg.buf[0]);
        }
        default:
            break;
        }
    }
}

void Inverter::debug_print()
{

}

void Inverter::writeControldisableWithZeros()
{
    CAN_message_t ctrlMsg;
    ctrlMsg.len = 8;
    ctrlMsg.id = ID_MC_COMMAND_MESSAGE; // OUR CONTROLLER
    memcpy(ctrlMsg.buf, disableWithZeros, sizeof(ctrlMsg.buf));
}

void Inverter::writeEnableNoTorque()
{
    CAN_message_t ctrlMsg;
    ctrlMsg.len = 8;
    ctrlMsg.id = ID_MC_COMMAND_MESSAGE; // OUR CONTROLLER
    memcpy(ctrlMsg.buf, enableNoTorque, sizeof(ctrlMsg.buf));
    WriteCANToInverter(ctrlMsg);
}

// returns false if the command was unable to be sent
bool Inverter::command_torque(int torque)
{
    uint8_t torquePart1 = torque % 256;
    uint8_t torquePart2 = torque / 256;
    uint8_t angularVelocity1 = 0, angularVelocity2 = 0;
    bool emraxDirection = true; // forward
    bool inverterEnable = true; // go brrr
    // // TODO actual regen mapping and not on/off, this was jerky on dyno
    //  if(pedals->VCUPedalReadings.get_brake_transducer_1()>=1950){
    //    torquePart1=0x9C;
    //    torquePart2=0xFf; //-10nm sussy regen
    //  }
    uint8_t torqueCommand[] = {
        torquePart1, torquePart2, angularVelocity1, angularVelocity2, emraxDirection, inverterEnable, 0, 0};
    if (timer_motor_controller_send->check())
    {
        CAN_message_t ctrlMsg;
        ctrlMsg.len = 8;
        ctrlMsg.id = ID_MC_COMMAND_MESSAGE;
        memcpy(ctrlMsg.buf, torqueCommand, sizeof(ctrlMsg.buf));
        if (WriteCANToInverter(ctrlMsg))
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    return true;
}

//
bool Inverter::check_inverter_ready()
{
    bool inverter_is_enabled = pm100State.get_inverter_enable_state();

    // delay(1000);
    if (inverter_is_enabled)
    {

        timer_inverter_enable->reset();
    }
    return inverter_is_enabled;
}

// check to see if the enverter has been enabled
bool Inverter::check_inverter_enable_timeout()
{
    return timer_inverter_enable->check();
}

void Inverter::enable_inverter()
{
    tryToClearMcFault();
    doStartup();
    timer_inverter_enable->reset();
    inverter_kick(1);
}

// kicks the inverter's heartbeat with the enable flag to either enable or disable inverter
void Inverter::inverter_kick(bool enable)
{ // do u want the MC on or not?
    if (mc_kick_tim->check())
    {
        CAN_message_t ctrlMsg;
        ctrlMsg.len = 8;
        ctrlMsg.id = ID_MC_COMMAND_MESSAGE;
        uint8_t heartbeatMsg[] = {0, 0, 0, 0, 1, enable, 0, 0};
        memcpy(ctrlMsg.buf, heartbeatMsg, sizeof(ctrlMsg.buf));
        WriteCANToInverter(ctrlMsg);
    }
}

// oh fuck go back
void Inverter::tryToClearMcFault()
{
    CAN_message_t ctrlMsg;
    ctrlMsg.len = 8;
    ctrlMsg.id = ID_MC_READ_WRITE_PARAMETER_COMMAND;
    uint8_t clearFaultMsg[] = {20, 0, 1, 0, 0, 0, 0, 0};
    memcpy(ctrlMsg.buf, clearFaultMsg, sizeof(ctrlMsg.buf));
    for (int i = 0; i < 3; i++)
    {
        WriteCANToInverter(ctrlMsg);
    }
}

// release the electrons they too hot
void Inverter::forceMCdischarge()
{

    elapsedMillis dischargeCountdown = 0;
    while (dischargeCountdown <= 100)
    {
        if (mc_kick_tim->check() == 1)
        {
            CAN_message_t ctrlMsg;
            ctrlMsg.len = 8;
            ctrlMsg.id = ID_MC_COMMAND_MESSAGE;
            uint8_t dischgMsg[] = {0, 0, 0, 0, 1, 0b0000010, 0, 0}; // bit one?
            memcpy(ctrlMsg.buf, dischgMsg, sizeof(ctrlMsg.buf));
            WriteCANToInverter(ctrlMsg);
        }
    }
    for (int i = 0; i <= 10; i++)
    {
        writeControldisableWithZeros();
    }
}

int Inverter::getmcBusVoltage()
{
    return pm100Voltage.get_dc_bus_voltage();
}
int Inverter::getmcMotorRPM()
{
    return pm100Speed.get_motor_speed();
}

/* Shared state functinality */

// returns false if mc bus voltage is below min, true if otherwise
bool Inverter::check_TS_active()
{
    /*


    */

    if ((getmcBusVoltage() < MIN_HV_VOLTAGE))
    {

        // set_state(MCU_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE);
        return false;
    }
    else
    {
        return true;
    }
}
// if the inverter becomes disabled, return to Tractive system active.
// returns false if the inverter is disabled
bool Inverter::check_inverter_disabled()
{
    return (!pm100State.get_inverter_enable_state());
}
// Calculate discharge and charge current limits based on current pack voltage and
// target power limit
bool Inverter::calc_and_send_current_limit(uint16_t pack_voltage, uint32_t discharge_power_limit, uint32_t charge_power_limit)
{
    pack_voltage /= 10;
    uint16_t discharge_current_limit = min((discharge_power_limit) / pack_voltage, accumulator_max_discharge_current);
    uint16_t charge_current_limit = min((charge_power_limit / pack_voltage), accumulator_max_charge_current);
    if (timer_current_limit->check())
    {
#if DEBUG
        Serial.printf("discharge current limit: %d charge current limit: %d\n", discharge_current_limit, charge_current_limit);
#endif
        CAN_message_t bms_current_limit_msg;
        bms_current_limit_msg.id = ID_MC_CURRENT_LIMIT_COMMAND;
        memcpy(&bms_current_limit_msg.buf[0], &discharge_current_limit, sizeof(discharge_current_limit));
        memcpy(&bms_current_limit_msg.buf[2], &charge_current_limit, sizeof(charge_current_limit));
        return WriteCANToInverter(bms_current_limit_msg);
    }
    else
    {
        return false;
    }
}
