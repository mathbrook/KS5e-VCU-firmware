#include "inverter.hpp"
#include "FlexCAN_util.hpp"

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
        default:
            break;
        }
    }
}

void Inverter::debug_print()
{
    // pm100Speed.print();
    // pm100temp1.print();
    // pm100temp2.print();
    // pm100temp3.print();
}

void Inverter::writeControldisableWithZeros()
{
    CAN_message_t ctrlMsg;
    ctrlMsg.len = 8;
    ctrlMsg.id = 0xC0; // OUR CONTROLLER
    memcpy(ctrlMsg.buf, disableWithZeros, sizeof(ctrlMsg.buf));
    if (WriteCANToInverter(ctrlMsg) > 0)
    {
        Serial.println("****DISABLE****");
    }
}

void Inverter::writeEnableNoTorque()
{
    CAN_message_t ctrlMsg;
    ctrlMsg.len = 8;
    ctrlMsg.id = 0xC0; // OUR CONTROLLER
    memcpy(ctrlMsg.buf, enableNoTorque, sizeof(ctrlMsg.buf));
    WriteCANToInverter(ctrlMsg);
}

// returns false if the command was unable to be sent
bool Inverter::command_torque(uint8_t torqueCommand[8])
{ // do u want the MC on or not?
    if (timer_motor_controller_send->check())
    {
        CAN_message_t ctrlMsg;
        ctrlMsg.len = 8;
        ctrlMsg.id = ID_MC_COMMAND_MESSAGE;
        memcpy(ctrlMsg.buf, torqueCommand, sizeof(ctrlMsg.buf));
        if (WriteCANToInverter(ctrlMsg) > 0)
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
    #if DEBUG
    // Serial.println("checking if inverter is ready");
    // pm100State.print();
    #endif
    
    bool inverter_is_enabled = pm100State.get_inverter_enable_state();
    Serial.println(inverter_is_enabled);
    if (inverter_is_enabled)
    {
        Serial.println("resting inverter reset timer");
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
    if (mcTim->check())
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
        if (mcTim->check() == 1)
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
    if ((getmcBusVoltage() < MIN_HV_VOLTAGE))
    {
#if DEBUG
        // Serial.println("Setting state to TS Not Active, because TS is below HV threshold");
#endif
        // set_state(MCU_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE);
        return false;
    }
    else
    {
        return true;
    }
}
// if the inverter becomes disabled, return to Tractive system active.
// returns true if the inverter is disabled
bool Inverter::check_inverter_disabled()
{;
    return (!pm100State.get_inverter_enable_state());
}
