#include "inverter.hpp"
#include "FlexCAN_util.hpp"
#include "pedal_handler.hpp"
#define HT_DEBUG_EN
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
    CAN_message_t rxMsg2;

    // TODO Hey John delete this because I'm just putting it here to read CAN1
    // ReadDaqCAN(rxMsg2);

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
    ctrlMsg.id = 0xC0; // OUR CONTROLLER
    memcpy(ctrlMsg.buf, disableWithZeros, sizeof(ctrlMsg.buf));
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
