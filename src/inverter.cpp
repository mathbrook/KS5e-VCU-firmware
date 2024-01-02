#include "inverter.hpp"
#include "FlexCAN_util.hpp"
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
    pm100temp1.print();
    pm100temp2.print();
    pm100temp3.print();
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
bool Inverter::command_torque(int torque)
{ 
    uint8_t torquePart1 = torque % 256;
    uint8_t torquePart2 = torque / 256;
    uint8_t angularVelocity1 = 0, angularVelocity2 = 0;
    bool emraxDirection = true; // forward
    bool inverterEnable = true; // go brrr
    // TODO actual regen mapping and not on/off, this was jerky on dyno
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
// Calculate discharge and charge current limits based on current pack voltage and
// target power limit
uint32_t Inverter::calc_current_limits(uint16_t pack_voltage, uint16_t discharge_power_limit,uint16_t charge_power_limit){
    #if DEBUG
    Serial.printf("calc_current_limits(uint16_t pack_voltage = %d, uint16_t discharge_power_limit = %d, uint16_t charge_power_limit = %d\n",pack_voltage,discharge_power_limit,charge_power_limit);
    #endif
    pack_voltage /=10;
    uint16_t discharge_current_limit = discharge_power_limit/pack_voltage;
    uint16_t charge_current_limit = charge_power_limit/pack_voltage;
    uint32_t current_limit = (discharge_current_limit << 16) + charge_current_limit; 
    return current_limit;
}

bool Inverter::send_current_limit(uint32_t current_limit){
    #if DEBUG
    Serial.printf("send_current_limit(uint32_t current_limit = %d\n",current_limit);
    #endif
    if (timer_current_limit->check()){
        CAN_message_t bms_current_limit_msg;
        bms_current_limit_msg.id=0x202;
        memcpy(&bms_current_limit_msg.buf[0],&current_limit,sizeof(current_limit));
        return WriteCANToInverter(bms_current_limit_msg);
    }
    else{   return false;   }
}
