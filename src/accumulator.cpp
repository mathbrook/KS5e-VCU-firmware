#include "accumulator.hpp"
#include "FlexCAN_util.hpp"

// returns true if the precharge for the BMS has been attempted
bool Accumulator::GetIfPrechargeAttempted()
{
    return preChargeAttempted_;
}
void Accumulator::sendPrechargeStartMsg()
{
    preChargeAttempted_ = true;
    uint8_t prechargeCmd[] = {0, 0, 0, 0, 0, 0, 0, 0};
    CAN_message_t ctrlMsg;
    ctrlMsg.len = 8;
    ctrlMsg.id = 420;
    memcpy(ctrlMsg.buf, prechargeCmd, sizeof(ctrlMsg.buf));
    WriteCANToAccumulator(ctrlMsg);
}


// returns true if precharge has succeeeded or not
bool Accumulator::check_precharge_success()
{
    return (pchgState == 2);
}
void Accumulator::resetPchgState(){
    pchgState=0;
    return;
}

// returns true if the precharge has timed out on the BMS
// if we havent timed out yet, and the precharge hasnt succeeded, dont change states
bool Accumulator::check_precharge_timeout()
{
    return (pchgTimeout->check());
}

void Accumulator::updateAccumulatorCAN()
{
    CAN_message_t rxMsg;
    char lineBuffer[200];
    Serial.println("Update Acc can");
    if (ReadAccumulatorCAN(rxMsg))
    {
        WriteToDaqCAN(rxMsg);
        Serial.println("In IF, ID: ");
        Serial.print(rxMsg.id);
        switch (rxMsg.id)
        {
        case (0x69):
        {
            pchgTimeout->reset();
            pchgState = rxMsg.buf[0];
            int accVoltage = rxMsg.buf[1] + (rxMsg.buf[2] * 100);
            int tsVoltage = rxMsg.buf[3] + (rxMsg.buf[4] * 100);
            sprintf(lineBuffer, "precharging: state: %d ACV: %dv TSV: %dv\n", pchgState, accVoltage, tsVoltage);
            #ifdef ACCDEBUG
            Serial.print(millis());
            Serial.println(lineBuffer);
            #endif
            break;
        }
        case (ID_BMS_INFO):
        {
            memcpy(BMS_packInfo, rxMsg.buf, sizeof(BMS_packInfo));
            break;
        }
        case (ID_BMS_SOC):
        {
            //forward message to dash
            WriteCANToInverter(rxMsg);
            break;
        }
        default:
            break;
        }
    }
}