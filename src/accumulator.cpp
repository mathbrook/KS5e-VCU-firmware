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

int Accumulator::get_precharge_state()
{
    return pchgState;
}

// returns true if precharge has succeeeded or not
// refer to precharge firmware repo for status enum, but for now, 2=success state
bool Accumulator::check_precharge_success()
{
    return (pchgState == 2);
}
void Accumulator::resetPchgState()
{
    pchgState = 0;
    return;
}

int16_t Accumulator::get_acc_current(){
    return 0; // TODO implement
}
// returns true if the precharge has timed out on the BMS
// if we havent timed out yet, and the precharge hasnt succeeded, dont change states
bool Accumulator::check_precharge_timeout()
{
    return (pchgTimeout->check());
}

bool Accumulator::get_imd_state()
{
    return imdstate;
}

bool Accumulator::get_bms_state()
{
    return bmsstate;
}


void Accumulator::updateAccumulatorCAN()
{
    CAN_message_t rxMsg;
    char lineBuffer[200];
    if (ReadAccumulatorCAN(rxMsg))
    {
        WriteToDaqCAN(rxMsg);
        switch (rxMsg.id)
        {
        case (ID_PRECHARGE_STATUS):
        {
            pchgTimeout->reset();
            pchgState = rxMsg.buf[0];
            int accVoltage = rxMsg.buf[1] + (rxMsg.buf[2] * 100);
            int tsVoltage = rxMsg.buf[3] + (rxMsg.buf[4] * 100);
            break;
        }
        case (ID_ACU_RELAY): // Added to recieve states from ACU for dash lights
        {
            imdstate = rxMsg.buf[1];
            bmsstate = rxMsg.buf[2];
            break;
        }
        case (ID_BMS_CURRENT_LIMIT_INFO):
        {
            break;
        }
        case (ID_BMS_PACK_VOLTAGE_INFO):
        {
            break;
        }
        case (ID_BMS_SOC):
        {
            // forward message to dash
            // write to just inverter so it doesn't get forwarded twice
            WriteCANToJUSTInverter(rxMsg);
            break;
        }
        default:
            break;
        }
    }
}