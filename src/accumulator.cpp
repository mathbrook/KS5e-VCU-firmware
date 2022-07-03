#include "accumulator.hpp"
#include "FlexCAN_util.hpp"

void Accumulator::sendPrechargeStartMsg()
{
    if (pchgMsgTimer->check())
    {
        uint8_t prechargeCmd[] = {0, 0, 0, 0, 0, 0, 0, 0};
        CAN_message_t ctrlMsg;
        ctrlMsg.len = 8;
        ctrlMsg.id = 420;
        memcpy(ctrlMsg.buf, prechargeCmd, sizeof(ctrlMsg.buf));
        WriteCANToAccumulator(ctrlMsg);
    }
}

void Accumulator::updateAccumulatorCAN()
{
    CAN_message_t rxMsg;
    char lineBuffer[200];
    if (ReadInverterCAN(rxMsg))
    {
        switch (rxMsg.id)
        {
        case (0x69):
        {
            pchgAliveTimer = millis();
            pchgState = rxMsg.buf[0];
            int accVoltage = rxMsg.buf[1] + (rxMsg.buf[2] * 100);
            int tsVoltage = rxMsg.buf[3] + (rxMsg.buf[4] * 100);
            sprintf(lineBuffer, "precharging: state: %d ACV: %dv TSV: %dv\n", pchgState, accVoltage, tsVoltage);
            break;
        }
        case (ID_BMS_INFO):
        {
            memcpy(BMS_packInfo, rxMsg.buf, sizeof(BMS_packInfo));
            break;
        }
        default:
            break;
        }
    }
}