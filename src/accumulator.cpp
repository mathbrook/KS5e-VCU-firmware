#include "accumulator.hpp"
#include "FlexCAN_util.hpp"

void Accumulator::sendPrechargeStartMsg()
{
    uint8_t prechargeCmd[] = {0, 0, 0, 0, 0, 0, 0, 0};
    CAN_message_t ctrlMsg;
    ctrlMsg.len = 8;
    ctrlMsg.id = 420;
    memcpy(ctrlMsg.buf, prechargeCmd, sizeof(ctrlMsg.buf));
    WriteCANToAccumulator(ctrlMsg);
}