#include "FlexCAN_util.hpp"

FlexCAN_T4<CAN3, RX_SIZE_1024, TX_SIZE_1024> DaqCAN_;
FlexCAN_T4<CAN2, RX_SIZE_1024, TX_SIZE_1024> Inverter_CAN_;
FlexCAN_T4<CAN1, RX_SIZE_1024, TX_SIZE_1024> AccumulatorCAN_;

void InitCAN()
{
    // Daqcan only needs to send, but fast
    DaqCAN_.begin();
    DaqCAN_.setBaudRate(500000);
    DaqCAN_.setMaxMB(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES);
    for (int i = 0; i < (NUM_RX_MAILBOXES - 1); i++)
    {
        DaqCAN_.setMB((FLEXCAN_MAILBOX)i, RX, STD);
    }
    for (int i = NUM_RX_MAILBOXES; i < (NUM_TX_MAILBOXES + NUM_RX_MAILBOXES); i++)
    {
        DaqCAN_.setMB((FLEXCAN_MAILBOX)i, TX, STD);
    }
    DaqCAN_.setMB((FLEXCAN_MAILBOX)(NUM_RX_MAILBOXES - 1), RX, EXT);
    DaqCAN_.mailboxStatus();

    // inverter can must send & receive, 6rx MB and 2tx MB
    Inverter_CAN_.begin();
    Inverter_CAN_.setBaudRate(500000);
    Inverter_CAN_.setMaxMB(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES);
    for (int i = 0; i < NUM_RX_MAILBOXES; i++)
    {
        Inverter_CAN_.setMB((FLEXCAN_MAILBOX)i, RX, STD);
    }
    for (int i = NUM_RX_MAILBOXES; i < (NUM_TX_MAILBOXES + NUM_RX_MAILBOXES); i++)
    {
        Inverter_CAN_.setMB((FLEXCAN_MAILBOX)i, TX, STD);
    }
    Inverter_CAN_.mailboxStatus();

    // accumulator can must send & receive, 6rx MB and 2tx MB

    AccumulatorCAN_.begin();
    AccumulatorCAN_.setBaudRate(500000);
    AccumulatorCAN_.setMaxMB(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES);
    for (int i = 0; i < (NUM_RX_MAILBOXES - 1); i++)
    { // leave one free for ext ID
        AccumulatorCAN_.setMB((FLEXCAN_MAILBOX)i, RX, STD);
    }
    for (int i = NUM_RX_MAILBOXES; i < (NUM_TX_MAILBOXES + NUM_RX_MAILBOXES); i++)
    {
        AccumulatorCAN_.setMB((FLEXCAN_MAILBOX)i, TX, STD);
    }
    AccumulatorCAN_.setMB((FLEXCAN_MAILBOX)(NUM_RX_MAILBOXES - 1), RX, EXT);
    // AccumulatorCAN_.setMBFilter(REJECT_ALL);
    // AccumulatorCAN_.setMBFilter(MB0, 0x69, ID_BMS_CURRENT_LIMIT_INFO, 0x6B2);
    AccumulatorCAN_.mailboxStatus();
}

int unpack_flexcan_message(can_obj_ksu_ev_can_h_t *o, CAN_message_t &msg)
{
    uint64_t data;
    memcpy(&data, msg.buf, sizeof(msg.buf));
    return unpack_message(o, msg.id, data, msg.len, millis());
}

int WriteCANToInverter(CAN_message_t &msg)
{
    DaqCAN_.write(msg);
    return Inverter_CAN_.write(msg);
}
int WriteCANToJUSTInverter(CAN_message_t &msg)
{
    return Inverter_CAN_.write(msg);
}
int WriteCANToAccumulator(CAN_message_t &msg)
{
    DaqCAN_.write(msg);
    return AccumulatorCAN_.write(msg);
}
bool writeCANToJUSTAccumulator(CAN_message_t &msg)
{
    return AccumulatorCAN_.write(msg);
}
int WriteToDaqCAN(CAN_message_t &msg)
{
    return DaqCAN_.write(msg);
}

int ReadDaqCAN(CAN_message_t &msg)
{
    int ret = DaqCAN_.read(msg);
    return ret;
}

int ReadInverterCAN(CAN_message_t &msg)
{
    int ret = Inverter_CAN_.read(msg);
    // DaqCAN_.write(msg);
    return ret;
}

int ReadAccumulatorCAN(CAN_message_t &msg)
{
    int ret = AccumulatorCAN_.read(msg);
    // DaqCAN_.write(msg);
    return ret;
}
