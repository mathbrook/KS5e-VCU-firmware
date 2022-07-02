#include "FlexCAN_util.hpp"


FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Inverter_CAN_;
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> DaqCAN_;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> AccumulatorCAN_;

void InitCAN()
{
    Inverter_CAN_.begin();
    Inverter_CAN_.setBaudRate(500000);
    DaqCAN_.begin();
    DaqCAN_.setBaudRate(1000000);
    AccumulatorCAN_.begin();
    AccumulatorCAN_.setBaudRate(500000);
    Inverter_CAN_.setMaxMB(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES);
    for (int i = 0; i < NUM_RX_MAILBOXES; i++)
    {
        Inverter_CAN_.setMB((FLEXCAN_MAILBOX)i, RX, STD);
    }
    for (int i = NUM_RX_MAILBOXES; i < (NUM_TX_MAILBOXES + NUM_RX_MAILBOXES); i++)
    {
        Inverter_CAN_.setMB((FLEXCAN_MAILBOX)i, TX, STD);
    }

    AccumulatorCAN_.setMaxMB(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES);
    for (int i = 0; i < (NUM_RX_MAILBOXES - 1); i++)
    { // leave one free for ext ID
        AccumulatorCAN_.setMB((FLEXCAN_MAILBOX)i, RX, STD);
    }
    for (int i = NUM_RX_MAILBOXES; i < (NUM_TX_MAILBOXES + NUM_RX_MAILBOXES); i++)
    {
        AccumulatorCAN_.setMB((FLEXCAN_MAILBOX)i, TX, STD);
    }
    AccumulatorCAN_.setMB((FLEXCAN_MAILBOX)5, RX, EXT);

    Inverter_CAN_.mailboxStatus();
    AccumulatorCAN_.setMBFilter(REJECT_ALL);
    AccumulatorCAN_.setMBFilter(MB0, 0x69, ID_BMS_INFO, 0x6B2);
}

int WriteCANToInverter(CAN_message_t &msg)
{
    DaqCAN_.write(msg);
    return Inverter_CAN_.write(msg);
    
}
int WriteCANToAccumulator(CAN_message_t &msg)
{
    DaqCAN_.write(msg);
    return AccumulatorCAN_.write(msg);
    
}

int WriteToDaqCAN(CAN_message_t &msg)
{
    return DaqCAN_.write(msg);
}

int ReadInverterCAN(CAN_message_t &msg)
{
    return Inverter_CAN_.read(msg);
}
int ReadAccumulatorCAN(CAN_message_t &msg)
{
    return AccumulatorCAN_.read(msg);
}
