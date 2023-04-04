#ifndef FLEXCAN_IS_SHIT_HPP
#define FLEXCAN_IS_SHIT_HPP
#include "FlexCAN_T4.h"
#include "KS2eCAN.hpp"

// global wrapper around flexcan_t4 because it is a shit driver that should feel bad
int WriteToDaqCAN(CAN_message_t &msg);
int WriteCANToInverter(CAN_message_t &msg);
int WriteCANToAccumulator(CAN_message_t &msg);

int ReadDaqCAN(CAN_message_t &msg);
int ReadInverterCAN(CAN_message_t &msg);
int ReadAccumulatorCAN(CAN_message_t &msg);

void InitCAN();

#endif