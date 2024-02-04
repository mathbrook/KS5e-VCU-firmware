#ifndef FLEXCAN_IS_SHIT_HPP
#define FLEXCAN_IS_SHIT_HPP
#include "FlexCAN_T4.h"
#include "KS2eCAN.hpp"
#include "ksu_dbc.h"
int unpack_flexcan_message(can_obj_ksu_dbc_h_t *o,CAN_message_t &msg);
// global wrapper around flexcan_t4 because it is a shit driver that should feel bad
int WriteToDaqCAN(CAN_message_t &msg);
int WriteCANToInverter(CAN_message_t &msg);
int WriteCANToAccumulator(CAN_message_t &msg);
int WriteCANToJUSTInverter(CAN_message_t &msg);
int ReadDaqCAN(CAN_message_t &msg);
int ReadInverterCAN(CAN_message_t &msg);
int ReadAccumulatorCAN(CAN_message_t &msg);

void InitCAN();

#endif