#ifndef ACCUMULATOR_HPP
#define ACCUMULATOR_HPP
#include <Arduino.h>
#include "Metro.h"
#include "FlexCAN_util.hpp"
class Accumulator
{

private:
    Metro *pchgTimeout;
    bool preChargeAttempted_;
    bool imdstate;
    bool bmsstate;
    int pchgState;
    unsigned long pchgAliveTimer;
    uint8_t BMS_packInfo[8];
    bool acc_ok;

public:
    Accumulator(Metro *pch_timeout) : pchgTimeout(pch_timeout){};
    void update_acc_state();
    void updateAccumulatorCAN();
    void sendPrechargeStartMsg();
    int get_precharge_state();
    bool check_precharge_success();
    bool check_precharge_timeout();
    bool GetIfPrechargeAttempted();
    void resetPchgState();
    int16_t get_acc_current();
    bool get_imd_state();
    bool get_bms_state();
    bool get_acc_state();
};

#endif