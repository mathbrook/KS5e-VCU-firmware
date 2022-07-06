#ifndef ACCUMULATOR_HPP
#define ACCUMULATOR_HPP
#include <Arduino.h>
#include "Metro.h"
class Accumulator
{

private:
    Metro *pchgTimeout;
    bool preChargeAttempted_;
    int pchgState;
    unsigned long pchgAliveTimer;
    uint8_t BMS_packInfo[8];

public:
    Accumulator(Metro *pch_timeout) : pchgTimeout(pch_timeout){};
    void updateAccumulatorCAN();
    void sendPrechargeStartMsg();
    bool check_precharge_success();
    bool check_precharge_timeout();
    bool GetIfPrechargeAttempted();
    void resetPchgState();
};

#endif