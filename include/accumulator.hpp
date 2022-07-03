#ifndef ACCUMULATOR_HPP
#define ACCUMULATOR_HPP
#include <Arduino.h>

class Accumulator {
    public:
        Accumulator(Metro * pch_timeout): pchgTimeout(pch_timeout) {};
        void updateAccumulatorCAN();
        void sendPrechargeStartMsg();
        bool check_precharge_success();
        bool check_precharge_timeout();
        bool GetIfPrechargeAttempted();
    private:
        bool preChargeAttempted_;
        Metro * pchgTimeout;
        int pchgState;
        unsigned long pchgAliveTimer;
        uint8_t BMS_packInfo[8];

};

#endif