#ifndef ACCUMULATOR_HPP
#define ACCUMULATOR_HPP
#include <Arduino.h>

class Accumulator {
    public:
        Accumulator(Metro *pch_timer): pchgMsgTimer(pch_timer) {};
        void updateAccumulatorCAN();
        void sendPrechargeStartMsg();
    private:
        Metro * pchgMsgTimer;
        int pchgState;
        unsigned long pchgAliveTimer;
        uint8_t BMS_packInfo[8];

};

#endif