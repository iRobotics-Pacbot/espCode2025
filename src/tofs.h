#include <vl53l4cx_class.h>
#pragma once
#define NUM_TOFS 1

class tofs{
    private:
        uint16_t tofDistance[NUM_TOFS];
        VL53L4CX sensors[NUM_TOFS];

    public:
        tofs();
        void tofInit();
        void tofBeginReadings();
        void tofUpdateReadings();
        double tofGet(int which);
};