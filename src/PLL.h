#pragma once
#include <Encoder.h>

class PLL{

private:
    float pll_kp;
    float pll_ki;
    float pll_pos = 0;
    float pll_vel = 0;
    float prev;
    Encoder encoder;

public:

    PLL(float pll_bandwidth, Encoder encoder);

    void update();

};

