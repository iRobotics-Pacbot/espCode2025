#include <cmath>
#include <Arduino.h>
#include <PLL.h>



PLL::PLL(float pll_bandwidth, Encoder encoder){
    pll_kp = 2.0*pll_bandwidth;
    pll_ki = 0.25*pll_kp*pll_kp;
    prev = ESP.getCycleCount();
    this->encoder = encoder;
    
}



void PLL::update(){
    
    //Calculate dt
    float freq = ESP.getCpuFreqMHz();
    
    float current = ESP.getCycleCount();
    float dt = (current - prev)/(freq*pow(10, 6));
    prev = current;
    
    
    //Update position
    int32_t measurement = encoder.read();
    pll_pos += dt * pll_vel;

    float delta_pos = measurement - floor(pll_pos);

    pll_pos += dt * pll_kp * delta_pos;
    pll_vel += dt * pll_ki * delta_pos;

    float margin = 0.5*dt*pll_ki;
    if(fabs(pll_vel) <= margin){
        pll_vel = 0;
    }
}
