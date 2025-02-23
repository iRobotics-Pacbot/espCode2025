#include <cmath>
#include <Arduino.h>
#include <Encoder.h>


class PLL{
 
private:
    float pll_kp;
    float pll_ki;
    float pll_pos = 0;
    float pll_vel = 0;


    
    public:
    PLL(float pll_bandwidth){
        pll_kp = 2.0*pll_bandwidth;
        pll_ki = 0.25*pll_kp*pll_kp;
    }


    void update(float dt, Encoder encoder){
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


};