#include <cmath>
#include <Arduino.h>
#include <Encoder.h>


class PLL{
 
private:

    //float sim_dt = 1/20000;

    float bandwidth;
    float last_update_time;
    float pll_kp;
    float pll_ki;
    float last_measurement;
    float pll_pos;
    float pll_vel;
    float pll_acc;
    
    Encoder encoder;

public:

    PLL(float pll_bandwidth){

        bandwidth = pll_bandwidth;
        last_update_time = 0;
        pll_kp = 2*bandwidth;
        pll_ki = 0.25*pll_kp*pll_kp;

        pll_pos = 0;
        pll_vel = 0;
        pll_acc = 0;


    }


    void update(float dt){

        int32_t measurement = encoder.read();
        pll_pos = pll_pos + (dt*pll_vel);

        float delta_pos = measurement - floor(pll_pos);

        pll_pos = pll_pos + (dt * pll_kp * delta_pos);
        pll_vel = pll_vel + (dt * pll_ki * delta_pos);

        float margin = 0.5*dt*pll_ki;
        if(fabs(pll_vel <= margin)){
            pll_vel = 0;
        }

        last_measurement = measurement;
    }


};