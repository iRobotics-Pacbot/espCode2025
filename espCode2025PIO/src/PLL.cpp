#include "Encoder.cpp"

class PLL{

private:
    Encoder encoder;
    double bandwidth;
    double sim_dt = 1/20000;

public:

    PLL(Encoder encoder, double pll_bandwidth){
        this->encoder = encoder;
        bandwidth = pll_bandwidth;

        //Initialize encoder values
        encoder.set_last_update_time(0);
        encoder.set_pll_kp(2*bandwidth);
        encoder.set_pll_ki(0.25*(this->encoder.get_kp())*(this->encoder.get_kp()));

        encoder.set_pll_pos(0);
        encoder.set_pll_vel(0);
        encoder.set_pll_acc(0);
    }


    void estimator(double measurement){

        encoder.increment_pos(sim_dt*(encoder.get_vel()));

        //Importing cmath not working so used a couple workarounds. Will try to figure out getting cmath to work
        int floor = (int)(encoder.get_pos());
        double delta_pos = measurement - floor;

        encoder.increment_pos(sim_dt*(encoder.get_kp())*delta_pos);
        encoder.increment_vel(sim_dt*(encoder.get_ki())*delta_pos);


        double margin = 0.5*sim_dt*encoder.get_ki();
        
        if(encoder.get_vel() <= margin && encoder.get_vel() >= 0){
            encoder.set_pll_vel(0);
        }
        if(encoder.get_vel() >= (margin*-1) && encoder.get_vel() <= 0){
            encoder.set_pll_vel(0);
        }

        encoder.set_last_measurement(measurement);

    }




};