#ifndef PID_H
#define PID_H

class PID {
private:
    double kp;
    double ki;
    double kd;

    double integral;
    double prev_error;

    double output_min;
    double output_max;

    bool wrap_angle;

    static double wrapToPi(double angle);

public:
    PID(double kp, double ki, double kd,
        double out_min = -1e9,
        double out_max = 1e9,
        bool wrap_angle = false);

    void reset();

    double update(double setpoint, double measurement, double dt);
};

#endif