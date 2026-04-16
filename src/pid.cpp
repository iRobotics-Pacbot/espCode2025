#include "pid.h"
#include <cmath>
#include <algorithm>

PID::PID(double kp, double ki, double kd,
         double out_min,
         double out_max,
         bool wrap_angle)
{
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;

    this->output_min = out_min;
    this->output_max = out_max;

    this->wrap_angle = wrap_angle;

    integral = 0.0;
    prev_error = 0.0;
}

void PID::reset()
{
    integral = 0.0;
    prev_error = 0.0;
}

double PID::wrapToPi(double angle)
{
    angle = fmod(angle + 180, 2 * 180);

    if (angle < 0)
        angle += 2 * 180;

    return angle - 180;
}

double PID::update(double setpoint, double measurement, double dt)
{
    double error = setpoint - measurement;

    if (wrap_angle)
        error = wrapToPi(error);

    integral += error * dt;

    double derivative = (error - prev_error) / dt;

    double output = kp * error +
                    ki * integral +
                    kd * derivative;

    prev_error = error;

    return output;
}