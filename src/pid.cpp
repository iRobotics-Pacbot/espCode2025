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
    angle = fmod(angle + M_PI, 2 * M_PI);

    if (angle < 0)
        angle += 2 * M_PI;

    return angle - M_PI;
}

double PID::update(double setpoint, double measurement, double dt)
{
    double error = setpoint - measurement;

    if (wrap_angle)
        error = wrapToPi(error);

    double derivative = (error - prev_error) / dt;

    // Tentative output (before clamping)
    double output = kp * error +
                    ki * integral +
                    kd * derivative;

    // Clamp output
    if (output > output_max)
        output = output_max;
    else if (output < output_min)
        output = output_min;

    // Anti-windup: only integrate when output is not saturated,
    // or when the error would reduce the integral (drive it back in-bounds).
    double tentative_integral = integral + error * dt;
    double test_output = kp * error + ki * tentative_integral + kd * derivative;
    if (test_output <= output_max && test_output >= output_min) {
        integral = tentative_integral;
    }

    prev_error = error;

    return output;
}
