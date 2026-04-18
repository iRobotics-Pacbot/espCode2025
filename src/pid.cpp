#include "PID.h"
#include <cmath>
#include <algorithm>

// ---------- Helper ----------
double PID::wrapToPi(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

// ---------- Constructor ----------
PID::PID(double kp, double ki, double kd,
         double out_min, double out_max,
         bool wrap_angle)
    : kp(kp), ki(ki), kd(kd),
      integral(0.0), prev_error(0.0),
      output_min(out_min), output_max(out_max),
      wrap_angle(wrap_angle) {}

// ---------- Reset ----------
void PID::reset() {
    integral = 0.0;
    prev_error = 0.0;
}

// ---------- Update ----------
double PID::update(double setpoint, double measurement, double dt) {
    if (dt <= 0.0) return 0.0;

    // Compute error
    double error = setpoint - measurement;

    if (wrap_angle) {
        error = wrapToPi(error);
    }

    // Integral term
    integral += error * dt;

    // Derivative term
    double derivative = (error - prev_error) / dt;

    // PID output before clamping
    double output = kp * error + ki * integral + kd * derivative;

    // Clamp output
    double clamped_output = std::clamp(output, output_min, output_max);

    // Anti-windup (simple: only integrate if not saturated)
    if (output != clamped_output) {
        integral -= error * dt;  // undo integration
    }

    prev_error = error;

    return clamped_output;
}