#include <cmath>

struct Wheels { double left, right; };

Wheels wall_follow(double dist_front, double dist_diagonal) {
    double dist_right = dist_diagonal * std::sin(M_PI / 4.0);

    double correction = KP * (dist_right - TARGET_DIST);

    double left  = BASE_SPEED + correction;
    double right = BASE_SPEED - correction;

    if (dist_front < FRONT_STOP) {
        left  = -BASE_SPEED;
        right =  BASE_SPEED;
    }

    return {left, right};
}