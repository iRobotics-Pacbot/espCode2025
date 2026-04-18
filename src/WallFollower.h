#pragma once
#include <cmath>
 
static const double TARGET_DIST = 0.3;
static const double KP          = 2.0;
static const double BASE_SPEED  = 0.5;
static const double FRONT_STOP  = 0.4;
 
struct Wheels { double left, right; };
 
Wheels wall_follow(double dist_front, double dist_diagonal);