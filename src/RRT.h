#ifndef RRT_H
#define RRT_H

#include <vector>
#include "Point.h"
#include <utility>

// Random point
Point createRandomPoint(double width_range, double height_range);

// Path (parametric line)
Line pathBetweenPoints(const Point& endpoint, const Point& init_pose);

// Closest point(s)
Point findClosestPoint(const std::vector<Line>& lines, const Point& point);
std::vector<Point> findClosestPoints(const std::vector<Line>& lines, const Point& point, double radius);

// Geometry helpers
void findParamCoeffs(const Line& line,  double *vx, double *vy, double *a, double *b);

// Collision / wall logic
bool pointIsBehindWall(const Line& newLine,
                       const Point& point,
                       const std::vector<Line>& vert,
                       const std::vector<Line>& horiz);

#endif