#include "RRT.h"
#include <cmath>
#include <cstdlib>
#include <limits>

Point createRandomPoint(double width_range, double height_range) {
    return {
        ( (double)rand() / RAND_MAX - 0.5 ) * width_range,
        ( (double)rand() / RAND_MAX - 0.5 ) * height_range
    };
}

Line pathBetweenPoints(const Point& endpoint, const Point& init_pose) {
    return {init_pose, endpoint};
}

double dist(const Point& a, const Point& b) {
    return std::sqrt(
        std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2)
    );
}

Point findClosestPoint(const std::vector<Line>& lines, const Point& point) {
    Point first = lines[0].start;
    double minDist = dist(first, point);
    Point closestPoint = first;

    for (const Line& line : lines) {
        Point endpoint = line.end;
        double d = dist(endpoint, point);

        if (d < minDist) {
            minDist = d;
            closestPoint = endpoint;
        }
    }

    return closestPoint;
}

std::vector<Point> findClosestPoints(const std::vector<Line>& lines, const Point& point, double radius) {
    std::vector<Point> closestPoints;
    Point closest = findClosestPoint(lines, point);
    closestPoints.push_back(closest);

    for (const Line& line : lines) {
        Point endpoint = line.end;
        double d = dist(endpoint, point);

        if (d <= radius) {
            closestPoints.push_back(endpoint);
        }
    }

    return closestPoints;
}

void findParamCoeffs(const Line& line, double *vx, double *vy, double *a, double *b) {
    *vx = line.end.x - line.start.x; 
    *vy = line.end.y - line.start.y; 
    *a = line.start.x;              
    *b = line.start.y;
}

bool solve2x2(double a, double b, double c, double d, double e, double f, double& x, double& y) {
    double det = a*d - b*c;
    if (std::abs(det) < 1e-8) return false;

    x = (e*d - b*f) / det;
    y = (a*f - e*c) / det;
    return true;
}

bool pointIsBehindWall(const Line& newLine, const Point& point, const std::vector<Line>& vert, const std::vector<Line>& horiz) {
    bool isBehindWall = false;
    double vx, vy, a1, b1;
    findParamCoeffs(newLine, &vx, &vy, &a1, &b1);

    if (vert.empty() && horiz.empty()) return false;

    for (const Line& line : vert) {
        double ux, uy, a2, b2;
        findParamCoeffs(line, &ux, &uy, &a2, &b2);

        double t1, t2;
        if (solve2x2(vx, -ux, vy, -uy, a2 - a1, b2 - b1, t1, t2)) {
            if (t1 >= 0 && t1 <= 1 && t2 >= 0 && t2 <= 1) {
                return true;
            }
        }

        Point lower, upper;
        if (line.end.y > line.start.y) {
            lower = line.start;
            upper = line.end;
        } else {
            lower = line.end;
            upper = line.start;
        }

        if ((line.start.x < 0 && point.x < 0) || (line.start.x > 0 && point.x > 0)) {
            double lineNum = std::abs(line.start.x);
            double pointNum = std::abs(point.x);

            if (pointNum > lineNum &&
                point.y > lower.y && point.y < upper.y) {
                isBehindWall = true;
            }
        }
    }

    for (const Line& line : horiz) {
        double ux, uy, a2, b2;
        findParamCoeffs(line, &ux, &uy, &a2, &b2);

        double t1, t2;
        if (solve2x2(vx, -ux, vy, -uy, a2 - a1, b2 - b1, t1, t2)) {
            if (t1 >= 0 && t1 <= 1 && t2 >= 0 && t2 <= 1) {
                return true;
            }
        }

        Point lower, upper;
        if (line.end.x > line.start.x) {
            lower = line.start;
            upper = line.end;
        } else {
            lower = line.end;
            upper = line.start;
        }

        if ((line.start.y < 0 && point.y < 0) || (line.start.x > 0 && point.y > 0)) {

            double lineNum = std::abs(line.start.y);
            double pointNum = std::abs(point.y);

            if (pointNum > lineNum &&
                point.x > lower.x && point.x < upper.x) {
                isBehindWall = true;
            }
        }
    }

    return isBehindWall;
}