#ifndef COMMON_H
#define COMMON_H

struct Point {
    double x;
    double y;

    bool operator==(const Point& other) const {
        return x == other.x && y == other.y;
    }
};

struct Line {
    Point start;
    Point end;
    bool direction;
};

#endif