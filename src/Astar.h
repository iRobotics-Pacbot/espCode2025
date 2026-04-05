#ifndef ASTAR_H
#define ASTAR_H

#include <vector>
#include "Point.h"

enum class Direction {
    UP,
    DOWN,
    LEFT,
    RIGHT
};

using Tree = std::vector<Line>;

// Goal finding
Point findGoal(Direction direction, const Tree& tree, double width, double height);
std::vector<Point> findGoals(Direction direction, const Tree& tree, double radius, double width, double height);

// A* helpers
double findDist(const Point& p2, const Point& p1);
double heuristic(const Point& point, const Point& goal);
std::vector<Point> findNeighbors(const Point& point, const Tree& tree);

// Path reconstruction helpers
int cameFromIdx(const Point& point, const std::vector<std::vector<Point>>& came_from);
std::vector<Point> reconstructPath(std::vector<std::vector<Point>>& came_from, Point point);

// Main A*
std::vector<Point> Astar(Point start, Point goal, const Tree& tree, double *cost_ptr);

#endif