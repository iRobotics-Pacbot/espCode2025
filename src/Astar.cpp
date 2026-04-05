#include "Astar.h"
#include <cmath>
#include <algorithm>

Point findGoal(Direction direction, const Tree& tree, double width, double height) {
    Point dirPoint;

    if (direction == Direction::UP) dirPoint = {0, -height};
    else if (direction == Direction::DOWN) dirPoint = {0, height};
    else if (direction == Direction::LEFT) dirPoint = {-width, 0};
    else if (direction == Direction::RIGHT) dirPoint = {width, 0};

    double minDist = width * 2;
    Point goalPoint = {0, 0};

    for (const Line& line : tree) {
        Point endpoint = line.end;
        double dist = findDist(dirPoint, endpoint);

        if (dist < minDist) {
            minDist = dist;
            goalPoint = endpoint;
        }
    }

    return goalPoint;
}

std::vector<Point> findGoals(Direction direction, const Tree& tree, double radius, double width, double height) {
    Point dirPoint;

    if (direction == Direction::UP) dirPoint = {0, -height};
    else if (direction == Direction::DOWN) dirPoint = {0, height};
    else if (direction == Direction::LEFT) dirPoint = {-width, 0};
    else if (direction == Direction::RIGHT) dirPoint = {width, 0};

    std::vector<Point> goals;

    for (const Line& line : tree) {
        Point endpoint = line.end;
        double dist = findDist(dirPoint, endpoint);

        if (dist < radius) {
            goals.push_back(endpoint);
        }
    }

    return goals;
}

double findDist(const Point& p2, const Point& p1) {
    return std::sqrt(std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2));
}

double heuristic(const Point& point, const Point& goal) {
    return findDist(goal, point);
}

std::vector<Point> findNeighbors(const Point& point, const Tree& tree) {
    std::vector<Point> neighbors;

    for (const Line& line : tree) {
        if (line.start == point) {
            neighbors.push_back(line.end);
        }
    }

    return neighbors;
}

int cameFromIdx(const Point& point, const std::vector<std::vector<Point>>& came_from) {
    for (int i = 0; i < came_from[0].size(); i++) {
        if (came_from[0][i] == point) {
            return i;
        }
    }
    return -1;
}

std::vector<Point> reconstructPath(std::vector<std::vector<Point>>& came_from, Point point) {
    std::vector<Point> path = {point};

    while (!(point.x == 0 && point.y == 0)) {
        int index = cameFromIdx(point, came_from);
        point = came_from[1][index];
        path.insert(path.begin(), point);
    }

    return path;
}

std::vector<Point> Astar(Point start, Point goal, const Tree& tree, double *cost_ptr) {
    std::vector<Point> open_set = {start};
    std::vector<double> g_scores = {0};

    std::vector<std::vector<Point>> came_from = {{start}, {}};

    bool isFirst = true;

    while (!open_set.empty()) {
        Point current = open_set[0];
        open_set.erase(open_set.begin());

        double curr_g = g_scores[0];
        g_scores.erase(g_scores.begin());

        if (current == goal) {
            *cost_ptr = curr_g;
            return reconstructPath(came_from, current); 
        }

        std::vector<Point> neighbors = findNeighbors(current, tree);
        for (const Point& neighbor : neighbors) {
            int neighborIndex = 0;
            bool isInOpenSet = false;

            for (int i = 0; i < open_set.size(); i++) {
                if (open_set[i] == neighbor) {
                    neighborIndex = i;
                    isInOpenSet = true;
                }
            }

            if (neighbor == start) {
                if (isInOpenSet) {
                    open_set.erase(open_set.begin() + neighborIndex);
                }
                continue;
            }

            double tentative_g = curr_g + findDist(current, neighbor);

            if (isInOpenSet && g_scores[neighborIndex] > tentative_g) {
                g_scores[neighborIndex] = tentative_g;
                int i = cameFromIdx(neighbor, came_from);
                came_from[1][i] = current;
            }

            if (!isInOpenSet) {
                if (isFirst) {
                    open_set.insert(open_set.begin(), neighbor);
                    g_scores.insert(g_scores.begin(), tentative_g);
                    came_from[1].insert(came_from[1].begin(), neighbor);
                    isFirst = false;
                }

                auto f = [&](const Point& p, double g) {
                    return heuristic(p, goal) + g;
                };

                std::vector<double> f_scores;
                for (int i = 0; i < open_set.size(); i++) {
                    f_scores.push_back(f(open_set[i], g_scores[i]));
                }

                double new_f = f(neighbor, tentative_g);

                int idx = std::upper_bound(f_scores.begin(), f_scores.end(), new_f) - f_scores.begin();

                open_set.insert(open_set.begin() + idx, neighbor);
                g_scores.insert(g_scores.begin() + idx, tentative_g);

                came_from[0].push_back(neighbor);
                came_from[1].push_back(current);
            }
        }
    }

    return {};
}