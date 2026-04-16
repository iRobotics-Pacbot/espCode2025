#ifndef WALL_DETECTOR_H
#define WALL_DETECTOR_H
#include "Point.h"
#include "dataTypes.h"
#include <vector>

class WallDetector {
public:
    WallDetector(std::vector<std::vector<double>>& sensorPoses);

    void newCalc(const double sensor_measurements[6],
                 MclPose& robot_pose,
                 double heading,
                 double width, double height,
                 std::vector<Line>& out_vert,
                 std::vector<Line>& out_horiz);

private:
    std::vector<std::vector<double>> sensor_poses;
    std::vector<Point> prev_readings;
    std::vector<Line> horizLines;
    std::vector<Line> vertLines;

    std::vector<Point> computePoints(const double sensor_measurements[6], MclPose &robot_pose, double heading);

    bool updateLine(int i, int point_idx,
                    const std::vector<Point>& points,
                    int dir,
                    std::vector<Line>& lines,
                    double xmin, double xmax,
                    double ymin, double ymax,
                    double threshold,
                    Line& resultLine);

    double getCoord(const Point& p, int dir);

    Point pointWithinRange(Point p, double xmin, double xmax,
                           double ymin, double ymax);

    bool isPointWithinRange(const Point& p, double xmin, double xmax,
                           double ymin, double ymax);
};

#endif