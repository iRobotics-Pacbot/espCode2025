#include "WallDetector.h"
#include <Arduino.h>
#include <cmath>

WallDetector::WallDetector(const std::vector<std::vector<double> >& sensorPoses) {
    sensor_poses = sensorPoses;
    prev_readings.resize(6);
}

std::vector<Point> WallDetector::computePoints(
    const std::vector<double>& sensor_measurements) {

    std::vector<Point> points(6);

    for (int i = 0; i < 6; i++) {

        double sx = sensor_poses[i][0];
        double sy = sensor_poses[i][1];
        double stheta = sensor_poses[i][2];

        double dist = sensor_measurements[i];

        // points[i].x = rx + cos(theta) * sx - sin(theta) * sy + cos(stheta + theta) * dist;
        points[i].x = sx + (dist * cos(stheta));

       // points[i].y = ry + sin(theta) * sx + cos(theta) * sy + sin(stheta + theta) * dist;
        points[i].y = sy + (dist * sin(stheta));
    }

    return points;
}


void WallDetector::newCalc(
    const std::vector<double>& sensor_measurements,
    double width, double height,
    std::vector<Line>& out_vert,
    std::vector<Line>& out_horiz) {

    std::vector<Point> points = computePoints(sensor_measurements);

    double fieldRelXMax = width / 2.0;
    double fieldRelXMin = width / 2.0;
    double fieldRelYMax = height / 2.0;
    double fieldRelYMin = height / 2.0;

    double threshold = 8.5;
    double threshold2 = 10.0;

    for (int i = 0; i < 6; i++) {
        Serial.println("inside top calc loop");
        if (isPointWithinRange(points[i], fieldRelXMin, fieldRelXMax,
                               fieldRelYMin, fieldRelYMax)) {

            if (fabs(points[i].x - prev_readings[i].x) <= threshold) {
                bool hasChanged = false;

                for (int j = 0; j < (int)vertLines.size(); j++) {
                    Serial.println("updating vert line");
                    Line newLine;
                    bool changed = updateLine(j, i, points, 1,
                                              vertLines,
                                              fieldRelXMin, fieldRelXMax,
                                              fieldRelYMin, fieldRelYMax,
                                              threshold2,
                                              newLine);
                    if (changed) {
                        vertLines[j] = newLine;
                        hasChanged = true;
                        break;
                    }
                }

                if (!hasChanged) {
                    Line line;
                    line.start = prev_readings[i];
                    line.end = points[i];
                    line.direction = (points[i].y > prev_readings[i].y);
                    vertLines.push_back(line);
                }

            } else if (fabs(points[i].y - prev_readings[i].y) <= threshold) {
                bool hasChanged = false;

                for (int j = 0; j < (int)horizLines.size(); j++) {
                    Serial.println("updating horiz line");
                    Line newLine;
                    bool changed = updateLine(j, i, points, 0,
                                              horizLines,
                                              fieldRelXMin, fieldRelXMax,
                                              fieldRelYMin, fieldRelYMax,
                                              threshold2,
                                              newLine);
                    if (changed) {
                        horizLines[j] = newLine;
                        hasChanged = true;
                        break;
                    }
                }

                if (!hasChanged) {
                    Line line;
                    line.start = prev_readings[i];
                    line.end = points[i];
                    line.direction = (points[i].x > prev_readings[i].x);
                    horizLines.push_back(line);
                }
            }
        }
    }
    // for(int i = 0; i < (int)vertLines.size(); i++) {
    //     if(!isPointWithinRange(vertLines[i].start, fieldRelXMin, fieldRelXMax, fieldRelYMin, fieldRelYMax)
    //      || !isPointWithinRange(vertLines[i].end, fieldRelXMin, fieldRelXMax, fieldRelYMin, fieldRelYMax)) {
    //         vertlines.erase(2);
    //      }
    // }

    // for(int i = 0; i < (int)horizLines.size(); i++) {
    //     if(!isPointWithinRange(horizLines[i].start, fieldRelXMin, fieldRelXMax, fieldRelYMin, fieldRelYMax)
    //      || !isPointWithinRange(horizLines[i].end, fieldRelXMin, fieldRelXMax, fieldRelYMin, fieldRelYMax)) {
    //         horizLines.erase(i);
    //      }
    // }
    prev_readings = points;
    out_vert = vertLines;
    out_horiz = horizLines;
}

bool WallDetector::updateLine(int i, int point_idx, 
    const std::vector<Point>& points,
    int dir,
    std::vector<Line>& lines, 
    double xmin, double xmax, 
    double ymin, double ymax, 
    double threshold, Line& resultLine) {

    Line current = lines[i];
    Point prev = prev_readings[point_idx];

    if (fabs(current.start.x - prev.x) < threshold &&
        fabs(current.end.y - prev.y) < threshold) {  // if prev is the end of the line

        if (current.direction &&
            getCoord(points[point_idx], dir) > getCoord(prev, dir)) {  //

            Point clipped = pointWithinRange(current.start, xmin, xmax, ymin, ymax);
            resultLine.start = clipped;
            resultLine.end = points[point_idx];
            resultLine.direction = true;

        } else if (!current.direction &&
                   getCoord(points[point_idx], dir) < getCoord(prev, dir)) {

            Point clipped = pointWithinRange(current.start, xmin, xmax, ymin, ymax);
            resultLine.start = clipped;
            resultLine.end = points[point_idx];
            resultLine.direction = false;

        } else {
            resultLine = current;
            return true;
        }

    } else if (fabs(current.start.x - prev.x) < threshold &&
               fabs(current.start.y - prev.y) < threshold) {

        if (current.direction &&
            getCoord(points[point_idx], dir) < getCoord(prev, dir)) {

            Point clipped = pointWithinRange(current.end, xmin, xmax, ymin, ymax);
            resultLine.start = points[point_idx];
            resultLine.end = clipped;
            resultLine.direction = true;

        } else if (!current.direction &&
                   getCoord(points[point_idx], dir) > getCoord(prev, dir)) {

            Point clipped = pointWithinRange(current.end, xmin, xmax, ymin, ymax);
            resultLine.start = points[point_idx];
            resultLine.end = clipped;
            resultLine.direction = false;

        } else {
            resultLine = current;
            return true;
        }

    } else {
        return false;
    }

    return true;
}

double WallDetector::getCoord(const Point& p, int dir) {
    if (dir == 0) return p.x;
    return p.y;
}

Point WallDetector::pointWithinRange(Point p,
                                     double xmin, double xmax,
                                     double ymin, double ymax) {
    if (p.x > xmax) p.x = xmax;
    if (p.x < xmin) p.x = xmin;
    if (p.y > ymax) p.y = ymax;
    if (p.y < ymin) p.y = ymin;
    return p;
}

bool WallDetector::isPointWithinRange(const Point& p,
                                      double xmin, double xmax,
                                      double ymin, double ymax) {
    return (p.x > xmax || p.x < xmin || p.y > ymax || p.y < ymin);
}