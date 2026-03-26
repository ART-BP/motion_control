#pragma once
#include "pid.h"

class point2D{
public:
    point2D() : x(0.0), y(0.0), theta(0.0) {}

    point2D(double x, double y, double theta) : x(x), y(y), theta(theta) {}
    double x;
    double y;
    double theta;
};

class CmdVel{
public:
    CmdVel(double linear_x, double linear_y, double angular_z) : linear_x(linear_x), linear_y(linear_y), angular_z(angular_z) {}
    double linear_x;
    double linear_y;
    double angular_z;
};

class PurePursuit {
public:
    PurePursuit(double look_ahead_distance, PID* xpid, PID* ypid, PID* thetapid);

    bool isGoalReached(const point2D& current_position, const point2D& goal_position, double threshold = 0.1);

    bool isGoalReached(const point2D& current_position, const point2D& goal_position);

    bool calculateTheta(const point2D& current_position, point2D* path,\
        int path_length);

    bool findLookaheadPoint(const point2D& current_position, point2D* path,\
        int path_length, double look_ahead_distance, point2D& target_point);

    bool motionControl(const point2D& current_position, point2D* path, int path_length,
        double dt, CmdVel& cmd_vel);
private:
    double look_ahead_distance_;
    PID* xpid_;
    PID* ypid_;
    PID* thetapid_;
};
