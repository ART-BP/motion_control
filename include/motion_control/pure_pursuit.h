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

    bool isGoalReached(const point2D& current_position, const point2D& goal_position, double threshold);

    bool isGoalReached(const point2D& current_position, const point2D& goal_position);

    bool calculateTheta(const point2D& current_position, point2D* path,\
        int path_length);

    bool findLookaheadPoint(const point2D& current_position, point2D* path,\
        int path_length, double look_ahead_distance, point2D& target_point);

    bool motionControl(const point2D& current_position, point2D* path, int path_length,
        double dt, CmdVel& cmd_vel);

    void reset_controllers();
    
    void setLookAheadDistance(double look_ahead_distance) {
        look_ahead_distance_ = look_ahead_distance;
    }

    void setXPIDParameters(double kp, double ki, double kd) {
        if (xpid_) {
            xpid_->setParameters(kp, ki, kd);
        }
    }

    void setYPIDParameters(double kp, double ki, double kd) {
        if (ypid_) {
            ypid_->setParameters(kp, ki, kd);
        }
    }

    void setThetaPIDParameters(double kp, double ki, double kd) {
        if (thetapid_) {
            thetapid_->setParameters(kp, ki, kd);
        }
    }

    point2D* insertPoints(point2D* path, int& path_length);


private:
    double look_ahead_distance_;
    double turn_ang_threshold_;
    PID* xpid_;
    PID* ypid_;
    PID* thetapid_;
    int last_lookahead_index_;
};
