#include "motion_control/pid.h"
#include "motion_control/pure_pursuit.h"
#include <ros/ros.h>

PurePursuit::PurePursuit(double look_ahead_distance, PID* xpid, PID* ypid, PID* thetapid) :
    look_ahead_distance_(look_ahead_distance), xpid_(xpid), ypid_(ypid), thetapid_(thetapid) {}

bool PurePursuit::findLookaheadPoint(const point2D& current_position, point2D* path,
    int path_length, double look_ahead_distance, point2D& target_point) {
    if(path_length <= 0) {
        return false; // Return false if the path is empty
    }
    for (int i = last_lookahead_index_; i < path_length; ++i) {
        double dx = path[i].x - current_position.x;
        double dy = path[i].y - current_position.y;
        double distance = sqrt(dx * dx + dy * dy);
        if (distance >= look_ahead_distance) {
            target_point = path[i];
            return true;
        }
    }
    target_point = path[path_length - 1]; // If no point is found, return the last point in the path
    return true;
}

bool PurePursuit::isGoalReached(const point2D& current_position, const point2D& goal_position){
    return isGoalReached(current_position, goal_position, 0.1);
}

bool PurePursuit::isGoalReached(const point2D& current_position, const point2D& goal_position, double threshold){
    double dx = goal_position.x - current_position.x;
    double dy = goal_position.y - current_position.y;
    double distance = sqrt(dx * dx + dy * dy);
    return distance < threshold; // Consider goal reached if within the specified threshold
}

bool PurePursuit::calculateTheta(const point2D& current_position, point2D* path,\
        int path_length) {
    if(path==nullptr || path_length <= 1) {
        return false; // Return false if the path is empty
    }

    path[0].theta = atan2(path[1].y - current_position.y, path[1].x - current_position.x);

    for(int i = 1; i < path_length - 1; ++i) {
        path[i].theta = atan2(path[i + 1].y - path[i - 1].y, path[i + 1].x - path[i - 1].x);
    }
    return true;
}

bool PurePursuit::motionControl(const point2D& current_position, point2D* path, int path_length,
    double dt, CmdVel& cmd_vel) {
    if(path==nullptr || path_length <= 0) {
        return false; // Return false if the path is empty
    }
    point2D lookahead_point(0.0, 0.0, 0.0);
    if (!findLookaheadPoint(current_position, path, path_length, look_ahead_distance_, lookahead_point)) {
        return false; // Return false if no valid target point is found
    }

    // Calculate the angle to the look-ahead point
    double dx = lookahead_point.x - current_position.x;
    double dy = lookahead_point.y - current_position.y;
    lookahead_point.theta = atan2(dy, dx);
    double dtheta = lookahead_point.theta - current_position.theta;
    // Normalize dtheta to the range [-pi, pi]
    while (dtheta > M_PI) dtheta -= 2 * M_PI;
    while (dtheta < -M_PI) dtheta += 2 * M_PI;

    // Use PID controllers to calculate control commands
    double control_x = xpid_->calculate(dx, dt, 1.0);
    double control_y = ypid_->calculate(dy, dt, 1.0);
    double control_theta = thetapid_->calculate(dtheta, dt, 1.0);

    cmd_vel.linear_x = control_x;
    cmd_vel.linear_y = control_y;
    cmd_vel.angular_z = control_theta;
    
    ROS_INFO("Current Position - X: %.2f, Y: %.2f, Theta: %.2f", current_position.x, current_position.y, current_position.theta);
    ROS_INFO("Lookahead Point - X: %.2f, Y: %.2f, Theta: %.2f", lookahead_point.x, lookahead_point.y, lookahead_point.theta);
    ROS_INFO("Control Output - Linear X: %.2f, Linear Y: %.2f, Angular Z: %.2f", control_x, control_y, control_theta);
    return true;
}

void PurePursuit::reset_controllers() {
    if (xpid_) xpid_->reset();
    if (ypid_) ypid_->reset();
    if (thetapid_) thetapid_->reset();
    last_lookahead_index_ = 0;
}
