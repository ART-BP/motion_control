#include "motion_control/pid.h"
#include "motion_control/pure_pursuit.h"
#include <ros/ros.h>
#include <algorithm>

PurePursuit::PurePursuit(double look_ahead_distance, PID* xpid, PID* ypid, PID* thetapid) :
    look_ahead_distance_(look_ahead_distance),
    xpid_(xpid),
    ypid_(ypid),
    thetapid_(thetapid),
    last_lookahead_index_(0),
    turn_ang_threshold_(3.0) {}
    
bool PurePursuit::findLookaheadPoint(const point2D& current_position, point2D* path,
    int path_length, double look_ahead_distance, point2D& target_point) {
    if(path_length <= 0) {
        return false; // Return false if the path is empty
    }
    if (last_lookahead_index_ >= path_length) {
        last_lookahead_index_ = 0; // Reset to start if we've gone through the path
    }
    for (int i = last_lookahead_index_; i < path_length; ++i) {
        double dx = path[i].x - current_position.x;
        double dy = path[i].y - current_position.y;
        double distance = sqrt(dx * dx + dy * dy);
        if (distance >= look_ahead_distance) {
            target_point = path[i];
            last_lookahead_index_ = i; // Update the last lookahead index
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
    double distance = sqrt(dx * dx + dy * dy);
    lookahead_point.theta = atan2(dy, dx);
    double dtheta = lookahead_point.theta - current_position.theta;
    // Normalize dtheta to the range [-pi, pi]
    while (dtheta > M_PI) dtheta -= 2 * M_PI;
    while (dtheta < -M_PI) dtheta += 2 * M_PI;

    // Use PID controllers to calculate control commands
    if(xpid_ == nullptr || thetapid_ == nullptr) {
        return false; // Return false if PID controllers are not initialized
    }
    
    double control_x = xpid_->calculate(distance, dt, 1.0);
    double control_theta = thetapid_->calculate(dtheta, dt, 1.0);

    if(dtheta > turn_ang_threshold_) {
        control_x = 0.0; // If the required turn angle is large, stop linear movement to allow turning in place
    }

    cmd_vel.linear_x = control_x;
    cmd_vel.angular_z = control_theta;

    ROS_INFO("Current Position - X: %.2f, Y: %.2f, Theta: %.2f", current_position.x, current_position.y, current_position.theta);
    ROS_INFO("Lookahead Point - X: %.2f, Y: %.2f, Theta: %.2f", lookahead_point.x, lookahead_point.y, lookahead_point.theta);
    ROS_INFO("Control Output - Linear X: %.2f, Linear Y: %.2f, Angular Z: %.2f", control_x, 0.0, control_theta);
    return true;
}

void PurePursuit::reset_controllers() {
    if (xpid_) xpid_->reset();
    if (ypid_) ypid_->reset();
    if (thetapid_) thetapid_->reset();
    last_lookahead_index_ = 0;
}

point2D* PurePursuit::insertPoints(point2D* path, int& path_length) {
    if (path == nullptr || path_length <= 1) {
        return path;
    }

    const double kMinSpacing = 0.05;
    const double spacing = std::max(kMinSpacing, look_ahead_distance_ * 0.5);
    const double kEpsilon = 1e-6;

    int interpolated_length = 1;
    for (int i = 0; i < path_length - 1; ++i) {
        const double dx = path[i + 1].x - path[i].x;
        const double dy = path[i + 1].y - path[i].y;
        const double segment_length = sqrt(dx * dx + dy * dy);
        if (segment_length < kEpsilon) {
            continue;
        }

        const int segment_count = std::max(1, static_cast<int>(ceil(segment_length / spacing)));
        interpolated_length += segment_count;
    }

    if (interpolated_length == path_length) {
        return path;
    }

    point2D* interpolated_path = new point2D[interpolated_length];
    int write_index = 0;
    interpolated_path[write_index++] = path[0];

    for (int i = 0; i < path_length - 1; ++i) {
        const double dx = path[i + 1].x - path[i].x;
        const double dy = path[i + 1].y - path[i].y;
        const double dtheta = path[i + 1].theta - path[i].theta;
        const double segment_length = sqrt(dx * dx + dy * dy);
        if (segment_length < kEpsilon) {
            continue;
        }

        const int segment_count = std::max(1, static_cast<int>(ceil(segment_length / spacing)));
        for (int j = 1; j <= segment_count; ++j) {
            const double ratio = static_cast<double>(j) / segment_count;
            interpolated_path[write_index].x = path[i].x + ratio * dx;
            interpolated_path[write_index].y = path[i].y + ratio * dy;
            interpolated_path[write_index].theta = path[i].theta + ratio * dtheta;
            ++write_index;
        }
    }

    path_length = write_index;
    return interpolated_path;
}
