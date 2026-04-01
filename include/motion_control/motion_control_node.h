#pragma once
#include "pure_pursuit.h"
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>

class MotionControlNode {
public:
    bool enable_motion_control_;
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber path_sub_;
    
    MotionControlNode(double look_ahead_distance, PID* xpid, PID* ypid, PID* thetapid);
    ~MotionControlNode() {
        delete pure_pursuit_;
        delete[] path_;
    }
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void pathCallback(const nav_msgs::Path::ConstPtr& msg);

private:
    point2D current_position_;
    point2D* path_;
    int path_length_;
    PurePursuit* pure_pursuit_;
    double last_time_;
};