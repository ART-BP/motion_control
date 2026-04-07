#pragma once
#include "pure_pursuit.h"
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>

class MotionControlNode {
public:
    bool enable_motion_control_;
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber path_sub_;
    ros::Timer control_timer_;
    ros::Publisher marker_pub_;

    
    MotionControlNode(double look_ahead_distance, PID* xpid, PID* ypid, PID* thetapid);
    ~MotionControlNode() {
        delete pure_pursuit_;
        delete[] path_;
    }
    void pathCallback(const nav_msgs::Path::ConstPtr& msg);
    void controlCallback(const ros::TimerEvent& event);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    
    void visualizePath(const point2D* path, int& path_length);

private:
    point2D current_position_;
    point2D* path_;
    int path_length_;
    PurePursuit* pure_pursuit_;
    double last_time_;
    double odom_time_;
    CmdVel last_cmd_;

    double max_lin_x_;
    double max_ang_z_;
    double max_lin_acc_;
    double max_ang_acc_;
    double turn_ang_threshold_;
    double turn_ang_max_;
    double max_lin_x_at_max_turn_;

};
