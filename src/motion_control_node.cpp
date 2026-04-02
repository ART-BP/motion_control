#include "motion_control/motion_control_node.h"
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>
#include <algorithm>
#include <cmath>

namespace {
double clampValue(double value, double low, double high) {
    return std::max(low, std::min(value, high));
}
}

MotionControlNode::MotionControlNode(double look_ahead_distance, PID* xpid, PID* ypid, PID* thetapid)
    : last_cmd_(0.0, 0.0, 0.0) {
    enable_motion_control_ = false;
    pure_pursuit_ = new PurePursuit(look_ahead_distance, xpid, ypid, thetapid);
    path_ = nullptr;
    path_length_ = 0;
    last_time_ = ros::Time::now().toSec();
    odom_time_ = ros::Time::now().toSec();
    ros::NodeHandle pnh("~");
    pnh.param("max_lin_x", max_lin_x_, 1.0);
    pnh.param("max_ang_z", max_ang_z_, 0.9);
    pnh.param("max_lin_acc", max_lin_acc_, 0.45);
    pnh.param("max_ang_acc", max_ang_acc_, 1.2);
    pnh.param("turn_ang_threshold", turn_ang_threshold_, 0.78); // ~45 degrees
    pnh.param("turn_ang_max", turn_ang_max_, 1.0);
    pnh.param("max_lin_x_at_max_turn", max_lin_x_at_max_turn_, 0.20);
}

void MotionControlNode::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    if(!enable_motion_control_) {
        return;
    }
    odom_time_ = ros::Time::now().toSec();
    current_position_.x = msg->pose.pose.position.x;
    current_position_.y = msg->pose.pose.position.y;
    current_position_.theta = tf::getYaw(msg->pose.pose.orientation);
}

void MotionControlNode::controlCallback(const ros::TimerEvent& event){
    if(!enable_motion_control_) {
        return;
    }
    double current_time = ros::Time::now().toSec();
    double dt = current_time - last_time_;
    last_time_ = current_time;

    CmdVel cmd_vel(0.0, 0.0, 0.0);
    if (current_time - odom_time_ > 0.1) {
        // 等待接收到里程计消息，或者里程计消息更新过慢，暂时不执行控制
        ROS_WARN("No recent odometry data received. Skipping control update.");
        geometry_msgs::Twist twist_msg;
        twist_msg.linear.x = cmd_vel.linear_x;
        twist_msg.linear.y = cmd_vel.linear_y;
        twist_msg.angular.z = cmd_vel.angular_z;
        cmd_vel_pub_.publish(twist_msg);
        return;
    }

    if(pure_pursuit_->isGoalReached(current_position_, path_[path_length_ - 1])) {
        // 已经到达目标点，停止运动
        ROS_INFO("Goal reached. Stopping the robot.");
        enable_motion_control_ = false;
        last_cmd_ = CmdVel(0.0, 0.0, 0.0);
        geometry_msgs::Twist twist_msg;
        twist_msg.linear.x = last_cmd_.linear_x;
        twist_msg.linear.y = last_cmd_.linear_y;
        twist_msg.angular.z = last_cmd_.angular_z;
        cmd_vel_pub_.publish(twist_msg);
        return;
    }

    if (pure_pursuit_->motionControl(current_position_, path_, path_length_, dt, cmd_vel)) {
        dt = std::max(1e-3, dt);

        // 1) Angular velocity hard clamp.
        cmd_vel.angular_z = clampValue(cmd_vel.angular_z, -max_ang_z_, max_ang_z_);

        // 2) Reduce allowed forward speed while turning to avoid rollover.
        const double abs_ang = std::abs(cmd_vel.angular_z);
        const double denom = std::max(1e-6, turn_ang_max_ - turn_ang_threshold_);
        const double turn_ratio = clampValue((abs_ang - turn_ang_threshold_) / denom, 0.0, 1.0);
        const double allowed_lin =
            max_lin_x_ - turn_ratio * std::max(0.0, max_lin_x_ - max_lin_x_at_max_turn_);
        cmd_vel.linear_x = clampValue(cmd_vel.linear_x, -allowed_lin, allowed_lin);

        // 3) Slew-rate limits to avoid abrupt acceleration / deceleration.
        const double max_dvx = max_lin_acc_ * dt;
        const double max_dwz = max_ang_acc_ * dt;
        cmd_vel.linear_x = clampValue(cmd_vel.linear_x, last_cmd_.linear_x - max_dvx, last_cmd_.linear_x + max_dvx);
        cmd_vel.angular_z = clampValue(cmd_vel.angular_z, last_cmd_.angular_z - max_dwz, last_cmd_.angular_z + max_dwz);

        last_cmd_ = cmd_vel;

        geometry_msgs::Twist twist_msg;
        twist_msg.linear.x = cmd_vel.linear_x;
        twist_msg.linear.y = cmd_vel.linear_y;
        twist_msg.angular.z = cmd_vel.angular_z;
        cmd_vel_pub_.publish(twist_msg);
    }
}

void MotionControlNode::pathCallback(const nav_msgs::Path::ConstPtr& msg) {
    ROS_INFO("Received path message with %zu poses", msg->poses.size());
    path_length_ = msg->poses.size();

    // If the path is empty, disable motion control and clean up any existing path
    if(path_length_ <= 0) {
        if (path_ != nullptr) {
            delete[] path_;
            path_ = nullptr;
        }
        enable_motion_control_ = false;
        last_cmd_ = CmdVel(0.0, 0.0, 0.0);
        return;
    }

    // Clean up any existing path before allocating a new one
    delete[] path_;
    path_ = new point2D[path_length_];
    for (size_t i = 0; i < path_length_; ++i) {
        path_[i].x = msg->poses[i].pose.position.x;
        path_[i].y = msg->poses[i].pose.position.y;
    }

    pure_pursuit_->calculateTheta(current_position_, path_, path_length_);

    point2D* interpolated_path = pure_pursuit_->insertPoints(path_, path_length_);
    if (interpolated_path != nullptr && interpolated_path != path_) {
        delete[] path_;
        path_ = interpolated_path;
    }

    ROS_INFO("Interpolated path length: %d", path_length_);
    visualizePath(path_, path_length_);
    enable_motion_control_ = true;
    pure_pursuit_->reset_controllers();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "motion_control_node");
    ros::NodeHandle nh;

    PID xpid(1.0, 0.0, 0.1);
    PID ypid(1.0, 0.0, 0.1);
    PID thetapid(10.0, 0.0, 0.1);
    MotionControlNode motion_control_node(1.0, &xpid, &ypid, &thetapid);

    motion_control_node.cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    motion_control_node.marker_pub_ = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);
    motion_control_node.odom_sub_ = nh.subscribe("odom", 10, &MotionControlNode::odomCallback, &motion_control_node);
    motion_control_node.path_sub_ = nh.subscribe("path", 10, &MotionControlNode::pathCallback, &motion_control_node);
    motion_control_node.control_timer_ = nh.createTimer(ros::Duration(0.1), &MotionControlNode::controlCallback, &motion_control_node); 
    
    // Create a timer for control updates
    ros::Duration(1.0).sleep(); 
    // 等待发布者和订阅者建立连接

    ROS_INFO("Motion Control Node is running...");

    ros::spin();
    return 0;
}


void MotionControlNode::visualizePath(const point2D* path, int& path_length){
    if(path == nullptr || path_length <= 0) {
        ROS_WARN("No path to visualize.");
        return;
    }
    std::vector<geometry_msgs::Point> points;
    for (int i = 0; i < path_length; ++i) {
        geometry_msgs::Point p;
        p.x = path[i].x;
        p.y = path[i].y;
        p.z = 0.0;
        points.push_back(p);
    }
    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = "map";
    line_strip.header.stamp = ros::Time::now();
    line_strip.ns = "path_visualization";
    line_strip.id = 0;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.scale.x = 0.1; // Line width
    line_strip.color.a = 1.0; // Alpha
    line_strip.color.r = 1.0; // Red
    line_strip.color.g = 0.0; // Green
    line_strip.color.b = 0.0; // Blue
    line_strip.points = points;

    marker_pub_.publish(line_strip);
}
