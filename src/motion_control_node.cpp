#include "motion_control/motion_control_node.h"
#include <tf/tf.h>


MotionControlNode::MotionControlNode(double look_ahead_distance, PID* xpid, PID* ypid, PID* thetapid) {
    enable_motion_control_ = false;
    pure_pursuit_ = new PurePursuit(look_ahead_distance, xpid, ypid, thetapid);
    path_ = nullptr;
    path_length_ = 0;
    last_time_ = ros::Time::now().toSec();
}

void MotionControlNode::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    if(!enable_motion_control_) {
        return;
    }
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
    if (pure_pursuit_->motionControl(current_position_, path_, path_length_, dt, cmd_vel)) {
        geometry_msgs::Twist twist_msg;
        twist_msg.linear.x = cmd_vel.linear_x;
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
        return;
    }

    // Clean up any existing path before allocating a new one
    delete[] path_;
    path_ = new point2D[path_length_];
    for (size_t i = 0; i < path_length_; ++i) {
        path_[i].x = msg->poses[i].pose.position.x;
        path_[i].y = msg->poses[i].pose.position.y;
    }
    enable_motion_control_ = true;
    pure_pursuit_->reset_controllers();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "motion_control_node");
    ros::NodeHandle nh;

    PID xpid(1.0, 0.0, 0.1);
    PID ypid(1.0, 0.0, 0.1);
    PID thetapid(1.0, 0.0, 0.1);
    MotionControlNode motion_control_node(1.0, &xpid, &ypid, &thetapid);

    motion_control_node.cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
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
