#include "motion_control/motion_control_node.h"
#include <tf/tf.h>

bool MotionControlNode::enable_motion_control_ = false;


MotionControlNode::MotionControlNode(double look_ahead_distance, PID* xpid, PID* ypid, PID* thetapid) {
    pure_pursuit_ = new PurePursuit(look_ahead_distance, xpid, ypid, thetapid);
    path_ = nullptr;
    path_length_ = 0;
}

void MotionControlNode::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    if(!enable_motion_control_) {
        return;
    }
    current_position_.x = msg->pose.pose.position.x;
    current_position_.y = msg->pose.pose.position.y;
    current_position_.theta = tf::getYaw(msg->pose.pose.orientation);
    CmdVel cmd_vel(0.0, 0.0, 0.0);
    pure_pursuit_->motionControl(current_position_, path_, path_length_, 0.1, cmd_vel);

    geometry_msgs::Twist twist_msg;
    twist_msg.linear.x = cmd_vel.linear_x;
    twist_msg.linear.y = cmd_vel.linear_y;
    twist_msg.angular.z = cmd_vel.angular_z;

    if(path_ != nullptr && path_length_ > 0 &&
       pure_pursuit_->isGoalReached(current_position_, path_[path_length_ - 1])){
        twist_msg.linear.x = 0.0;
        twist_msg.linear.y = 0.0;
        twist_msg.angular.z = 0.0;
        enable_motion_control_ = false;
    }
    cmd_vel_pub_.publish(twist_msg);
}

void MotionControlNode::pathCallback(const nav_msgs::Path::ConstPtr& msg) {
    path_length_ = msg->poses.size();

    if(path_length_ <= 0) {
        if (path_ != nullptr) {
            delete[] path_;
            path_ = nullptr;
        }
        return;
    }

    delete[] path_;
    path_ = new point2D[path_length_];
    for (size_t i = 0; i < path_length_; ++i) {
        path_[i].x = msg->poses[i].pose.position.x;
        path_[i].y = msg->poses[i].pose.position.y;
    }
    pure_pursuit_->calculateTheta(current_position_, path_, path_length_);
    enable_motion_control_ = true;
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

    ros::spin();
    return 0;
}
