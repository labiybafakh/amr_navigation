#include "cmdvel_to_odom.hpp"

CmdVelToOdom::CmdVelToOdom(ros::NodeHandle& nh) : nh_(nh),
    x_(0.0), y_(0.0), theta_(0.0),
    linear_x_(0.0), linear_y_(0.0), angular_z_(0.0)
{
    cmd_vel_sub_ = nh_.subscribe("cmd_vel", 1, &CmdVelToOdom::cmdVelCallback, this);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 10);
    last_time_ = ros::Time::now();
}

void CmdVelToOdom::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    linear_x_ = msg->linear.x;
    linear_y_ = msg->linear.y;
    angular_z_ = msg->angular.z;
}

void CmdVelToOdom::update() {
    ros::Time current_time = ros::Time::now();
    double dt = (current_time - last_time_).toSec();

    // Update position and orientation from velocities data
    double delta_x = (linear_x_ * cos(theta_) - linear_y_ * sin(theta_)) * dt;
    double delta_y = (linear_x_ * sin(theta_) + linear_y_ * cos(theta_)) * dt;
    double delta_theta = angular_z_ * dt;

    x_ += delta_x;
    y_ += delta_y;
    theta_ += delta_theta;

    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);

    // Publish footprint odom and its tf
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;

    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    odom.twist.twist.linear.x = linear_x_;
    odom.twist.twist.linear.y = linear_y_;
    odom.twist.twist.angular.z = angular_z_;

    odom_pub_.publish(odom);

    geometry_msgs::TransformStamped transform_stamped;
    transform_stamped.header.stamp = current_time;
    transform_stamped.header.frame_id = "odom";
    transform_stamped.child_frame_id = "base_link";

    transform_stamped.transform.translation.x = x_;
    transform_stamped.transform.translation.y = y_;
    transform_stamped.transform.translation.z = 0.0;

    transform_stamped.transform.rotation.x = q.x();
    transform_stamped.transform.rotation.y = q.y();
    transform_stamped.transform.rotation.z = q.z();
    transform_stamped.transform.rotation.w = q.w();

    tf_broadcaster_.sendTransform(transform_stamped);

    last_time_ = current_time;
}
