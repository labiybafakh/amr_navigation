#ifndef CMD_VEL_TO_ODOM_HPP
#define CMD_VEL_TO_ODOM_HPP

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

class CmdVelToOdom {
public:
    CmdVelToOdom(ros::NodeHandle& nh);
    void update();

private:
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);

    ros::NodeHandle& nh_;
    ros::Subscriber cmd_vel_sub_;
    ros::Publisher odom_pub_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    double x_, y_, theta_;
    double linear_x_, linear_y_, angular_z_;
    ros::Time last_time_;
};

#endif