#include "cmdvel_to_odom.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "odom_publisher");
    ros::NodeHandle nh;

    CmdVelToOdom odom_publisher(nh);

    ros::Rate rate(10);
    while (ros::ok()) {
        odom_publisher.update();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}