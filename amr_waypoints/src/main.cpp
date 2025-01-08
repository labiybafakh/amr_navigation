#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/simple_action_client.h"
#include "tf2/LinearMath/Quaternion.h"


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

struct Waypoint{
    double x, y, theta;
};

int main(int argc, char** argv){
    ros::init(argc, argv, "WaypointsNode");
    ros::NodeHandle node_handler;

    MoveBaseClient move_base_action("move_base", true);
    
    move_base_action.waitForServer();

    std::vector<Waypoint> waypoints ={
        {0.0, 6.0, 0},
        {9.0, 1.2, -1.54},
        {11.2, -5.0, 3.14},
        {-1.8, -9.0, 1.54},
        {-7.0, -2.0, 0},
        {-8.1, 3.0, 3.14},
        {-3.0, 3.0, 1.54},
        {-5.5, 7.5, 0},
        {0.0, 6.0, 0}


    };

    tf2::Quaternion quaternion_;

    for(const auto& waypoints_ : waypoints){
        std::cout << "Goal position[x, y, theta]:" << waypoints_.x << "," << waypoints_.y << "," <<  waypoints_.theta << std::endl;

        move_base_msgs::MoveBaseGoal goal_position;

        quaternion_.setRPY(0, 0, waypoints_.theta);

        goal_position.target_pose.header.frame_id = "map";
        goal_position.target_pose.header.stamp = ros::Time::now();
        goal_position.target_pose.pose.position.x = waypoints_.x;
        goal_position.target_pose.pose.position.y = waypoints_.y;
        goal_position.target_pose.pose.orientation.x = quaternion_.getX();
        goal_position.target_pose.pose.orientation.y = quaternion_.getY();
        goal_position.target_pose.pose.orientation.z = quaternion_.getZ();
        goal_position.target_pose.pose.orientation.w = quaternion_.getW();
        
        move_base_action.sendGoal(goal_position);
        move_base_action.waitForResult();

    }

    ros::shutdown();

    return 0;
    
}