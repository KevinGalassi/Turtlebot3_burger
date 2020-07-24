#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <fstream>

#include <std_srvs/Empty.h>


#define ANGULAR_VELOCITY 1
using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
int main(int argc, char** argv){

    ros::init(argc, argv, "navigation_goals");
    ros::NodeHandle n;

    geometry_msgs::Twist cmd_vel;
    std_srvs::Empty srv;

    ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    ros::ServiceClient  client = n.serviceClient<std_srvs::Empty>("/global_localization");
    ros::ServiceClient  client2 = n.serviceClient<std_srvs::Empty>("/move_base/clear");

    int no_goal = 0;

    cmd_vel.linear.x = 0;
    cmd_vel.angular.z = 0;
    geometry_msgs::Pose target_pose;
    std::vector<geometry_msgs::Pose> target_pose_list;

    ifstream dati("/home/user/tb3_ws/src/tb3_exame/src/Elenco_goal.txt");


    ros::Duration(2).sleep();
    
    ROS_INFO("Start to read the goal list");
    if (dati.good())
    {
        ROS_INFO("The file is open, start making local copy of data");
        while(!dati.eof())
        {
            dati >> target_pose.position.x;
            dati >> target_pose.position.y;
            dati >> target_pose.orientation.w;
            target_pose_list.push_back(target_pose);
            no_goal++;
        }
        dati.close();
        ROS_INFO("file closed");
    }
    else
    {
        ROS_INFO("Error during the opening of the file");
        return 1;
    }

    ros::Duration(2).sleep();

    ROS_INFO("Perform localization");
    cmd_vel.angular.z = 1;
    cmd_vel_pub.publish(cmd_vel);
    ros::Duration(30).sleep();
    cmd_vel.angular.z = 0;
    cmd_vel_pub.publish(cmd_vel);


    ROS_INFO("Startin to send navigation goal!");
    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);   
    while(!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    no_goal = target_pose_list.size();
    move_base_msgs::MoveBaseGoal goal;


    for (size_t i=0; i<no_goal; i++)
    {
    
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose = target_pose_list[i];

        ROS_INFO("Sending goal no: %lu", i+1);
        ac.sendGoal(goal);
        ROS_INFO("Copy that");
        ac.waitForResult();
        
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Turtlebot3 has reached Oscar %lu ", i+1);
        else
            ROS_INFO("The base failed to move to the %lu position for some reason", i+1);
            
    }
    
    return 0;
}