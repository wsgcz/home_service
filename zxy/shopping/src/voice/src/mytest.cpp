#include <ros/ros.h>
#include "std_msgs/String.h"
#include <string>
using namespace std;
ros::Publisher pub;
void speak(string str){
    std_msgs::String msg;
    msg.data=str;
    pub.publish(msg);
}
int main(int argc, char* argv[])
{
	ros::init(argc,argv,"mytest");
	ros::NodeHandle nd;
    pub=nd.advertise<std_msgs::String>("/ourspeak",10);
    ros::Duration du(1);
    du.sleep();
    ROS_INFO("i want sleep");
    speak("i am sleep");
    ROS_INFO("i said something");
    ros::spin();
	return 0;
}