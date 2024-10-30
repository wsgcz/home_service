#include <ros/ros.h>
#include <string>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/tf.h>
#include <map>
#include <tf/transform_listener.h>

using namespace std;
using std::map;
static ros::ServiceClient cliGetWPName;
static ros::Publisher marker_pub;
static ros::Publisher vel_pub;
static ros::Publisher waypoint_add;


void reset_vel(geometry_msgs::Twist& vel_cmd) {
    vel_cmd.linear.x = 0;
    vel_cmd.angular.z= 0;
    vel_pub.publish(vel_cmd);
}
int main(int argc, char** argv) {
    ros::init(argc, argv, "vel_crtl");
    ros::NodeHandle n;
    vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",10);
    
    ros::Rate rate = ros::Rate(0.4);
    geometry_msgs::Twist vel_cmd;
    vel_cmd.linear.x = 0;
    vel_cmd.linear.y = 0;
    vel_cmd.linear.z = 0;

    vel_cmd.angular.x = 0;
    vel_cmd.angular.y = 0;
    vel_cmd.angular.z = 0;
    while (ros::ok())
    {
        char c = getchar();
        switch (c)
        {
            case '1':
                vel_cmd.linear.x = 0.18;
                vel_pub.publish(vel_cmd);
                break;
            case '2':
                vel_cmd.linear.x = -0.18;
                vel_pub.publish(vel_cmd);
                break;
            case '3':
                vel_cmd.angular.z= 0.3;
                vel_pub.publish(vel_cmd);
                break;
            case '4':
                vel_cmd.angular.z = -0.3;
                vel_pub.publish(vel_cmd);
                break;
            default:
                break;
        }
        rate.sleep();
        reset_vel(vel_cmd);
    }    
}