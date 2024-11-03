#include <ros/ros.h>
#include <string>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <waterplus_map_tools/Waypoint.h>
#include "geometry_msgs/PoseStamped.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf/transform_listener.h>
#include <waterplus_map_tools/Waypoint.h>

using namespace std;
static ros::Publisher vel_pub;
static ros::Publisher waypoint_add;
tf2_ros::Buffer buffer;

/**
 * 将速度重置为0并且发布
 */
void reset_vel(geometry_msgs::Twist& vel_cmd) {
    vel_cmd.linear.x = 0;
    vel_cmd.angular.z= 0;
    vel_pub.publish(vel_cmd);
}

/**
 * 获得机器人当前在地图坐标系的pose
 */
geometry_msgs::Pose get_mappose(){
    geometry_msgs::Pose pose;

    tf::TransformListener listener;
    tf::StampedTransform transform;
    try
    {
        listener.waitForTransform("/map","base_footprint",ros::Time(0),ros::Duration(10.0));
        listener.lookupTransform("/map","base_footprint",ros::Time(0),transform);
    }
    catch(tf::TransformException &ex)
    {
        ROS_ERROR("[lookupTransformer] %s",ex.what());
    }
    
    float tx=transform.getOrigin().x();
    float ty=transform.getOrigin().y();
    tf::Stamped<tf::Pose> tf_pos=tf::Stamped<tf::Pose>(tf::Pose(transform.getRotation(),tf::Point(tx,ty,0.0)),ros::Time(0),"map");   
    //get a pose_input, a time_stamp and a frame_id to instantiate a stamped pose object.
    //the pose_input consists of a group of quaternion and a xyz-vector
    geometry_msgs::PoseStamped msg_pos;
    tf::poseStampedTFToMsg(tf_pos,msg_pos);
    return msg_pos.pose;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "home_service_waypoint_add");
    ros::NodeHandle n;

    tf2_ros::TransformListener listener(buffer);

    vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",10);
    waypoint_add = n.advertise<waterplus_map_tools::Waypoint>("waterplus/add_waypoint",10);//向waterplus模块发送添加航点的消息
    waterplus_map_tools::Waypoint newWayPoint; //航点数据类型
    newWayPoint.frame_id = "map"; //航点的坐标系为地图坐标系
    int waypoint_count = 0; //航点命名，从0开始依次累加
    ros::Rate rate = ros::Rate(0.4);
    geometry_msgs::Twist vel_cmd;
    vel_cmd.linear.x = 0;
    vel_cmd.linear.y = 0;
    vel_cmd.linear.z = 0;

    vel_cmd.angular.x = 0;
    vel_cmd.angular.y = 0;
    vel_cmd.angular.z = 0;

    /**
     * 键盘输入1，前进
     * 键盘输入2，后退
     * 键盘输入3，左转
     * 键盘输入4，右转
     * 键盘输入5，小幅度前进
     * 键盘输入6，小幅度后退
     * 键盘输入7，小幅度左转
     * 键盘输入8，小幅度右转
     * 键盘输入c，在当前位置以及机器人的朝向在地图上添加航点
     */
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
            case '5':
                vel_cmd.linear.x = 0.06;
                vel_pub.publish(vel_cmd);
                break;
            case '6':
                vel_cmd.linear.x = -0.06;
                vel_pub.publish(vel_cmd);
                break;
            case '7':
                vel_cmd.angular.z= 0.10;
                vel_pub.publish(vel_cmd);
                break;
            case '8':
                vel_cmd.angular.z = -0.10;
                vel_pub.publish(vel_cmd);
                break;
            case 'c':
                newWayPoint.name = std::to_string(waypoint_count);
                newWayPoint.pose = get_mappose();
                waypoint_add.publish(newWayPoint);
                waypoint_count += 1; //累加航点的名字
                break;
            default:
                break;
        }
        rate.sleep();
        reset_vel(vel_cmd);
    }
    return 0;    
}