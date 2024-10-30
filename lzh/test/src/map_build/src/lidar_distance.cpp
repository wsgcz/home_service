#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/LaserScan.h>

int frame = 1;
const int RANGE = 10;
ros::Publisher lidar_pub;
std_msgs::Float32 msg_distance;

/**
 * 发送给话题“/home_service_lidar_distance” 消息 “true”，则返回距离
 */
void request_callback(const std_msgs::String::ConstPtr& msg) {
    if (msg->data == "true") {
        frame = 0;
    }
}

void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    if (frame == 1){
        return;
    }

    int nNum = scan->ranges.size();
    float distance_sum = 0;
    float distance_average = 0;
    int number_start = (nNum / 2)  - (RANGE / 2);
    for (int i = 0; i < RANGE; i += 1){
        distance_sum += scan->ranges[number_start + i];
    }
    distance_average = distance_sum / RANGE;
    ROS_INFO("the average distance is %f", distance_average);
    msg_distance.data = distance_average;
    lidar_pub.publish(msg_distance);
    frame = 1;
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"/home_service_lidar");
    
    ROS_INFO("home_service_lidar_data start!");

    ros::NodeHandle nh;
    ros::Subscriber request_sub = nh.subscribe("/home_service_lidar_distance", 10, &request_callback);
    ros::Subscriber lidar_sub = nh.subscribe("/scan", 10, &lidarCallback);
    lidar_pub = nh.advertise<std_msgs::Float32>("/home_service", 10);
    ros::spin();
}