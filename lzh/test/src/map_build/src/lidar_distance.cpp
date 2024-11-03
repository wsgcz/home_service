#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/LaserScan.h>

int frame = 1;
const int RANGE = 3;
ros::Publisher lidar_pub;
std_msgs::Float32 msg_distance;

/**
 * 发送给话题“/home_service_lidar_distance” 消息 “true”，则返回距离
 */
void request_callback(const std_msgs::String::ConstPtr& msg) {
    if (msg->data == "true") {
        frame = 0;
        printf("----------lidar get message-----------\n");
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
    int count = 0;
    int number_start = (nNum / 2)  - (RANGE / 2);
    for (int i = 0; i < RANGE; i += 1){
        if (!(scan->ranges[number_start + i] < 2 && scan->ranges[number_start + i] > 0)){
            continue;
        }
        distance_sum += scan->ranges[number_start + i];
        count += 1;
        printf("-----distance %d is %f", number_start + i, scan->ranges[number_start + i]);
    }
    if (count == 0) {
        msg_distance.data = 55;
        lidar_pub.publish(msg_distance);
    }
    else {
        distance_average = distance_sum / count;
        ROS_INFO("-----------the average distance is %f------------", distance_average);
        msg_distance.data = distance_average;
        lidar_pub.publish(msg_distance);
    }
    frame = 1;
    
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"/home_service_lidar");
    
    ROS_INFO("home_service_lidar_data start!");

    ros::NodeHandle nh;
    ros::Subscriber request_sub = nh.subscribe("/home_service/lidar_distance", 10, &request_callback);
    ros::Subscriber lidar_sub = nh.subscribe("/scan", 10, &lidarCallback);
    lidar_pub = nh.advertise<std_msgs::Float32>("/home_service/robot_getdistance", 10);
    printf("i have pub lidar_pub");
    ros::spin();
}