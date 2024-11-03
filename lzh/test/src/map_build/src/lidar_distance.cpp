#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/LaserScan.h>

int frame = 1; //当frame为0时，向/home_service/robot_getdistance，否则只接受lidar的发布信息而不发送。设置初始值为1。
const int RANGE = 3; //取雷达前方的三个点
ros::Publisher lidar_pub;
std_msgs::Float32 msg_distance;

/**
 * 发送给话题“/home_service_lidar_distance” 消息 “true”，则返回距离
 */
void request_callback(const std_msgs::String::ConstPtr& msg) {
    if (msg->data == "true") {
        frame = 0; //将frame设置为0，表示可以发送雷达距离
        printf("----------lidar get message-----------\n");
    }
}

/**
 * 机器人雷达信息的回调函数，计算前方三个点的平均距离并向/home_service/robot_getdistance发布
 */
void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    if (frame == 1){ //如果frame==1，说明/home_service/lidar_distance向你发送他
        return;
    }

    int nNum = scan->ranges.size();//雷达发送消息总数
    float distance_sum = 0; 
    float distance_average = 0;
    int count = 0;
    int number_start = (nNum / 2)  - (RANGE / 2); //获得正前方RANGE个点的平均距离
    for (int i = 0; i < RANGE; i += 1){
        if (!(scan->ranges[number_start + i] < 2 && scan->ranges[number_start + i] > 0)){
            //如果距离过大或是过小，则不计算该点，用于避免雷达传回的inf值
            continue;
        }
        distance_sum += scan->ranges[number_start + i];
        count += 1;
        printf("-----distance %d is %f", number_start + i, scan->ranges[number_start + i]);
    }
    if (count == 0) {
        //如果说所有点都过大或者是过小，则说明机器人不是正对垃圾桶，则直接丢垃圾
        msg_distance.data = 55;
        lidar_pub.publish(msg_distance);
    }
    else {
        //返回符合条件的所有点的平均距离
        distance_average = distance_sum / count;
        ROS_INFO("-----------the average distance is %f------------", distance_average);
        msg_distance.data = distance_average;
        lidar_pub.publish(msg_distance);
    }
    frame = 1;//frame设置为1，不再发布雷达获得的距离
    
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