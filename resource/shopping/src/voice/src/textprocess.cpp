#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
ros::Publisher pub_res;
ros::Publisher pub_sleep;
ros::Publisher pub_speak;
ros::Publisher pub3;
std_msgs::String msg_res;
std_msgs::String msg_sleep;
void domsg(const std_msgs::StringConstPtr& msg_p){
    std_msgs::String tmpmsg;
    std::stringstream ss;
    ros::Duration du(2.2);
    std::string msg_front="i hear";
    std_msgs::String msg_start;
    msg_start.data = "startlisten";
    std::string data = msg_p->data;
    msg_sleep.data="sleep";
    if (data.find("否") != data.npos) {
        pub_sleep.publish(msg_sleep);
    } 
    else if (data.find("是") != data.npos) {
        pub_res.publish(msg_res);
        pub_sleep.publish(msg_sleep);
    } 
    else {
        if (data.find("停止")!=data.npos) {
        msg_res.data="stop";
        ss<<msg_front<<"停止";
        }
        else if (data.find("奥利奥")!=data.npos) {
            msg_res.data="oreo";
            ss<<msg_front<<" 奥利奥";
        }
        else if (data.find("芬达")!=data.npos) {
            msg_res.data="orangejuice";
            ss<<msg_front<<" 芬达";
        }
        else if (data.find("雪碧")!=data.npos) {
            msg_res.data="sprite";
            ss<<msg_front<<" 雪碧";
        }
        else if (data.find("洗手液")!=data.npos) {
            msg_res.data="handwash";
            ss<<msg_front<<" 洗手液";
        }
        else if (data.find("洗发水")!=data.npos){
            msg_res.data="shampoo";
            ss<<msg_front<<" 洗发水";
        }
        else if (data.find("水")!=data.npos){
            msg_res.data="water";
            ss<<msg_front<<" 水";
        }
        else if (data.find("面包")!=data.npos){
            msg_res.data="bread";
            ss<<msg_front<<" 面包";
        }
        else if (data.find("薯片")!=data.npos){
            msg_res.data="chip";
            ss<<msg_front<<" 薯片";
        }
        else if (data.find("乐事")!=data.npos){
            msg_res.data="lays";
            ss<<msg_front<<" 乐事";
        }
        else if (data.find("曲奇")!=data.npos){
            msg_res.data="cookie";
            ss<<msg_front<<" 曲奇";
        }
        else if (data.find("饼干")!=data.npos) {
            msg_res.data="biscuit";
            ss<<msg_front<<" 饼干";
        }
        else if (data.find("可乐")!=data.npos) {
            msg_res.data="cola";
            ss<<msg_front<<" 可乐";
        }
        ss<<" please check it";
        tmpmsg.data=ss.str();
        pub_speak.publish(tmpmsg);
        du.sleep();
        pub3.publish(msg_start);
        }
}
int main(int argc, char* argv[])
{	
	setlocale(LC_ALL,"");
	ros::init(argc,argv,"textprocess");
	ros::NodeHandle nh;
    ros::Subscriber sub=nh.subscribe<std_msgs::String>("/getmsg",10,domsg);
    pub_res=nh.advertise<std_msgs::String>("/outlisten",10);
    pub_speak=nh.advertise<std_msgs::String>("/ourspeak",10);
    pub_sleep=nh.advertise<std_msgs::String>("/sleep",10);
    pub3=nh.advertise<std_msgs::String>("/start",10);
	ros::spin();
}