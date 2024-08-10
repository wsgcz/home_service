#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
ros::Publisher pub_tts;
ros::Publisher pub_listen;
void domsg(const std_msgs::StringConstPtr& msg_p){
    std::string data = msg_p->data;
    std_msgs::String msg_listen;
    std_msgs::String msg_tts;
    msg_listen.data = "startlisten";
    ros::Duration du(2);
    if (data.compare("one") == 0 || data.compare("two") == 0 || data.compare("three") == 0) {
        msg_tts.data = "请问你需要什么";
        pub_tts.publish(msg_tts);
        du.sleep();
        pub_listen.publish(msg_listen);
    } 
    else{
        msg_tts.data = data;
        pub_tts.publish(msg_tts);       
    }
}
int main(int argc, char* argv[])
{	
	setlocale(LC_ALL,"");
	ros::init(argc,argv,"textprocess");
	ros::NodeHandle nh;
    ros::Subscriber sub=nh.subscribe<std_msgs::String>("/general_service_xfsaywords",10,domsg);
    pub_tts=nh.advertise<std_msgs::String>("/xf_tts",10);
    pub_listen = nh.advertise<std_msgs::String>("/start_listen",10);
	ros::spin();
}