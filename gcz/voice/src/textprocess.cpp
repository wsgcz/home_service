#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <map>
ros::Publisher pub_res;
ros::Publisher pub_speak;
ros::Publisher pub_listen;
std_msgs::String msg_res;
std::map<const char*,const char*> keywords_map;

void initial_keywords() {
    keywords_map["可乐"] = "cola";
    keywords_map["奥利奥"] = "oreo";
    keywords_map["芬达"] = "orangejuice";
    keywords_map["雪碧"] = "sprite";
    keywords_map["洗手液"] = "handwash";
    keywords_map["洗发水"] = "shampoo";
    keywords_map["可乐"] = "cola";
    keywords_map["水"] = "water";
    keywords_map["薯片"] = "chip";
}

void domsg(const std_msgs::StringConstPtr& msg_p){

    std_msgs::String tmpmsg;
    std::stringstream ss;
    ros::Duration du(2.2);
    std::string msg_front="i hear";
    std_msgs::String msg_listen;
    msg_listen.data = "startlisten";
    std::string data = msg_p->data;
    if (data.find("否") != data.npos) {
        pub_listen.publish(msg_listen);
    } 
    else if (data.find("是") != data.npos) {
        pub_res.publish(msg_res);
    } 
    else {
        for (std::map<const char *,const char *>::iterator it=keywords_map.begin(); it!=keywords_map.end(); ++it) {
            if (data.find(it->first) != data.npos) {
                msg_res.data=it->second;
                ss<<msg_front<<" "<<it->first;
            }
        }
        ss<<" please check it";
        tmpmsg.data=ss.str();
        pub_speak.publish(tmpmsg);
        du.sleep();
        pub_listen.publish(msg_listen);
    }
}

int main(int argc, char* argv[])
{	
    initial_keywords();
	setlocale(LC_ALL,"");
	ros::init(argc,argv,"textprocess");
	ros::NodeHandle nh;
    ros::Subscriber sub=nh.subscribe<std_msgs::String>("/get_listen_res",10,domsg);
    pub_res=nh.advertise<std_msgs::String>("/home_service_object_name_return",10);
    pub_speak=nh.advertise<std_msgs::String>("/xf_tts",10);
    pub_listen=nh.advertise<std_msgs::String>("/start_listen",10);
	ros::spin();
}
