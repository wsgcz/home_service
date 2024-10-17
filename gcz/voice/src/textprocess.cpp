#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
ros::Publisher pub_res;
ros::Publisher pub_speak;
ros::Publisher pub_listen;
std_msgs::String msg_res;
std::vector<const char*> keywords;
std::vector<const char*> keywords_english;
void initial_keywords() {
    keywords.push_back("可乐");
    keywords.push_back("奥利奥");
    keywords.push_back("芬达");
    keywords.push_back("雪碧");
    keywords.push_back("洗手液");
    keywords.push_back("洗发水");
    keywords.push_back("水");
    keywords.push_back("薯片");
    
    keywords_english.push_back("cola");
    keywords_english.push_back("oreo");
    keywords_english.push_back("orangejuice");
    keywords_english.push_back("sprite");
    keywords_english.push_back("handwash");
    keywords_english.push_back("shampoo");
    keywords_english.push_back("water");
    keywords_english.push_back("chip");
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
        int length = keywords.size();
        for(int i = 0; i < length; i += 1) {
            if (data.find(keywords[i]) != data.npos) {
                msg_res.data=keywords_english[i];
                ss<<msg_front<<" "<<keywords[i];
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
    pub_res=nh.advertise<std_msgs::String>("general_service_object_name_return",10);
    pub_speak=nh.advertise<std_msgs::String>("/xf_tts",10);
    pub_listen=nh.advertise<std_msgs::String>("/start_listen",10);
	ros::spin();
}