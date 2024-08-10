#include"ros/ros.h"
#include"std_msgs/String.h"

int main(int argc,  char *argv[])
{
    ros::init(argc,argv,"yuyin");
	ros::NodeHandle nh;
	ros::Publisher pub=nh.advertise<std_msgs::String>("/xfsaywords",10);
	std_msgs::String shuju;
	shuju.data="对不起，我听不懂";
    while(ros::ok())
        pub.publish(shuju);
    return 0;
}
