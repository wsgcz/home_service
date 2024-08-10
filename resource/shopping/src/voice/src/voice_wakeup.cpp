#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include "../include/msp_cmn.h"
#include "../include/msp_errors.h"
#include "asr_record.h"
#include "awaken.h"
#include "play_audio.h"

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>

ros::Publisher wakeup_pub;
ros::Publisher pub;
const char *lgi_param = "appid = 8e6b9e2e,work_dir = .";
const char *ssb_param = "ivw_threshold=0:1450,sst=wakeup,ivw_res_path =fo|/home/gcz/shopping/src/voice/bin/msc/res/ivw/wakeupresource.jet";

int16_t g_order = ORDER_NONE;
BOOL g_is_order_publiced = FALSE;

#define MAX_SIZE 100


void wakeupcode()
{
  int ret = 0 ;
  std_msgs::String msg;
  std_msgs::String msg2;
	msg.data="startlisten";
  ret = MSPLogin(NULL, NULL, lgi_param);
  if (MSP_SUCCESS != ret)
  {
    printf("MSPLogin failed, error code: %d.\n", ret);
    goto exit ;//登录失败，退出登录
  }
  while (1)
  {
    run_ivw(NULL, ssb_param); 
    printf("finish run_ivw\n");
    if(g_is_awaken_succeed){
      // msg2.data="i am up";
      // pub.publish(msg2);
      // ros::Duration du(2);
      // du.sleep();
      wakeup_pub.publish(msg);
      g_is_awaken_succeed = FALSE;
      break;
    }
  }
exit:
  MSPLogout();
}
void domsg(const std_msgs::StringConstPtr& msg_p){
	if(msg_p->data=="sleep"){
		ROS_INFO("i am sleeping");
    std_msgs::String str1;
    str1.data="i am sleeping";
    pub.publish(str1);
		wakeupcode();
	}
}
int main(int argc,char **argv)
{
    ros::init(argc,argv,"voice_wakeup");
    ros::NodeHandle n;
    wakeup_pub = n.advertise<std_msgs::String>("/start",10);   
    ros::Subscriber sub=n.subscribe<std_msgs::String>("/sleep",10,domsg);
		pub=n.advertise<std_msgs::String>("/ourspeak",10);
    wakeupcode();
	  ros::spin();
    return 0;
}
