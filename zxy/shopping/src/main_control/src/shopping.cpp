#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <waterplus_map_tools/Waypoint.h>
#include <waterplus_map_tools/GetWaypointByName.h>
#include <vision/LeaderDist.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>

#include <map>
#include <vector>
#include <string>
#include <cmath>
#include <iostream>
using namespace std;
using std::map;


//**************All State**************//
#define STATE_READY 0       
#define STATE_FOLLOW 1
#define STATE_FIND_GO 2
#define STATE_GOTO 3
#define STATE_ADJUST 4
#define STATE_PLACE 5
#define STATE_BACK 6
#define STATE_LEAVE 7
#define STATE_OVER 8
#define STATE_GRASP 9
#define STATE_DETECT 10
#define STATE_DETECT_READY 11
static int nState = STATE_FOLLOW; 
static int nDelay = 0;
static int arm_ready = 0;
//***************************************************************//

//****************Map Arrays*************************************//
static map<string, bool> is_record;     // has recorded or not
static map<string, bool> is_goto;       // has gone or not
static map<string, waterplus_map_tools::Waypoint>  points;  //WayPoint recorder
//************************************************//

//**************Interaction Nodes******************************//
static ros::Subscriber voice_listener;
static ros::Publisher voice_speaker;

static ros::Publisher follow_switch;
static ros::Publisher grasp_open;

static ros::Publisher detect_open;
static ros::Subscriber detect_sub;
static ros::Subscriber grasp_sub;

static ros::Publisher arm_pub;
static ros::Subscriber place_sub;

static ros::Publisher vel_pub;
static ros::Publisher goto_switch;

static ros::Publisher adjust_switch;
static ros::Subscriber adjust_sub;

static ros::Subscriber odom_sub;
static ros::Publisher waypoint_add;
ros::ServiceClient cliGetSecondWP;
static ros::ServiceClient cliGetWPName;
static waterplus_map_tools::GetWaypointByName srvName;      //a class(service) for navigation(go to waypoint)

static ros::Subscriber front_sub;
static ros::Subscriber behind_sub;
static ros::Subscriber right_sub;
static ros::Subscriber left_sub;

static ros::ServiceClient cliGetLidarDist;
//*************************************************************//

//*******************Global Variables**************************//
static int grab_height = -1;
static int placeDH = 0.2;
// ros::param::get() 
static int place_count = 0;
static int max_place_count = 3;
static int point_nums = 0;
static float lift_v = 0.5;
static float gripper_v = 2.0;
static float person_dist = 0.5;
static float additional_dist = 0.2;

static string item_name = "None";
static sensor_msgs::JointState arm_msg;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
//*************************************************************//

//*******************Public Function**************************//
struct EulerAngles;
EulerAngles ToEulerAngles(double w, double x, double y, double z);
static string FindKeyword(int index);
void Speak(string inStr);
void FollowSwitch(bool s);
void Goto(string name);
void OpenDetect(bool);
int ReadySwitch(bool s);
//***********************************************************//

//***************Structs for Mapping*************************//
struct EulerAngles
{
    double roll, pitch, yaw;
};
EulerAngles ToEulerAngles(double w, double x, double y, double z)
{
    EulerAngles angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (w * x + y * z);
    double cosr_cosp = 1 - 2 * (x * x + y * y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (w * y - z * x);
    if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);
    return angles;
}
struct Quaternion
{
    double w, x, y, z;
};
Quaternion ToQuaternion(double yaw, double pitch, double roll) // yaw (Z), pitch (Y), roll (X)
{
    // Abbreviations for the various angular functions
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    Quaternion q;
    q.w = cy * cp * cr + sy * sp * sr;
    q.x = cy * cp * sr - sy * sp * cr;
    q.y = sy * cp * sr + cy * sp * cr;
    q.z = sy * cp * cr - cy * sp * sr;

    return q;
}
//***********************************************************//

//*******************Real Functions**************************//
void InitArmMsg(float lift_velocity = lift_v, float gripper_velocity = gripper_v){
    arm_msg.name.resize(2);
    arm_msg.position.resize(2);
    arm_msg.velocity.resize(2);
    arm_msg.name[0] = "lift";
    arm_msg.name[1] = "gripper";
    arm_msg.position[0] = 0;
    arm_msg.position[1] = 0;     
    arm_msg.velocity[0] = lift_velocity;
    arm_msg.velocity[1] = gripper_velocity;
}
void InitKeyword()
{
    is_record["biscuit"] = false;        
    is_record["chip"] = false;       
    is_record["lays"] = false;              
    is_record["bread"] = false;            
    is_record["cookie"] = false;            
    is_record["handwash"] = false;              
    is_record["dishsoap"] = false;      
    is_record["water"] = false;             
    is_record["sprite"] = false;              
    is_record["cola"] = false;          
    is_record["orange juice"] = false;          
    is_record["shampoo"] = false;           

    is_goto["biscuit"] = false;                
    is_goto["chip"] = false;          
    is_goto["lays"] = false;                
    is_goto["bread"] = false;              
    is_goto["cookie"] = false;              
    is_goto["handwash"] = false;           
    is_goto["dishsoap"] = false;       
    is_goto["water"] = false;            
    is_goto["sprite"] = false;         
    is_goto["cola"] = false;           
    is_goto["orange juice"] = false;  
    is_goto["shampoo"] = false;   
}
int AddNewWaypoint(string inStr, float dist = 0)
{
    static int point_ID=0;
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
        return -1;
    }
    
    float tx=transform.getOrigin().x();
    float ty=transform.getOrigin().y();
    tf::Stamped<tf::Pose> tf_pos=tf::Stamped<tf::Pose>(tf::Pose(transform.getRotation(),tf::Point(tx,ty,0.0)),ros::Time(0),"map");   
    //get a pose_input, a time_stamp and a frame_id to instantiate a stamped pose object.
    //the pose_input consists of a group of quaternion and a xyz-vector
    geometry_msgs::PoseStamped msg_pos;
    tf::poseStampedTFToMsg(tf_pos,msg_pos);
    waterplus_map_tools::Waypoint new_waypoint;
    new_waypoint.name=inStr;
    new_waypoint.pose=msg_pos.pose;
    if (strcmp(inStr.c_str(), "objects"))
    {
        EulerAngles euler;
        euler = ToEulerAngles(new_waypoint.pose.orientation.w, new_waypoint.pose.orientation.x, new_waypoint.pose.orientation.y, new_waypoint.pose.orientation.z);
        // euler.yaw += rotate*double(M_PI)/2;
        new_waypoint.pose.position.x += dist*cos(euler.yaw);
        new_waypoint.pose.position.y += dist*sin(euler.yaw);

        Quaternion qua;
        qua = ToQuaternion(euler.yaw, euler.pitch, euler.roll);
        new_waypoint.pose.orientation.w = qua.w;
        new_waypoint.pose.orientation.x = qua.x;
        new_waypoint.pose.orientation.y = qua.y;
        new_waypoint.pose.orientation.z = qua.z;
    }
    else return -1;
    waypoint_add.publish(new_waypoint);
    // publish to the specific topic and the subscriber will save it to a waypoint server for future using, however, it's close to us.
    // points[new_waypoint.name]=new_waypoint;
    // try to use an array to substitute the topic above
    ROS_INFO("[New Waypoint] %s ( %.2f , %.2f )", new_waypoint.name.c_str(), tx, ty);
    // point_nums = point_ID;
    return 1;
}

int GoToWaypoint(std::string strFind ){
    // 完全靠航点名称来进行导航索引，与航点ID无关
    srvName.request.name = strFind;
    if (cliGetWPName.call(srvName))
    {
        std::string name = srvName.response.name;
        float x = srvName.response.pose.position.x;
        float y = srvName.response.pose.position.y;
        ROS_INFO("[STATE_GOTO] Get_wp_name = %s (%.2f,%.2f)", strFind.c_str(), x, y);

        MoveBaseClient ac("move_base", true);
        if (!ac.waitForServer(ros::Duration(5.0)))
        {
            ROS_INFO("The move_base action server is no running. action abort...");
        }
        else
        {
            move_base_msgs::MoveBaseGoal goal;
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.pose = srvName.response.pose;
            ac.sendGoal(goal);
            ac.waitForResult();
            if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO("Arrived at %s!", strFind.c_str());
                Speak("OK. I am placing it.");
                nDelay = 0;
                return 1;
            }
            else
            {
                ROS_INFO("Failed to get to %s ...", strFind.c_str());
                Speak("Failed to go to the waypoint.");
                return -1;
            }
        }
    }
    else
    {
        ROS_ERROR("Failed to call service GetWaypointByName");
        Speak("There is no this waypoint.");
        return -1;
    }
}

int ArmMove( float height, float width){
    arm_msg.position[0] = height;
    arm_msg.position[1] = width;     // 需要读取原本的机械臂参数，然后输入相同的参数以保证不会松爪
    arm_pub.publish(arm_msg);
    arm_ready = 1;
    return 1;                       //查询机械臂就位的反馈函数

}

int BaseMove(float x_move = 0,float y_move = 0){
    MoveBaseClient ac("move_base", true);
    if (!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("The move_base action server is no running. action abort...");
    }
    else
    {
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "base_footprint";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = x_move;
        goal.target_pose.pose.position.y = y_move;
        ac.sendGoal(goal);
        ac.waitForResult();
        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
        ROS_INFO("Arrived!");
        return 1;
        }
        else
        {
        ROS_INFO("Failed to arrive!");
        return -1;
        }
    }

}

void FollowSwitch(bool s)
{
    std_msgs::Bool msg;
    if(s == true) msg.data=true;
    else msg.data=false;
    follow_switch.publish(msg);
}

int ReadySwitch(bool s){
    if(s){
        GoToWaypoint("master_person"); // Bug!!! NO secure measures.
        sleep(3.5);
        ArmMove(0.8, 0.2);
        sleep(5.5);
    }
}
void Find_Go()
{
    ros::param::get("item_name",item_name); 
    ROS_INFO("FindSub:%s",item_name);
    Speak("I find " + item_name);
    Goto(item_name);
    Speak("I'm going to" + item_name);
}
int VelMove(float x_move = 0, float time = 1){
    geometry_msgs::Twist vel_cmd;
    vel_cmd.linear.x = x_move/time;
    vel_cmd.linear.y = 0;
    vel_cmd.linear.z = 0;
    vel_cmd.angular.x = 0;
    vel_cmd.angular.y = 0;
    vel_cmd.angular.z = 0;
    vel_pub.publish(vel_cmd);
    ROS_WARN("backwards");
    sleep(time);
    vel_cmd.linear.x = 0;
    vel_cmd.linear.y = 0;
    vel_cmd.linear.z = 0;
    vel_cmd.angular.x = 0;
    vel_cmd.angular.y = 0;
    vel_cmd.angular.z = 0;
    for(int i = 0;i < 3;i++){
        vel_pub.publish(vel_cmd);
        ROS_WARN("stop");
    }

    return 1;
}
void Goto(string name)
{
    int isSucceed=0;
    int level = 0;
    ros::param::get(name, level);
    float height = 0.5+float((level-1)*0.2);
    ROS_INFO("param_get:%d",level);
    isSucceed =  GoToWaypoint(name);
    if(!isSucceed)ROS_WARN("first waypoint wrong");
    else ROS_INFO("first waypoint achieved");
    ArmMove(height, 0);
    sleep(5.5);

    string sec_WPname = name+"_person";
    isSucceed = GoToWaypoint(sec_WPname);
    if(!isSucceed)ROS_WARN("second waypoint wrong");
    else ROS_INFO("second waypoint achieved");
    arm_ready = 0;
    ArmMove(height, 0.2);
    sleep(5);

    VelMove(-0.5,1.5);
    nState = STATE_BACK;
    arm_ready = 0;
    place_count += 1;
}

void Back()
{

    if (place_count < max_place_count){
        /* 返回收银台 */
        int isSucceed=GoToWaypoint("master");
        if(isSucceed == 1 ){
            Speak("Congratulations");
            ROS_INFO("Congratulations");
        }
        else ROS_FATAL("failed while going to master!");
        nState = STATE_DETECT_READY;
    }
    else{
        //Leave();
        ArmMove(0,0);
        sleep(5.5);
        int isSucceed=GoToWaypoint("start");
        if(isSucceed == 1 ){
            Speak("Congratulations");
            ROS_INFO("Congratulations");
        }
        else ROS_FATAL("failed while going to start!");
        nState = STATE_OVER;
    }
}

void Speak(string inStr)
{
    std_msgs::String msg;
    msg.data = inStr;
    // voice_speaker.publish(msg);
    cout<<"Message sent: "<< msg.data<<endl;
}

void KeywordCB(const std_msgs::String msg) //用于字符串关键词
{
        // msg.data="stop";
        // msg.data="oreo";
        // msg.data="water";
        // msg.data="follow";
        // msg.data="biscuit";
    string strKeyword = msg.data;
    if (nState == STATE_FOLLOW)
    {
        if(msg.data=="stop")
        {
            FollowSwitch(false);                    // change the follow state
            AddNewWaypoint("master");

            ros::service::waitForService("/vision/LeaderDist");
            vision::LeaderDist ai;
            int add_flag = 0;
            add_flag = cliGetSecondWP.call(ai) + additional_dist;
            if (add_flag)
            {
                ROS_FATAL("response:%f",ai.response.distance);
            }
            else
            {
                ROS_ERROR("请求处理失败....");
            }
            person_dist = ai.response.distance>0.5? ai.response.distance:0.4 + additional_dist;
            add_flag = AddNewWaypoint("master_person",person_dist); 
            if(!add_flag) ROS_FATAL("add second WP wrong!");
            Speak("OK! I have stoped.");
            VelMove(-0.2,2);
            VelMove(0.2,2);
            // three functions above are packed for convenience
            ROS_WARN("Follow stop!");
            ROS_WARN("Follow stop!");
            ROS_WARN("Follow stop!");
            FollowSwitch(false);
            nState = STATE_DETECT_READY;
        }
        else if(strKeyword.length() > 0 && is_record[strKeyword] == false)
        {
            // 发现物品（航点）关键词
            string strKeyword = msg.data;
            string sec_WPname = msg.data+"_person";
            
            is_record[strKeyword] = true;
            int add_flag = 0;
            add_flag = AddNewWaypoint(strKeyword);
            if(!add_flag) ROS_FATAL("add first WP wrong!");

            ros::service::waitForService("/vision/LeaderDist");
            vision::LeaderDist ai;
            add_flag = cliGetSecondWP.call(ai);
            if (add_flag)
            {
                ROS_FATAL("response:%f",ai.response.distance);
            }
            else
            {
                ROS_ERROR("请求处理失败....");
            }
            
            person_dist = ai.response.distance>0.5? ai.response.distance:0.4;
            add_flag = AddNewWaypoint(sec_WPname,person_dist);
            if(!add_flag) ROS_FATAL("add second WP wrong!");

            ROS_FATAL("KeywordCB got:%s", strKeyword.c_str());
            string strSpeak = "OK. I have memoried" + strKeyword + " Next one , please";
            Speak(strSpeak);
        }
    }
}

void open_grasp(const std_msgs::Bool::ConstPtr& s)
{
    std_msgs::Bool msg;
    msg.data=true;
    ROS_INFO("-----=====STATE_GRASP");
    grasp_open.publish(msg);
    OpenDetect(false);
    nState = STATE_GRASP;
}
void OpenDetect(bool s)
{
    std_msgs::Bool msg;
    if(s == true) msg.data=true;
    else msg.data=false;
    ROS_INFO("-----=====STATE_DETECT");
    detect_open.publish(msg);
}
void Find_Go_Open(const std_msgs::Bool::ConstPtr& s)
{
    nState=STATE_FIND_GO;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "new_shopping");
    ros::NodeHandle n;
    
    // follow state
    follow_switch = n.advertise<std_msgs::Bool>("follow_switch",100);                          // no follow_sub because voice module will handle the start and stop
    voice_listener = n.subscribe<std_msgs::String>("/ourlisten",10,KeywordCB);

    // odom_sub = n.subscribe<nav_msgs::Odometry>("odom",10,OdomCB);
    waypoint_add = n.advertise<waterplus_map_tools::Waypoint>("/waterplus/add_waypoint",1);     // a transfer for the waypoint message, accord to the cliGetWPName, but not clear. can be replaced if necessary.

    // find state and grab state
    detect_open = n.advertise<std_msgs::Bool>("detect_switch",100);
    detect_sub = n.subscribe<std_msgs::Bool>("detect_finish",100,open_grasp);

    grasp_open = n.advertise<std_msgs::Bool>("easy_grasp_switch",100);
    grasp_sub = n.subscribe<std_msgs::Bool>("easy_grasp_finish",100,Find_Go_Open);
    //参数服务器中提取物品名
 
    // goto state
    cliGetWPName = n.serviceClient<waterplus_map_tools::GetWaypointByName>("/waterplus/get_waypoint_name");     // no goto_sub because waterplus_map_tools will tell us the result
    cliGetSecondWP = n.serviceClient<vision::LeaderDist>("/vision/LeaderDist");     // no goto_sub because waterplus_map_tools will tell us the result

    // speaking nodes
    voice_speaker = n.advertise<std_msgs::String>("/ourspeak",10);

    // robot arm nodes
    arm_pub = n.advertise<sensor_msgs::JointState>("/wpb_home/mani_ctrl", 30);
    vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",10);
    InitKeyword();
    InitArmMsg(0.5,2.0);
    ros::Rate r(10);
    ROS_WARN("I'm ready");
    sleep(1);
    AddNewWaypoint("start");

    while(ros::ok())
    {
        r.sleep();
        switch (nState)
            {
                case STATE_FOLLOW:
                    FollowSwitch(true);
                    break;
                case STATE_DETECT_READY:
                    ReadySwitch(true);
                    nState = STATE_DETECT;
                    break;
                case STATE_DETECT:
                    OpenDetect(true);
                    break;
                case STATE_FIND_GO:
                    Find_Go();
                    break;
                case STATE_BACK:
                    Back();
                    Speak("I'm backing to the master.");
                    // unlock process in GoToWaypoint Function.
                    break;
                case STATE_OVER:
                    Speak("Congratulations");
                    ROS_INFO("Congratulations");
                    break;
            }
    
        ros::spinOnce();
    }
    return 0;
}