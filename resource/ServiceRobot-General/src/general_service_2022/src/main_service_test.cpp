#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>

#include <vector>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "waterplus_map_tools/GetWaypointByName.h"

enum class AllStates {
  kWaitEnter = 0,
  kWaitRoom = 1,
  kExplore = 2,
  kNear = 4,
  kCollect = 8,
  kGrab = 16,
  kReturn = 32,
  kPut = 64,
  kStop = 128,
  kError = 256,
};  // 设置机器人在比赛中的所有状态
using MoveBaseClient =
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;

static AllStates now_state{AllStates::kWaitEnter};  // 机器人当前所在状态
static int open_count{0};                           // 开门时间检测
static std::string entrance_message{""};
static std::string human_detection_success_face_message{"first"};
static waterplus_map_tools::GetWaypointByName srv_name;  // 通过名字找航点
static std::vector<move_base_msgs::MoveBaseGoal> front_goals;
static bool front_goal_finish{false};
void EntranceCallBack(
    const std_msgs::String::ConstPtr &msg);  // 检测是否开门的回调函数
void HumanDetectionSuccessFaceCallBack(const std_msgs::String::ConstPtr &msg);
void FaceExistCallBack(const std_msgs::String::ConstPtr
                           &msg);  // 检测摄像头是否检测到人脸的回调函数
void PersonFrontCallBack(const move_base_msgs::MoveBaseGoal &goal);
inline bool GoToWayPoint(ros::ServiceClient &&client_get_waypoint_name,
                         const char *position);  // 指定航点
inline bool SpeechRecognitionStatePublish(
    std_msgs::String &&speech_recognition_state,
    std::stringstream &&splice_status_and_number_of_people, const char *state,
    int current_person_index,
    ros::Publisher &&speech_recognition_state_publisher);

int main(int argc, char *argv[]) {
  setlocale(LC_ALL, "");  // 保证输出不乱码

  ros::init(argc, argv,
            "general_service_main_service");  // 节点名无特殊理由设为文件名
  ros::NodeHandle node_handle;

  int current_person_index{0};
  std::array<std::string, 3> person_names;
  std::stringstream splice_status_and_number_of_people;
  std_msgs::String speech_recognition_state;
  speech_recognition_state.data = "close 0";
  std_msgs::String upload_image_state;
  upload_image_state.data = "-1";
  std_msgs::String human_detection_switch;
  front_goals.clear();

  ros::Publisher speech_recognition_state_publisher{
      node_handle.advertise<std_msgs::String>("speech_recognition_state", 100)};
  ros::Publisher upload_image_state_publisher{
      node_handle.advertise<std_msgs::String>("upload_image_state", 100)};
  ros::Publisher human_detection_switch_publisher{
      node_handle.advertise<std_msgs::String>(
          "general_service_human_detection_switch", 100)};
  ros::Subscriber entrance_subcriber{
      node_handle.subscribe("/wpb_home/entrance_detect", 10, EntranceCallBack)};
  ros::Subscriber human_detection_success_face{
      node_handle.subscribe("general_service_human_detection_success_face", 100,
                            HumanDetectionSuccessFaceCallBack)};
  ros::Subscriber person_front_subscriber{
      node_handle.subscribe("/person/waypoint", 100, PersonFrontCallBack)};
  ros::ServiceClient client_get_waypoint_name{
      node_handle.serviceClient<waterplus_map_tools::GetWaypointByName>(
          "/waterplus/get_waypoint_name")};

  ros::Rate rate(40.0);  // 没有特殊理由，项目频率一律设为 40.0 HZ
  MoveBaseClient ac("move_base", true);

  ROS_INFO("main_service 节点启动.....");
  GoToWayPoint(std::move(client_get_waypoint_name), "persons");
  while (ros::ok()) {
    if (now_state == AllStates::kWaitEnter) {
      // if (front_goals.size() == 1) {
      //   MoveBaseClient ac("move_base", true);
      //   for (int i{0}; i < 3; ++i) {
      //     ac.waitForServer();
      //     ac.sendGoal(front_goals[i]);
      //     ac.waitForResult();
      //     while (ac.getState() !=
      //     actionlib::SimpleClientGoalState::SUCCEEDED) {
      //       ac.waitForServer();
      //       ac.sendGoal(front_goals[i]);
      //       ac.waitForResult();
      //     }
      //     ROS_INFO("到达第 %d 个目标", i);
      //   }
      // } else {
      if (!front_goal_finish) {
        std_msgs::String message;
        message.data = human_detection_success_face_message;
        human_detection_switch_publisher.publish(message);
      } else {
        for (int i{0}; i < front_goals.size(); ++i) {
          ac.waitForServer();
          ac.sendGoal(front_goals[i]);
          ac.waitForResult();
          while (ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
            ac.waitForServer();
            ac.sendGoal(front_goals[i]);
            ac.waitForResult();
          }
          ROS_INFO("到达第 %d 个目标", i);
        }
      }

      // if (open_count > 20) {
      //   bool success{GoToWayPoint(std::move(client_get_waypoint_name),
      //   "wait")}; if (success) {
      //     now_state = AllStates::kWaitRoom;
      //   }
      // }
    } else if (now_state == AllStates::kWaitRoom) {
      ROS_INFO("wait 4 s");
      ros::Duration wait_time(4.0);
      bool success{
          GoToWayPoint(std::move(client_get_waypoint_name), "persons")};
      // if (success) {
      // now_state = AllStates::kExplore;

      // ROS_INFO("start explore");
      // }
    } else if (now_state == AllStates::kExplore) {
      // 暂时还不知道怎么实现
      ros::Duration(5.0).sleep();
      now_state = AllStates::kCollect;
      // TODO: 进入后启动视觉识别，并慢速移动，直到找到目标，然后转为 near
    } else if (now_state == AllStates::kNear) {
      ROS_INFO("当前 main_service 状态为 near");
      // TODO: 靠近目标，使得摄像头正对人脸，转为 collect
    } else if (now_state == AllStates::kCollect) {
      ROS_INFO("当前 main_service 状态为 collect");
      SpeechRecognitionStatePublish(
          std::move(speech_recognition_state),
          std::move(splice_status_and_number_of_people), "open ",
          current_person_index, std::move(speech_recognition_state_publisher));
      ROS_INFO("语音识别和人脸信息采集开启");
      ros::Duration(10.0).sleep();
      // if (client_get_identified_item_name.call(speech_data)) {
      if (true) {
        SpeechRecognitionStatePublish(
            std::move(speech_recognition_state),
            std::move(splice_status_and_number_of_people), "close ",
            current_person_index + 1,
            std::move(speech_recognition_state_publisher));
        upload_image_state.data = std::to_string(current_person_index);
        upload_image_state_publisher.publish(upload_image_state);
        ++current_person_index;
        if (current_person_index >= 3) {
          now_state = AllStates::kGrab;
        } else if (current_person_index < 0) {
          ROS_WARN("current_person_index 的值出现异常，建议立即进行检查");
        } else {
          // TODO: 操作机器人转向使其远离已采集客人，防止重复
          now_state = AllStates::kExplore;
        }
      } else {
        ROS_WARN("Failed to call service SpeechRecognition");
        // TODO: 语音节点得不到物品名称，错误处理
      }
    } else if (now_state == AllStates::kGrab) {
      ROS_INFO("当前 main_service 状态为 grab");
      GoToWayPoint(std::move(client_get_waypoint_name), "targets");
      // TODO: 向物品抓取server请求，然后转为 return
      now_state = AllStates::kReturn;
    } else if (now_state == AllStates::kReturn) {
      ROS_INFO("当前 main_service 状态为 return");
      // TODO: 去委托人所在房间后开启人脸识别找到委托人，然后转为 put
    } else if (now_state == AllStates::kPut) {
      ROS_INFO("当前 main_service 状态为 put");
      // TODO: 将物品递给委托人
      // TODO: 如果三个物品全部递交，则转为 stop
      // TODO: 否则转为 grab 抓取先下一个物品
    } else if (now_state == AllStates::kStop) {
      ROS_INFO("当前 main_service 状态为 stop");
      // TODO: 机器人停留
    } else if (now_state == AllStates::kError) {
      ROS_WARN("当前 main_service 状态为 error");
      // TODO: 出现错误，调用处理函数
    } else {
      ROS_ERROR("当前 main_service 的 now_state 不合法");
      // TODO: now_state 错误状态处理
    }
    ros::spinOnce();
    rate.sleep();
  }
  ROS_INFO("main_service 节点已退出");
  return 0;
}

void EntranceCallBack(const std_msgs::String::ConstPtr &msg) {
  if (open_count > 100) {
    return;
  }
  if (open_count < 0) {
    ROS_WARN("open_count 数值明显错误");
  }

  entrance_message = msg->data;
  if (entrance_message == "door open") {
    open_count += 1;
  } else {
    open_count = 0;
  }
}

void HumanDetectionSuccessFaceCallBack(const std_msgs::String::ConstPtr &msg) {
  if (msg->data == "success") {
    if (human_detection_success_face_message == "first") {
      human_detection_success_face_message = "second";
    } else if (human_detection_success_face_message == "second") {
      human_detection_success_face_message = "third";
    } else if (human_detection_success_face_message == "third") {
      human_detection_success_face_message = "done";
    }
  }
}

void PersonFrontCallBack(const move_base_msgs::MoveBaseGoal &goal) {
  if (goal.target_pose.header.frame_id.compare("base_link") == 0) {
    front_goal_finish = true;
  } else {
    front_goals.emplace_back(goal);
  }
}

inline bool GoToWayPoint(ros::ServiceClient &&client_get_waypoint_name,
                         const char *position) {
  srv_name.request.name = position;
  if (client_get_waypoint_name.call(srv_name)) {
    ROS_INFO("get_waypoint_name: name = %s (%.2lf,%.2lf)", position,
             srv_name.response.pose.position.x,
             srv_name.response.pose.position.y);
    MoveBaseClient ac("move_base", true);
    if (!ac.waitForServer(ros::Duration(5.0))) {
      ROS_INFO("The move_base action server is no running. action abort...");
      return false;
    } else {
      move_base_msgs::MoveBaseGoal goal;
      goal.target_pose.header.frame_id = "map";
      goal.target_pose.header.stamp = ros::Time::now();
      goal.target_pose.pose = srv_name.response.pose;
      ac.sendGoal(goal);
      ac.waitForResult();
      if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Arrived at %s!", position);
        ros::spinOnce();
        return true;
      } else {
        ROS_INFO("Failed to get to %s ...", position);
        return false;
      }
    }
  } else {
    ROS_WARN("Failed to call service GetWaypointByName");
    return false;
  }
}

inline bool SpeechRecognitionStatePublish(
    std_msgs::String &&speech_recognition_state,
    std::stringstream &&splice_status_and_number_of_people, const char *state,
    int current_person_index,
    ros::Publisher &&speech_recognition_state_publisher) {
  splice_status_and_number_of_people.clear();
  splice_status_and_number_of_people << state << current_person_index;
  speech_recognition_state.data = splice_status_and_number_of_people.str();
  speech_recognition_state_publisher.publish(speech_recognition_state);
  splice_status_and_number_of_people.clear();
  return true;
}