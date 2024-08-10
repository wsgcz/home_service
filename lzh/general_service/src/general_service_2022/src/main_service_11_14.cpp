#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>

#include <unordered_map>
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
  kCallBack = 256,
  kError = 512,
};  // 设置机器人在比赛中的所有状态
using MoveBaseClient =
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;

static AllStates now_state{AllStates::kWaitEnter};  // 机器人当前所在状态
static int open_count{0};                           // 开门时间检测
static std::string entrance_message{""};
static std::string human_detection_success_face_message{"first"};
static std::vector<std::string> object_names;
static std::string get_it_message{"0"};
static std::string put_down_result_message{"0"};
static std::string recognition_message{"null"};
static std::string register_message{"null"};
static waterplus_map_tools::GetWaypointByName srv_name;  // 通过名字找航点
static std::vector<move_base_msgs::MoveBaseGoal> front_goals;
static std::unordered_map<const char *, int> names_to_goal_index;
static bool front_goal_finish{false};
static bool face_register_flag{false};
void EntranceCallBack(
    const std_msgs::String::ConstPtr &msg);  // 检测是否开门的回调函数
void ObjectNameReturnCallBack(
    const std_msgs::String::ConstPtr &msg);  // 得到返回物体名字的回调函数
void FaceExistCallBack(const std_msgs::String::ConstPtr
                           &msg);  // 检测摄像头是否检测到人脸的回调函数
void PersonFrontCallBack(const move_base_msgs::MoveBaseGoal &goal);
void GetItCallBack(const std_msgs::String::ConstPtr &msg);
void PutDownResultCallBack(const std_msgs::String::ConstPtr &msg);
void RecognitionCallBack(const std_msgs::String::ConstPtr &msg);
void RegisterCallBack(const std_msgs::String::ConstPtr &msg);
inline bool GoToWayPoint(ros::ServiceClient &&client_get_waypoint_name,
                         const char *position);  // 指定航点
inline bool GoToGoal(const move_base_msgs::MoveBaseGoal &goal);

int main(int argc, char *argv[]) {
  setlocale(LC_ALL, "");  // 保证输出不乱码

  ros::init(argc, argv, "main_service");  // 节点名无特殊理由设为文件名
  ros::NodeHandle node_handle;

  int current_person_index{0};
  int grab_count{0};
  int recognition_count{0};
  int put_count{0};
  int max_num{3};
  int recognition_index{0};
  bool recognition_success{false};
  std::array<std::string, 3> person_names;
  std::stringstream splice_status_and_number_of_people;
  std_msgs::String speech_recognition_state;
  std_msgs::String face_detection_message;
  std::string face_detection_cache;
  speech_recognition_state.data = "0";
  std_msgs::String upload_image_state;
  upload_image_state.data = "-1";
  std_msgs::String human_detection_switch;
  std::unordered_map<int, std::string> grab_count_to_names;
  std::unordered_map<std::string, int> new_position;
  std::vector<int> done_index;
  std::vector<int> is_done;
  front_goals.clear();
  std::string cache = "Alice";
  grab_count_to_names[0] = cache;
  cache = "Bob";
  grab_count_to_names[1] = cache;
  cache = "Carol";
  grab_count_to_names[2] = cache;

  ros::Publisher speech_recognition_state_publisher{
      node_handle.advertise<std_msgs::String>("/general_service_xfsaywords",
                                              100)};
  ros::Publisher general_service_need_publisher{
      node_handle.advertise<std_msgs::String>("general_service_need", 1)};
  ros::Publisher general_service_put_down_publisher{
      node_handle.advertise<std_msgs::String>("genenal_service_put_down", 10)};
  ros::Publisher human_detection_switch_publisher{
      node_handle.advertise<std_msgs::String>(
          "general_service_human_detection_switch", 100)};
  ros::Publisher face_detection_publisher{
      node_handle.advertise<std_msgs::String>("general_service_face_detection",
                                              10)};
  ros::Publisher open_yolo_publisher{
      node_handle.advertise<std_msgs::String>("general_service_open_yolo", 10)};
  ros::Subscriber entrance_subcriber{
      node_handle.subscribe("/wpb_home/entrance_detect", 10, EntranceCallBack)};
  ros::Subscriber person_front_subscriber{
      node_handle.subscribe("/person/waypoint", 100, PersonFrontCallBack)};
  ros::ServiceClient client_get_waypoint_name{
      node_handle.serviceClient<waterplus_map_tools::GetWaypointByName>(
          "/waterplus/get_waypoint_name")};
  ros::Subscriber object_name_return_subscriber{node_handle.subscribe(
      "general_service_object_name_return", 10, ObjectNameReturnCallBack)};
  ros::Subscriber genenal_service_get_it_subscriber{
      node_handle.subscribe("genenal_service_get_it", 10, GetItCallBack)};
  ros::Subscriber general_service_put_down_result_subscriber{
      node_handle.subscribe("genenal_service_put_down_result", 10,
                            PutDownResultCallBack)};
  ros::Subscriber recognition_subscriber{node_handle.subscribe(
      "general_service_recognition", 10, RecognitionCallBack)};
  ros::Subscriber register_subscriber{node_handle.subscribe(
      "general_service_register_face", 10, RegisterCallBack)};

  ros::Rate rate(40.0);  // 没有特殊理由，项目频率一律设为 40.0 HZ
  MoveBaseClient ac("move_base", true);

  ROS_INFO("main_service 节点启动.....");
  while (ros::ok()) {
    if (now_state == AllStates::kWaitEnter) {
      if (open_count > 20) {
        bool success{GoToWayPoint(std::move(client_get_waypoint_name), "wait")};
        if (success) {
          now_state = AllStates::kWaitRoom;
        }
      }
    } else if (now_state == AllStates::kWaitRoom) {
      ROS_INFO("wait 2 s");
      ros::Duration wait_time(2.0);
      bool success{
          GoToWayPoint(std::move(client_get_waypoint_name), "persons")};
      if (success) {
        now_state = AllStates::kExplore;
        ROS_INFO("start explore");
      }
    } else if (now_state == AllStates::kExplore) {
      if (!front_goal_finish) {
        std_msgs::String message;
        message.data = human_detection_success_face_message;
        human_detection_switch_publisher.publish(message);
      } else {
        max_num = front_goals.size();
        is_done.resize(max_num);
        for (int i{0}; i < max_num; ++i) {
          is_done[i] = 0;
        }

        ac.waitForServer();
        ac.sendGoal(front_goals[current_person_index]);
        ac.waitForResult();
        while (ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
          ac.waitForServer();
          ac.sendGoal(front_goals[current_person_index]);
          ac.waitForResult();
        }
        ROS_INFO("到达第 %d 个目标", current_person_index);
        now_state = AllStates::kCollect;
      }
      // TODO: 进入后启动视觉识别，并慢速移动，直到找到目标，然后转为 near
    } else if (now_state == AllStates::kNear) {
      ROS_INFO("当前 main_service 状态为 near");
      // TODO: 靠近目标，使得摄像头正对人脸，转为 collect
    } else if (now_state == AllStates::kCollect) {
      ROS_INFO("当前 main_service 状态为 collect");
      if (current_person_index == 0) {
        speech_recognition_state.data = std::string("one");
      } else if (current_person_index == 1) {
        speech_recognition_state.data = std::string("two");
      } else if (current_person_index == 2) {
        speech_recognition_state.data = std::string("three");
      } else {
        ROS_INFO("当前 current_person_index为 %d", current_person_index);
        continue;
      }
      ROS_INFO("collect 要发送 %s", speech_recognition_state.data.c_str());
      speech_recognition_state_publisher.publish(speech_recognition_state);
      ROS_INFO("语音识别和人脸信息采集开启");
      if (current_person_index == 0) {
        face_detection_cache = "detection";
        face_detection_cache += "1";
        face_detection_message.data = face_detection_cache;
        face_detection_publisher.publish(face_detection_message);
        if (register_message.compare("success") != 0) {
          now_state = AllStates::kCallBack;
        }
      } else if (current_person_index == 1) {
        face_detection_cache = "detection";
        face_detection_cache += "2";
        face_detection_message.data = face_detection_cache;
        face_detection_publisher.publish(face_detection_message);
        if (register_message.compare("success") != 0) {
          now_state = AllStates::kCallBack;
        }
      } else if (current_person_index == 2) {
        face_detection_cache = "detection";
        face_detection_cache += "3";
        face_detection_message.data = face_detection_cache;
        face_detection_publisher.publish(face_detection_message);
        if (register_message.compare("success") != 0) {
          now_state = AllStates::kCallBack;
        }
      }
      // upload_image_state.data = std::to_string(current_person_index);
      // upload_image_state_publisher.publish(upload_image_state);
      now_state = AllStates::kCallBack;
      continue;
    } else if (now_state == AllStates::kGrab) {
      ROS_INFO("当前 main_service 状态为 grab");
      GoToWayPoint(std::move(client_get_waypoint_name), "targets");
      std::string open_yolo_str{"on"};
      std_msgs::String open_yolo_message;
      open_yolo_message.data = open_yolo_str;
      open_yolo_publisher.publish(open_yolo_message);
      std_msgs::String object_need;
      object_need.data = object_names[grab_count];
      // object_need.data = "bottle";  // only test grab, please delete it while
      // real run
      ros::Rate(2).sleep();
      general_service_need_publisher.publish(object_need);
      ROS_INFO("发送完成第%d个客人的物品: %s", grab_count + 1,
               object_need.data.c_str());
      now_state = AllStates::kCallBack;
      continue;
      // TODO: 向物品抓取server请求，然后转为 return
      now_state = AllStates::kCallBack;
    } else if (now_state == AllStates::kReturn) {
      ROS_INFO("当前 main_service 状态为 return");
      // TODO: 去委托人所在房间后开启人脸识别找到委托人，然后转为 put
    } else if (now_state == AllStates::kPut) {
      ROS_INFO("当前 main_service 状态为 put");
      for (auto key_value : new_position) {
        ROS_INFO("%s: %d", key_value.first.c_str(), key_value.second);
      }
      // if (new_position.count(grab_count_to_names[grab_count]) != 0) {
      ROS_INFO("当前要找的人是 %s", grab_count_to_names[grab_count].c_str());
      if (new_position.find(grab_count_to_names[grab_count]) !=
          new_position.end()) {
        recognition_index = new_position[grab_count_to_names[grab_count]];
        GoToGoal(front_goals[recognition_index]);
        ++grab_count;
        recognition_message = "null";
        const char *the_always = "please take your ";
        char str_the_word_to_say[40];
        strcpy(str_the_word_to_say, the_always);
        strcat(str_the_word_to_say, object_names[put_count].c_str());
        const char *the_end = "\0";
        strcat(str_the_word_to_say, the_end);
        speech_recognition_state.data = str_the_word_to_say;
        ++put_count;
        ROS_INFO("put 要发送 %s", speech_recognition_state.data.c_str());
        speech_recognition_state_publisher.publish(speech_recognition_state);
        std_msgs::String put_down_message;
        put_down_message.data = "1";
        general_service_put_down_publisher.publish(put_down_message);
        now_state = AllStates::kCallBack;
        continue;
      } else {
        ROS_INFO("没在库里识别到 %s", grab_count_to_names[grab_count].c_str());
        recognition_index = 0;
        for (int i{0}; i < max_num; ++i) {
          if (is_done[i] == 0) {
            recognition_index = i;
            break;
          }
        }
        GoToGoal(front_goals[recognition_index]);
        face_detection_cache = "recognition";
        face_detection_message.data = face_detection_cache;
        face_detection_publisher.publish(face_detection_message);
        now_state = AllStates::kCallBack;
        continue;
      }

    } else if (now_state == AllStates::kStop) {
      ROS_INFO("当前 main_service 状态为 stop");
      GoToWayPoint(std::move(client_get_waypoint_name), "exit");
      ROS_INFO("机器人出场完成");
      break;
      // TODO: 机器人停留
    } else if (now_state == AllStates::kCallBack) {
      // continue
      // ROS_INFO("当前正在等待语音结果");
      if (register_message.compare("success") == 0) {
        register_message = "null";
        face_detection_cache = "off";
        face_detection_message.data = face_detection_cache;
        face_detection_publisher.publish(face_detection_message);
        face_register_flag = true;
      } else {
        if (register_message.compare("null") != 0) {
          register_message = "null";
          face_detection_publisher.publish(face_detection_message);
        }
      }
      if (object_names.size() == current_person_index + 1 &&
          face_register_flag) {
        ROS_INFO("收到语音识别结果，第 %d 个客人需要 %s",
                 current_person_index + 1,
                 object_names[current_person_index].c_str());
        face_register_flag = false;
        person_names[current_person_index] = object_names[current_person_index];
        ++current_person_index;
        if (current_person_index >= max_num) {
          now_state = AllStates::kGrab;
        } else if (current_person_index < 0) {
          ROS_WARN("current_person_index 的值出现异常，建议立即进行检查");
        } else {
          // TODO: 操作机器人转向使其远离已采集客人，防止重复
          // person_names[current_person_index] =
          // object_names[current_person_index];
          now_state = AllStates::kExplore;
        }
      }
      if (put_down_result_message.compare("1") == 0) {
        put_down_result_message = "0";
        if (grab_count == max_num) {
          ROS_INFO("完成全部抓取，准备出场");
          now_state = AllStates::kStop;
        } else {
          ROS_INFO("送完了第%d个物品", grab_count);
          now_state = AllStates::kGrab;
        }
        continue;
      }
      if (get_it_message.compare("1") == 0) {
        get_it_message = "0";
        now_state = AllStates::kPut;
        ROS_INFO("抓取完成，正在去客人所在位置");
        GoToWayPoint(std::move(client_get_waypoint_name), "persons");
        continue;
      }
      if (recognition_message.compare("Alice") == 0 ||
          recognition_message.compare("Bob") == 0 ||
          recognition_message.compare("Carol") == 0) {
        // const char *name{recognition_message.c_str()};
        ROS_INFO("当前人是 %s", recognition_message.c_str());
        new_position[recognition_message] = recognition_index;
        ROS_INFO("%s 存入了 %d", recognition_message.c_str(), recognition_index);
        is_done[recognition_index] = 1;
        if (std::strcmp(grab_count_to_names[grab_count].c_str(), recognition_message.c_str()) == 0) {
          ++grab_count;
          recognition_message = "null";
          const char *the_always = "please take your ";
          char str_the_word_to_say[40];
          strcpy(str_the_word_to_say, the_always);
          strcat(str_the_word_to_say, object_names[put_count].c_str());
          const char *the_end = "\0";
          strcat(str_the_word_to_say, the_end);
          speech_recognition_state.data = str_the_word_to_say;
          ++put_count;
          ROS_INFO("put 要发送 %s", speech_recognition_state.data.c_str());
          speech_recognition_state_publisher.publish(speech_recognition_state);
          std_msgs::String put_down_message;
          put_down_message.data = "1";
          general_service_put_down_publisher.publish(put_down_message);
          now_state = AllStates::kCallBack;
          continue;
        } else {
          ++recognition_index;
          if (recognition_index > max_num - 1) {
            recognition_index = 0;
          }
          if (is_done[recognition_index] == 1) {
            ++recognition_index;
            if (recognition_index > max_num - 1) {
              recognition_index = 0;
            }
          } else {
            GoToGoal(front_goals[recognition_index]);
            recognition_message = "null";
            face_detection_cache = "recognition";
            face_detection_message.data = face_detection_cache;
            face_detection_publisher.publish(face_detection_message);
            now_state = AllStates::kCallBack;
            continue;
          }
        }
      }
      if (recognition_message.compare("unknown") == 0) {
        recognition_message = "null";
        face_detection_cache = "recognition";
        face_detection_message.data = face_detection_cache;
        face_detection_publisher.publish(face_detection_message);
        now_state = AllStates::kCallBack;
        continue;
      }
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

void ObjectNameReturnCallBack(const std_msgs::String::ConstPtr &msg) {
  if (msg->data == "false") {
  } else {
    object_names.emplace_back(msg->data);
  }
}

void GetItCallBack(const std_msgs::String::ConstPtr &msg) {
  if (get_it_message.compare("0") == 0) {
    ROS_INFO("收到 get_it 为 1, 状态转为 put");
    get_it_message = msg->data;
  } else {
    ROS_INFO("接受get_it话题错误，发过来的时候get_it_message不是0");
  }
}

void PutDownResultCallBack(const std_msgs::String::ConstPtr &msg) {
  if (put_down_result_message.compare("0") == 0) {
    ROS_INFO("收到 put_down_result 为 1, 状态转为 stop 或 grab");
    put_down_result_message = msg->data;
  } else {
    ROS_INFO(
        "接受put_down_result话题错误，发过来的时候put_down_result_"
        "message不是0");
  }
}

void PersonFrontCallBack(const move_base_msgs::MoveBaseGoal &goal) {
  if (goal.target_pose.header.frame_id.compare("base_link") == 0) {
    front_goal_finish = true;
  } else {
    front_goals.emplace_back(goal);
  }
}

void RecognitionCallBack(const std_msgs::String::ConstPtr &msg) {
  recognition_message = msg->data;
  ROS_INFO("main: 收到 recognition: %s", recognition_message.c_str());
}

void RegisterCallBack(const std_msgs::String::ConstPtr &msg) {
  register_message = msg->data;
  ROS_INFO("main: %s", register_message.c_str());
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

inline bool GoToGoal(const move_base_msgs::MoveBaseGoal &goal) {
  MoveBaseClient ac("move_base", true);
  ac.waitForServer();
  ac.sendGoal(goal);
  ac.waitForResult();
  while (ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
    ac.waitForServer();
    ac.sendGoal(goal);
    ac.waitForResult();
  }
  return true;
}