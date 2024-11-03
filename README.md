# 2024RoboCup居家服务

> 获得2024年居家服务机器人亚军

> 联系方式：qq群`683244882`

## 成员分工

### 视觉识别

曾子颖，刘锦山

---

### 主函数

李泽辉

---

### 抓取

王梦瑶

---

### 语音模块，建图导航

郭陈谆

## 仓库架构

### grabth
抓取功能包，实现抓取和丢垃圾

### iai_kinect2
将kinect2相机获得的消息封装为ros消息

### main_function
主函数

### realsense-ros
将realsense相机获得的消息封装为ros消息

### rplidar_ros
将雷达获得的消息封装为ros消息

### sound_play
语言播报功能包

### vis
视觉功能包，能够实现人脸识别，垃圾识别，姿态识别

### voice
语言功能包，能够实现语音播报的功能

### waterplus_map_tools
航点功能包，能够实现航点的创建以及保存

### wpb_home && wpb_home_apps
六部工坊机器人控制功能包，可以通过ros向话题发布实现对机器人的控制

### xfyun_waterplus
六部工坊封装的语音功能包
