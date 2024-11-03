cmake_minimum_required(VERSION 3.0.2)
project(main_function)
add_compile_options(-std=c++14)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(
  include ${catkin_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  include
)

#catkin_python_setup("main_function.py")


catkin_package(
  CATKIN_DEPENDS  std_msgs roscpp rospy
)
include_directories(include  ${catkin_INCLUDE_DIRS}) # 指定C++头文件路径

include_directories(include ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  include
)

catkin_install_python(PROGRAMS src/main_lzh.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
