# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "general_service_2022: 3 messages, 0 services")

set(MSG_I_FLAGS "-Igeneral_service_2022:/home/zxy/general_service/src/general_service_2022/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg;-Imove_base_msgs:/opt/ros/noetic/share/move_base_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/noetic/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(general_service_2022_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/zxy/general_service/src/general_service_2022/msg/YoloResult.msg" NAME_WE)
add_custom_target(_general_service_2022_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "general_service_2022" "/home/zxy/general_service/src/general_service_2022/msg/YoloResult.msg" "std_msgs/String:sensor_msgs/Image:std_msgs/Header"
)

get_filename_component(_filename "/home/zxy/general_service/src/general_service_2022/msg/the_way_out.msg" NAME_WE)
add_custom_target(_general_service_2022_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "general_service_2022" "/home/zxy/general_service/src/general_service_2022/msg/the_way_out.msg" ""
)

get_filename_component(_filename "/home/zxy/general_service/src/general_service_2022/msg/Goals_name.msg" NAME_WE)
add_custom_target(_general_service_2022_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "general_service_2022" "/home/zxy/general_service/src/general_service_2022/msg/Goals_name.msg" "geometry_msgs/Pose:geometry_msgs/PoseStamped:geometry_msgs/Quaternion:std_msgs/Header:move_base_msgs/MoveBaseGoal:geometry_msgs/Point"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(general_service_2022
  "/home/zxy/general_service/src/general_service_2022/msg/YoloResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/general_service_2022
)
_generate_msg_cpp(general_service_2022
  "/home/zxy/general_service/src/general_service_2022/msg/the_way_out.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/general_service_2022
)
_generate_msg_cpp(general_service_2022
  "/home/zxy/general_service/src/general_service_2022/msg/Goals_name.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/move_base_msgs/cmake/../msg/MoveBaseGoal.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/general_service_2022
)

### Generating Services

### Generating Module File
_generate_module_cpp(general_service_2022
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/general_service_2022
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(general_service_2022_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(general_service_2022_generate_messages general_service_2022_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zxy/general_service/src/general_service_2022/msg/YoloResult.msg" NAME_WE)
add_dependencies(general_service_2022_generate_messages_cpp _general_service_2022_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zxy/general_service/src/general_service_2022/msg/the_way_out.msg" NAME_WE)
add_dependencies(general_service_2022_generate_messages_cpp _general_service_2022_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zxy/general_service/src/general_service_2022/msg/Goals_name.msg" NAME_WE)
add_dependencies(general_service_2022_generate_messages_cpp _general_service_2022_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(general_service_2022_gencpp)
add_dependencies(general_service_2022_gencpp general_service_2022_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS general_service_2022_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(general_service_2022
  "/home/zxy/general_service/src/general_service_2022/msg/YoloResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/general_service_2022
)
_generate_msg_eus(general_service_2022
  "/home/zxy/general_service/src/general_service_2022/msg/the_way_out.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/general_service_2022
)
_generate_msg_eus(general_service_2022
  "/home/zxy/general_service/src/general_service_2022/msg/Goals_name.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/move_base_msgs/cmake/../msg/MoveBaseGoal.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/general_service_2022
)

### Generating Services

### Generating Module File
_generate_module_eus(general_service_2022
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/general_service_2022
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(general_service_2022_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(general_service_2022_generate_messages general_service_2022_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zxy/general_service/src/general_service_2022/msg/YoloResult.msg" NAME_WE)
add_dependencies(general_service_2022_generate_messages_eus _general_service_2022_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zxy/general_service/src/general_service_2022/msg/the_way_out.msg" NAME_WE)
add_dependencies(general_service_2022_generate_messages_eus _general_service_2022_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zxy/general_service/src/general_service_2022/msg/Goals_name.msg" NAME_WE)
add_dependencies(general_service_2022_generate_messages_eus _general_service_2022_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(general_service_2022_geneus)
add_dependencies(general_service_2022_geneus general_service_2022_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS general_service_2022_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(general_service_2022
  "/home/zxy/general_service/src/general_service_2022/msg/YoloResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/general_service_2022
)
_generate_msg_lisp(general_service_2022
  "/home/zxy/general_service/src/general_service_2022/msg/the_way_out.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/general_service_2022
)
_generate_msg_lisp(general_service_2022
  "/home/zxy/general_service/src/general_service_2022/msg/Goals_name.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/move_base_msgs/cmake/../msg/MoveBaseGoal.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/general_service_2022
)

### Generating Services

### Generating Module File
_generate_module_lisp(general_service_2022
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/general_service_2022
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(general_service_2022_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(general_service_2022_generate_messages general_service_2022_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zxy/general_service/src/general_service_2022/msg/YoloResult.msg" NAME_WE)
add_dependencies(general_service_2022_generate_messages_lisp _general_service_2022_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zxy/general_service/src/general_service_2022/msg/the_way_out.msg" NAME_WE)
add_dependencies(general_service_2022_generate_messages_lisp _general_service_2022_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zxy/general_service/src/general_service_2022/msg/Goals_name.msg" NAME_WE)
add_dependencies(general_service_2022_generate_messages_lisp _general_service_2022_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(general_service_2022_genlisp)
add_dependencies(general_service_2022_genlisp general_service_2022_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS general_service_2022_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(general_service_2022
  "/home/zxy/general_service/src/general_service_2022/msg/YoloResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/general_service_2022
)
_generate_msg_nodejs(general_service_2022
  "/home/zxy/general_service/src/general_service_2022/msg/the_way_out.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/general_service_2022
)
_generate_msg_nodejs(general_service_2022
  "/home/zxy/general_service/src/general_service_2022/msg/Goals_name.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/move_base_msgs/cmake/../msg/MoveBaseGoal.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/general_service_2022
)

### Generating Services

### Generating Module File
_generate_module_nodejs(general_service_2022
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/general_service_2022
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(general_service_2022_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(general_service_2022_generate_messages general_service_2022_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zxy/general_service/src/general_service_2022/msg/YoloResult.msg" NAME_WE)
add_dependencies(general_service_2022_generate_messages_nodejs _general_service_2022_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zxy/general_service/src/general_service_2022/msg/the_way_out.msg" NAME_WE)
add_dependencies(general_service_2022_generate_messages_nodejs _general_service_2022_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zxy/general_service/src/general_service_2022/msg/Goals_name.msg" NAME_WE)
add_dependencies(general_service_2022_generate_messages_nodejs _general_service_2022_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(general_service_2022_gennodejs)
add_dependencies(general_service_2022_gennodejs general_service_2022_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS general_service_2022_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(general_service_2022
  "/home/zxy/general_service/src/general_service_2022/msg/YoloResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/general_service_2022
)
_generate_msg_py(general_service_2022
  "/home/zxy/general_service/src/general_service_2022/msg/the_way_out.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/general_service_2022
)
_generate_msg_py(general_service_2022
  "/home/zxy/general_service/src/general_service_2022/msg/Goals_name.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/move_base_msgs/cmake/../msg/MoveBaseGoal.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/general_service_2022
)

### Generating Services

### Generating Module File
_generate_module_py(general_service_2022
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/general_service_2022
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(general_service_2022_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(general_service_2022_generate_messages general_service_2022_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zxy/general_service/src/general_service_2022/msg/YoloResult.msg" NAME_WE)
add_dependencies(general_service_2022_generate_messages_py _general_service_2022_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zxy/general_service/src/general_service_2022/msg/the_way_out.msg" NAME_WE)
add_dependencies(general_service_2022_generate_messages_py _general_service_2022_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zxy/general_service/src/general_service_2022/msg/Goals_name.msg" NAME_WE)
add_dependencies(general_service_2022_generate_messages_py _general_service_2022_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(general_service_2022_genpy)
add_dependencies(general_service_2022_genpy general_service_2022_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS general_service_2022_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/general_service_2022)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/general_service_2022
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(general_service_2022_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(general_service_2022_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET move_base_msgs_generate_messages_cpp)
  add_dependencies(general_service_2022_generate_messages_cpp move_base_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/general_service_2022)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/general_service_2022
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(general_service_2022_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(general_service_2022_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET move_base_msgs_generate_messages_eus)
  add_dependencies(general_service_2022_generate_messages_eus move_base_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/general_service_2022)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/general_service_2022
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(general_service_2022_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(general_service_2022_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET move_base_msgs_generate_messages_lisp)
  add_dependencies(general_service_2022_generate_messages_lisp move_base_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/general_service_2022)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/general_service_2022
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(general_service_2022_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(general_service_2022_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET move_base_msgs_generate_messages_nodejs)
  add_dependencies(general_service_2022_generate_messages_nodejs move_base_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/general_service_2022)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/general_service_2022\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/general_service_2022
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(general_service_2022_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(general_service_2022_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET move_base_msgs_generate_messages_py)
  add_dependencies(general_service_2022_generate_messages_py move_base_msgs_generate_messages_py)
endif()
