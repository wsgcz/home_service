# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "sentence_voice: 1 messages, 2 services")

set(MSG_I_FLAGS "-Isentence_voice:/home/zxy/general_service/src/sentence_voice/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(sentence_voice_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/zxy/general_service/src/sentence_voice/msg/Sentence.msg" NAME_WE)
add_custom_target(_sentence_voice_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sentence_voice" "/home/zxy/general_service/src/sentence_voice/msg/Sentence.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/zxy/general_service/src/sentence_voice/srv/Listen.srv" NAME_WE)
add_custom_target(_sentence_voice_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sentence_voice" "/home/zxy/general_service/src/sentence_voice/srv/Listen.srv" "std_msgs/Header"
)

get_filename_component(_filename "/home/zxy/general_service/src/sentence_voice/srv/Speak.srv" NAME_WE)
add_custom_target(_sentence_voice_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sentence_voice" "/home/zxy/general_service/src/sentence_voice/srv/Speak.srv" "std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(sentence_voice
  "/home/zxy/general_service/src/sentence_voice/msg/Sentence.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sentence_voice
)

### Generating Services
_generate_srv_cpp(sentence_voice
  "/home/zxy/general_service/src/sentence_voice/srv/Listen.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sentence_voice
)
_generate_srv_cpp(sentence_voice
  "/home/zxy/general_service/src/sentence_voice/srv/Speak.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sentence_voice
)

### Generating Module File
_generate_module_cpp(sentence_voice
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sentence_voice
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(sentence_voice_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(sentence_voice_generate_messages sentence_voice_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zxy/general_service/src/sentence_voice/msg/Sentence.msg" NAME_WE)
add_dependencies(sentence_voice_generate_messages_cpp _sentence_voice_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zxy/general_service/src/sentence_voice/srv/Listen.srv" NAME_WE)
add_dependencies(sentence_voice_generate_messages_cpp _sentence_voice_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zxy/general_service/src/sentence_voice/srv/Speak.srv" NAME_WE)
add_dependencies(sentence_voice_generate_messages_cpp _sentence_voice_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(sentence_voice_gencpp)
add_dependencies(sentence_voice_gencpp sentence_voice_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS sentence_voice_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(sentence_voice
  "/home/zxy/general_service/src/sentence_voice/msg/Sentence.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sentence_voice
)

### Generating Services
_generate_srv_eus(sentence_voice
  "/home/zxy/general_service/src/sentence_voice/srv/Listen.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sentence_voice
)
_generate_srv_eus(sentence_voice
  "/home/zxy/general_service/src/sentence_voice/srv/Speak.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sentence_voice
)

### Generating Module File
_generate_module_eus(sentence_voice
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sentence_voice
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(sentence_voice_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(sentence_voice_generate_messages sentence_voice_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zxy/general_service/src/sentence_voice/msg/Sentence.msg" NAME_WE)
add_dependencies(sentence_voice_generate_messages_eus _sentence_voice_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zxy/general_service/src/sentence_voice/srv/Listen.srv" NAME_WE)
add_dependencies(sentence_voice_generate_messages_eus _sentence_voice_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zxy/general_service/src/sentence_voice/srv/Speak.srv" NAME_WE)
add_dependencies(sentence_voice_generate_messages_eus _sentence_voice_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(sentence_voice_geneus)
add_dependencies(sentence_voice_geneus sentence_voice_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS sentence_voice_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(sentence_voice
  "/home/zxy/general_service/src/sentence_voice/msg/Sentence.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sentence_voice
)

### Generating Services
_generate_srv_lisp(sentence_voice
  "/home/zxy/general_service/src/sentence_voice/srv/Listen.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sentence_voice
)
_generate_srv_lisp(sentence_voice
  "/home/zxy/general_service/src/sentence_voice/srv/Speak.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sentence_voice
)

### Generating Module File
_generate_module_lisp(sentence_voice
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sentence_voice
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(sentence_voice_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(sentence_voice_generate_messages sentence_voice_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zxy/general_service/src/sentence_voice/msg/Sentence.msg" NAME_WE)
add_dependencies(sentence_voice_generate_messages_lisp _sentence_voice_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zxy/general_service/src/sentence_voice/srv/Listen.srv" NAME_WE)
add_dependencies(sentence_voice_generate_messages_lisp _sentence_voice_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zxy/general_service/src/sentence_voice/srv/Speak.srv" NAME_WE)
add_dependencies(sentence_voice_generate_messages_lisp _sentence_voice_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(sentence_voice_genlisp)
add_dependencies(sentence_voice_genlisp sentence_voice_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS sentence_voice_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(sentence_voice
  "/home/zxy/general_service/src/sentence_voice/msg/Sentence.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sentence_voice
)

### Generating Services
_generate_srv_nodejs(sentence_voice
  "/home/zxy/general_service/src/sentence_voice/srv/Listen.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sentence_voice
)
_generate_srv_nodejs(sentence_voice
  "/home/zxy/general_service/src/sentence_voice/srv/Speak.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sentence_voice
)

### Generating Module File
_generate_module_nodejs(sentence_voice
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sentence_voice
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(sentence_voice_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(sentence_voice_generate_messages sentence_voice_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zxy/general_service/src/sentence_voice/msg/Sentence.msg" NAME_WE)
add_dependencies(sentence_voice_generate_messages_nodejs _sentence_voice_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zxy/general_service/src/sentence_voice/srv/Listen.srv" NAME_WE)
add_dependencies(sentence_voice_generate_messages_nodejs _sentence_voice_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zxy/general_service/src/sentence_voice/srv/Speak.srv" NAME_WE)
add_dependencies(sentence_voice_generate_messages_nodejs _sentence_voice_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(sentence_voice_gennodejs)
add_dependencies(sentence_voice_gennodejs sentence_voice_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS sentence_voice_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(sentence_voice
  "/home/zxy/general_service/src/sentence_voice/msg/Sentence.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sentence_voice
)

### Generating Services
_generate_srv_py(sentence_voice
  "/home/zxy/general_service/src/sentence_voice/srv/Listen.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sentence_voice
)
_generate_srv_py(sentence_voice
  "/home/zxy/general_service/src/sentence_voice/srv/Speak.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sentence_voice
)

### Generating Module File
_generate_module_py(sentence_voice
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sentence_voice
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(sentence_voice_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(sentence_voice_generate_messages sentence_voice_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zxy/general_service/src/sentence_voice/msg/Sentence.msg" NAME_WE)
add_dependencies(sentence_voice_generate_messages_py _sentence_voice_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zxy/general_service/src/sentence_voice/srv/Listen.srv" NAME_WE)
add_dependencies(sentence_voice_generate_messages_py _sentence_voice_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zxy/general_service/src/sentence_voice/srv/Speak.srv" NAME_WE)
add_dependencies(sentence_voice_generate_messages_py _sentence_voice_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(sentence_voice_genpy)
add_dependencies(sentence_voice_genpy sentence_voice_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS sentence_voice_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sentence_voice)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sentence_voice
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(sentence_voice_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(sentence_voice_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sentence_voice)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sentence_voice
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(sentence_voice_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(sentence_voice_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sentence_voice)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sentence_voice
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(sentence_voice_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(sentence_voice_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sentence_voice)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sentence_voice
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(sentence_voice_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(sentence_voice_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sentence_voice)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sentence_voice\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sentence_voice
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(sentence_voice_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(sentence_voice_generate_messages_py sensor_msgs_generate_messages_py)
endif()
