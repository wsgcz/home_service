# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "xf_voice: 0 messages, 1 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(xf_voice_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/zxy/general_service/src/xf_voice/srv/voice.srv" NAME_WE)
add_custom_target(_xf_voice_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xf_voice" "/home/zxy/general_service/src/xf_voice/srv/voice.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(xf_voice
  "/home/zxy/general_service/src/xf_voice/srv/voice.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xf_voice
)

### Generating Module File
_generate_module_cpp(xf_voice
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xf_voice
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(xf_voice_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(xf_voice_generate_messages xf_voice_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zxy/general_service/src/xf_voice/srv/voice.srv" NAME_WE)
add_dependencies(xf_voice_generate_messages_cpp _xf_voice_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(xf_voice_gencpp)
add_dependencies(xf_voice_gencpp xf_voice_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS xf_voice_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(xf_voice
  "/home/zxy/general_service/src/xf_voice/srv/voice.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xf_voice
)

### Generating Module File
_generate_module_eus(xf_voice
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xf_voice
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(xf_voice_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(xf_voice_generate_messages xf_voice_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zxy/general_service/src/xf_voice/srv/voice.srv" NAME_WE)
add_dependencies(xf_voice_generate_messages_eus _xf_voice_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(xf_voice_geneus)
add_dependencies(xf_voice_geneus xf_voice_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS xf_voice_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(xf_voice
  "/home/zxy/general_service/src/xf_voice/srv/voice.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xf_voice
)

### Generating Module File
_generate_module_lisp(xf_voice
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xf_voice
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(xf_voice_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(xf_voice_generate_messages xf_voice_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zxy/general_service/src/xf_voice/srv/voice.srv" NAME_WE)
add_dependencies(xf_voice_generate_messages_lisp _xf_voice_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(xf_voice_genlisp)
add_dependencies(xf_voice_genlisp xf_voice_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS xf_voice_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(xf_voice
  "/home/zxy/general_service/src/xf_voice/srv/voice.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xf_voice
)

### Generating Module File
_generate_module_nodejs(xf_voice
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xf_voice
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(xf_voice_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(xf_voice_generate_messages xf_voice_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zxy/general_service/src/xf_voice/srv/voice.srv" NAME_WE)
add_dependencies(xf_voice_generate_messages_nodejs _xf_voice_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(xf_voice_gennodejs)
add_dependencies(xf_voice_gennodejs xf_voice_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS xf_voice_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(xf_voice
  "/home/zxy/general_service/src/xf_voice/srv/voice.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xf_voice
)

### Generating Module File
_generate_module_py(xf_voice
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xf_voice
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(xf_voice_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(xf_voice_generate_messages xf_voice_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zxy/general_service/src/xf_voice/srv/voice.srv" NAME_WE)
add_dependencies(xf_voice_generate_messages_py _xf_voice_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(xf_voice_genpy)
add_dependencies(xf_voice_genpy xf_voice_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS xf_voice_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xf_voice)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xf_voice
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(xf_voice_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xf_voice)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xf_voice
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(xf_voice_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xf_voice)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xf_voice
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(xf_voice_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xf_voice)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xf_voice
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(xf_voice_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xf_voice)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xf_voice\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xf_voice
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(xf_voice_generate_messages_py std_msgs_generate_messages_py)
endif()
