# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "syn_cpp: 1 messages, 0 services")

set(MSG_I_FLAGS "-Isyn_cpp:/home/slam327/syn_ws/src/syn_cpp/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(syn_cpp_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/slam327/syn_ws/src/syn_cpp/msg/BboxData.msg" NAME_WE)
add_custom_target(_syn_cpp_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "syn_cpp" "/home/slam327/syn_ws/src/syn_cpp/msg/BboxData.msg" "std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(syn_cpp
  "/home/slam327/syn_ws/src/syn_cpp/msg/BboxData.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/syn_cpp
)

### Generating Services

### Generating Module File
_generate_module_cpp(syn_cpp
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/syn_cpp
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(syn_cpp_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(syn_cpp_generate_messages syn_cpp_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/slam327/syn_ws/src/syn_cpp/msg/BboxData.msg" NAME_WE)
add_dependencies(syn_cpp_generate_messages_cpp _syn_cpp_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(syn_cpp_gencpp)
add_dependencies(syn_cpp_gencpp syn_cpp_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS syn_cpp_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(syn_cpp
  "/home/slam327/syn_ws/src/syn_cpp/msg/BboxData.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/syn_cpp
)

### Generating Services

### Generating Module File
_generate_module_eus(syn_cpp
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/syn_cpp
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(syn_cpp_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(syn_cpp_generate_messages syn_cpp_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/slam327/syn_ws/src/syn_cpp/msg/BboxData.msg" NAME_WE)
add_dependencies(syn_cpp_generate_messages_eus _syn_cpp_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(syn_cpp_geneus)
add_dependencies(syn_cpp_geneus syn_cpp_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS syn_cpp_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(syn_cpp
  "/home/slam327/syn_ws/src/syn_cpp/msg/BboxData.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/syn_cpp
)

### Generating Services

### Generating Module File
_generate_module_lisp(syn_cpp
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/syn_cpp
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(syn_cpp_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(syn_cpp_generate_messages syn_cpp_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/slam327/syn_ws/src/syn_cpp/msg/BboxData.msg" NAME_WE)
add_dependencies(syn_cpp_generate_messages_lisp _syn_cpp_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(syn_cpp_genlisp)
add_dependencies(syn_cpp_genlisp syn_cpp_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS syn_cpp_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(syn_cpp
  "/home/slam327/syn_ws/src/syn_cpp/msg/BboxData.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/syn_cpp
)

### Generating Services

### Generating Module File
_generate_module_nodejs(syn_cpp
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/syn_cpp
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(syn_cpp_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(syn_cpp_generate_messages syn_cpp_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/slam327/syn_ws/src/syn_cpp/msg/BboxData.msg" NAME_WE)
add_dependencies(syn_cpp_generate_messages_nodejs _syn_cpp_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(syn_cpp_gennodejs)
add_dependencies(syn_cpp_gennodejs syn_cpp_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS syn_cpp_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(syn_cpp
  "/home/slam327/syn_ws/src/syn_cpp/msg/BboxData.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/syn_cpp
)

### Generating Services

### Generating Module File
_generate_module_py(syn_cpp
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/syn_cpp
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(syn_cpp_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(syn_cpp_generate_messages syn_cpp_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/slam327/syn_ws/src/syn_cpp/msg/BboxData.msg" NAME_WE)
add_dependencies(syn_cpp_generate_messages_py _syn_cpp_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(syn_cpp_genpy)
add_dependencies(syn_cpp_genpy syn_cpp_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS syn_cpp_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/syn_cpp)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/syn_cpp
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(syn_cpp_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/syn_cpp)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/syn_cpp
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(syn_cpp_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/syn_cpp)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/syn_cpp
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(syn_cpp_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/syn_cpp)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/syn_cpp
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(syn_cpp_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/syn_cpp)
  install(CODE "execute_process(COMMAND \"/home/slam327/anaconda3/envs/rosenv/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/syn_cpp\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/syn_cpp
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(syn_cpp_generate_messages_py std_msgs_generate_messages_py)
endif()
