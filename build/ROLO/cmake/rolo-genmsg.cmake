# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "rolo: 1 messages, 0 services")

set(MSG_I_FLAGS "-Irolo:/home/yuaqi/3D_slam/rolo/src/ROLO/msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg;-Inav_msgs:/opt/ros/melodic/share/nav_msgs/cmake/../msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/melodic/share/sensor_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/melodic/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(rolo_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/yuaqi/3D_slam/rolo/src/ROLO/msg/CloudInfoStamp.msg" NAME_WE)
add_custom_target(_rolo_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rolo" "/home/yuaqi/3D_slam/rolo/src/ROLO/msg/CloudInfoStamp.msg" "sensor_msgs/PointCloud2:sensor_msgs/PointField:std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(rolo
  "/home/yuaqi/3D_slam/rolo/src/ROLO/msg/CloudInfoStamp.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rolo
)

### Generating Services

### Generating Module File
_generate_module_cpp(rolo
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rolo
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(rolo_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(rolo_generate_messages rolo_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yuaqi/3D_slam/rolo/src/ROLO/msg/CloudInfoStamp.msg" NAME_WE)
add_dependencies(rolo_generate_messages_cpp _rolo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rolo_gencpp)
add_dependencies(rolo_gencpp rolo_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rolo_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(rolo
  "/home/yuaqi/3D_slam/rolo/src/ROLO/msg/CloudInfoStamp.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rolo
)

### Generating Services

### Generating Module File
_generate_module_eus(rolo
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rolo
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(rolo_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(rolo_generate_messages rolo_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yuaqi/3D_slam/rolo/src/ROLO/msg/CloudInfoStamp.msg" NAME_WE)
add_dependencies(rolo_generate_messages_eus _rolo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rolo_geneus)
add_dependencies(rolo_geneus rolo_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rolo_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(rolo
  "/home/yuaqi/3D_slam/rolo/src/ROLO/msg/CloudInfoStamp.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rolo
)

### Generating Services

### Generating Module File
_generate_module_lisp(rolo
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rolo
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(rolo_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(rolo_generate_messages rolo_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yuaqi/3D_slam/rolo/src/ROLO/msg/CloudInfoStamp.msg" NAME_WE)
add_dependencies(rolo_generate_messages_lisp _rolo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rolo_genlisp)
add_dependencies(rolo_genlisp rolo_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rolo_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(rolo
  "/home/yuaqi/3D_slam/rolo/src/ROLO/msg/CloudInfoStamp.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rolo
)

### Generating Services

### Generating Module File
_generate_module_nodejs(rolo
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rolo
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(rolo_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(rolo_generate_messages rolo_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yuaqi/3D_slam/rolo/src/ROLO/msg/CloudInfoStamp.msg" NAME_WE)
add_dependencies(rolo_generate_messages_nodejs _rolo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rolo_gennodejs)
add_dependencies(rolo_gennodejs rolo_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rolo_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(rolo
  "/home/yuaqi/3D_slam/rolo/src/ROLO/msg/CloudInfoStamp.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rolo
)

### Generating Services

### Generating Module File
_generate_module_py(rolo
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rolo
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(rolo_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(rolo_generate_messages rolo_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yuaqi/3D_slam/rolo/src/ROLO/msg/CloudInfoStamp.msg" NAME_WE)
add_dependencies(rolo_generate_messages_py _rolo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rolo_genpy)
add_dependencies(rolo_genpy rolo_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rolo_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rolo)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rolo
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(rolo_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET nav_msgs_generate_messages_cpp)
  add_dependencies(rolo_generate_messages_cpp nav_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(rolo_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(rolo_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rolo)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rolo
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(rolo_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET nav_msgs_generate_messages_eus)
  add_dependencies(rolo_generate_messages_eus nav_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(rolo_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(rolo_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rolo)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rolo
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(rolo_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET nav_msgs_generate_messages_lisp)
  add_dependencies(rolo_generate_messages_lisp nav_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(rolo_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(rolo_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rolo)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rolo
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(rolo_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET nav_msgs_generate_messages_nodejs)
  add_dependencies(rolo_generate_messages_nodejs nav_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(rolo_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(rolo_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rolo)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rolo\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rolo
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(rolo_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET nav_msgs_generate_messages_py)
  add_dependencies(rolo_generate_messages_py nav_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(rolo_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(rolo_generate_messages_py sensor_msgs_generate_messages_py)
endif()
