cmake_minimum_required(VERSION 3.0.2)
project(rolo)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-O3 -Wall -g")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(DEFINED ENV{ROS_VERSION})
  set(ROS_VERSION $ENV{ROS_VERSION})
endif()

find_package(catkin REQUIRED COMPONENTS
  autoware_rviz_msgs
  geometry_msgs
  jsk_recognition_msgs
  nav_msgs
  cv_bridge
  pcl_ros
  pcl_conversions
  roscpp
  rospy
  std_msgs
  sensor_msgs
  tf
  visualization_msgs
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  message_generation
)

find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)
find_package(GTSAM REQUIRED QUIET)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS timer)
find_package(glog QUIET)
if(TARGET glog::glog)
  set(GLOG_LIBRARIES glog::glog)
else()
  find_library(GLOG_LIBRARIES glog REQUIRED)
endif()
find_package(OpenMP)
if (OPENMP_FOUND)
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

add_message_files(
  # DIRECTORY msg
  FILES
  CloudInfoStamp.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs   nav_msgs   std_msgs   sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS PCL

  CATKIN_DEPENDS
  autoware_rviz_msgs
  std_msgs
  nav_msgs
  geometry_msgs
  jsk_recognition_msgs
  sensor_msgs
  message_runtime
  visualization_msgs
)

catkin_install_python(PROGRAMS test/translation_plot.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS test/dual_odom_delta_state_filter_future_split.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(
	include
  include/scancontext
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
	${GTSAM_INCLUDE_DIR}
)

link_directories(
	include
	${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  ${GTSAM_LIBRARY_DIRS}
)

add_executable(${PROJECT_NAME}_scanPreprocess src/scanPreprocess.cpp)
add_dependencies(${PROJECT_NAME}_scanPreprocess ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_scanPreprocess ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${GLOG_LIBRARIES})

add_executable(${PROJECT_NAME}_lidarOdometry src/lidarOdometry.cpp)
add_dependencies(${PROJECT_NAME}_lidarOdometry ${PROJECT_NAME}_generate_messages_cpp)
target_compile_options(${PROJECT_NAME}_lidarOdometry PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(${PROJECT_NAME}_lidarOdometry ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenMP_CXX_FLAGS} ${EIGEN_LIBRARIES} gtsam ${GLOG_LIBRARIES})

add_executable(${PROJECT_NAME}_backMapping
  src/backMapping.cpp
  src/scancontext/Scancontext.cpp
  src/prior_pose/pose_solver.cpp
)
add_dependencies(${PROJECT_NAME}_backMapping ${PROJECT_NAME}_generate_messages_cpp)
target_compile_options(${PROJECT_NAME}_backMapping PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(${PROJECT_NAME}_backMapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenMP_CXX_FLAGS} ${EIGEN_LIBRARIES} gtsam ${GLOG_LIBRARIES})

add_executable(prior_pose_node
  src/prior_pose/pose_solver.cpp
  src/prior_pose/prior_pose_node.cpp
)
add_dependencies(prior_pose_node ${PROJECT_NAME}_generate_messages_cpp)
target_compile_options(prior_pose_node PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(prior_pose_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenMP_CXX_FLAGS} ${EIGEN_LIBRARIES} gtsam ${GLOG_LIBRARIES}
)

## Test executable
# add_executable(rotation_test test/rotation_test.cpp)
# target_link_libraries(rotation_test ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

# add_executable(eskf_odom_test test/eskf_odom_test.cpp)
# add_dependencies(eskf_odom_test ${PROJECT_NAME}_generate_messages_cpp)
# target_link_libraries(eskf_odom_test ${catkin_LIBRARIES})
