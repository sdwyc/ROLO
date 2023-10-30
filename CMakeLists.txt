cmake_minimum_required(VERSION 3.0.2)
project(rolo)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  cv_bridge
  pcl_ros
  roscpp
  rospy
  std_msgs
  sensor_msgs
  tf
  visualization_msgs
  message_generation
)

find_package(teaserpp REQUIRED)
find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)
find_package(GTSAM REQUIRED QUIET)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS timer)

# pmc (Parallel Maximum Clique)

unset(PMC CACHE)
find_library(PMC pmc)
if(${PMC} STREQUAL "PMC-NOTFOUND")
    message(WARNING "PMC library was not found, Git Now")
    execute_process(COMMAND mkdir ${CMAKE_BINARY_DIR}/pmc-src)
    execute_process(COMMAND mkdir ${CMAKE_BINARY_DIR}/pmc-build)
    
    configure_file(cmake/pmc.CMakeLists.txt ${CMAKE_BINARY_DIR}/pmc-download/CMakeLists.txt)
    execute_process(COMMAND "${CMAKE_COMMAND}" -G "${CMAKE_GENERATOR}" .
            WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/pmc-download")
    execute_process(COMMAND "${CMAKE_COMMAND}" --build .
            WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/pmc-download")
    add_subdirectory("${CMAKE_BINARY_DIR}/pmc-src"
            "${CMAKE_BINARY_DIR}/pmc-build")
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
  DEPENDS PCL GTSAM

  CATKIN_DEPENDS
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  message_runtime
  visualization_msgs
)

catkin_install_python(PROGRAMS test/translation_plot.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(
	include
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

add_executable(${PROJECT_NAME}_imageProjection src/imageProjection.cpp)
add_dependencies(${PROJECT_NAME}_imageProjection ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_imageProjection ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(${PROJECT_NAME}_featureExtraction src/featureExtraction.cpp)
add_dependencies(${PROJECT_NAME}_featureExtraction ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_featureExtraction ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(${PROJECT_NAME}_lidarOdometry src/lidarOdometry.cpp src/undirected_graph.cpp)
add_dependencies(${PROJECT_NAME}_lidarOdometry ${PROJECT_NAME}_generate_messages_cpp)
target_compile_options(${PROJECT_NAME}_lidarOdometry PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(${PROJECT_NAME}_lidarOdometry ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenMP_CXX_FLAGS} ${EIGEN_LIBRARIES} gtsam pmc teaserpp::teaser_registration teaserpp::teaser_io teaserpp::teaser_features)

add_executable(${PROJECT_NAME}_backMapping src/backMapping.cpp)
add_dependencies(${PROJECT_NAME}_backMapping ${PROJECT_NAME}_generate_messages_cpp)
target_compile_options(${PROJECT_NAME}_backMapping PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(${PROJECT_NAME}_backMapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenMP_CXX_FLAGS} ${EIGEN_LIBRARIES} gtsam)


## Test executable
add_executable(rotation_test test/rotation_test.cpp)
target_link_libraries(rotation_test ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
