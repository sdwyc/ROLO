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
find_package(OpenMP)
if (OPENMP_FOUND)
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

set(SMALL_GICP_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/include/small_gicp")
set(SMALL_GICP_INCLUDE_DIR "${SMALL_GICP_ROOT}/include")
set(SMALL_GICP_PCL_HEADER "${SMALL_GICP_INCLUDE_DIR}/small_gicp/pcl/pcl_registration.hpp")

if(EXISTS "${SMALL_GICP_PCL_HEADER}")
  # if(DEFINED PCL_VERSION AND PCL_VERSION VERSION_LESS "1.11.0")
  #   message(FATAL_ERROR
  #     "small_gicp PCL interface requires PCL >= 1.11, but found ${PCL_VERSION}. "
  #   )
  # endif()

  add_library(small_gicp_vendor INTERFACE)
  target_include_directories(small_gicp_vendor INTERFACE
    ${SMALL_GICP_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
  )
  target_compile_definitions(small_gicp_vendor INTERFACE BUILD_WITH_PCL)
  target_link_libraries(small_gicp_vendor INTERFACE
    Eigen3::Eigen
    ${PCL_LIBRARIES}
  )
  if(OPENMP_FOUND AND TARGET OpenMP::OpenMP_CXX)
    target_link_libraries(small_gicp_vendor INTERFACE OpenMP::OpenMP_CXX)
  endif()
# else()
#   message(WARNING
#     "small_gicp vendored repo was not found at ${SMALL_GICP_ROOT}. "
#     "Clone https://github.com/koide3/small_gicp into src/ROLO/include before using the small_gicp PCL interface."
#   )
endif()

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
# Add icp lib
add_library(rot_gicp SHARED
  src/rot_gicp/gicp/lsq_registration.cpp
  src/rot_gicp/gicp/rot_vgicp.cpp
)
target_link_libraries(rot_gicp
  ${PCL_LIBRARIES}
)
if (OPENMP_FOUND)
    if (TARGET OpenMP::OpenMP_CXX)
        target_link_libraries(rot_gicp OpenMP::OpenMP_CXX)
    endif ()
endif ()
target_include_directories(rot_gicp PUBLIC
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(${PROJECT_NAME}_imageProjection src/imageProjection.cpp)
add_dependencies(${PROJECT_NAME}_imageProjection ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_imageProjection ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(${PROJECT_NAME}_featureExtraction src/featureExtraction.cpp)
add_dependencies(${PROJECT_NAME}_featureExtraction ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_featureExtraction ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(${PROJECT_NAME}_lidarOdometry src/lidarOdometry.cpp)
add_dependencies(${PROJECT_NAME}_lidarOdometry ${PROJECT_NAME}_generate_messages_cpp rot_gicp)
target_compile_options(${PROJECT_NAME}_lidarOdometry PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(${PROJECT_NAME}_lidarOdometry ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenMP_CXX_FLAGS} ${EIGEN_LIBRARIES} gtsam rot_gicp)

add_executable(${PROJECT_NAME}_backMapping
  src/backMapping.cpp
  src/scancontext/Scancontext.cpp
  src/prior_pose/pose_solver.cpp
)
add_dependencies(${PROJECT_NAME}_backMapping ${PROJECT_NAME}_generate_messages_cpp)
target_compile_options(${PROJECT_NAME}_backMapping PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(${PROJECT_NAME}_backMapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenMP_CXX_FLAGS} ${EIGEN_LIBRARIES} gtsam)
if(TARGET small_gicp_vendor)
  target_link_libraries(${PROJECT_NAME}_backMapping small_gicp_vendor)
endif()

add_executable(prior_pose_node
  src/prior_pose/pose_solver.cpp
  src/prior_pose/prior_pose_node.cpp
)
add_dependencies(prior_pose_node ${PROJECT_NAME}_generate_messages_cpp)
target_compile_options(prior_pose_node PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(prior_pose_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenMP_CXX_FLAGS} ${EIGEN_LIBRARIES} gtsam
)

## Test executable
add_executable(rotation_test test/rotation_test.cpp)
target_link_libraries(rotation_test ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
