# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/yuaqi/3D_slam/rolo/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/yuaqi/3D_slam/rolo/build

# Utility rule file for rolo_generate_messages_cpp.

# Include the progress variables for this target.
include ROLO/CMakeFiles/rolo_generate_messages_cpp.dir/progress.make

ROLO/CMakeFiles/rolo_generate_messages_cpp: /home/yuaqi/3D_slam/rolo/devel/include/rolo/CloudInfoStamp.h


/home/yuaqi/3D_slam/rolo/devel/include/rolo/CloudInfoStamp.h: /opt/ros/melodic/lib/gencpp/gen_cpp.py
/home/yuaqi/3D_slam/rolo/devel/include/rolo/CloudInfoStamp.h: /home/yuaqi/3D_slam/rolo/src/ROLO/msg/CloudInfoStamp.msg
/home/yuaqi/3D_slam/rolo/devel/include/rolo/CloudInfoStamp.h: /opt/ros/melodic/share/sensor_msgs/msg/PointCloud2.msg
/home/yuaqi/3D_slam/rolo/devel/include/rolo/CloudInfoStamp.h: /opt/ros/melodic/share/sensor_msgs/msg/PointField.msg
/home/yuaqi/3D_slam/rolo/devel/include/rolo/CloudInfoStamp.h: /opt/ros/melodic/share/std_msgs/msg/Header.msg
/home/yuaqi/3D_slam/rolo/devel/include/rolo/CloudInfoStamp.h: /opt/ros/melodic/share/gencpp/msg.h.template
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/yuaqi/3D_slam/rolo/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating C++ code from rolo/CloudInfoStamp.msg"
	cd /home/yuaqi/3D_slam/rolo/src/ROLO && /home/yuaqi/3D_slam/rolo/build/catkin_generated/env_cached.sh /usr/bin/python2 /opt/ros/melodic/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/yuaqi/3D_slam/rolo/src/ROLO/msg/CloudInfoStamp.msg -Irolo:/home/yuaqi/3D_slam/rolo/src/ROLO/msg -Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg -Inav_msgs:/opt/ros/melodic/share/nav_msgs/cmake/../msg -Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg -Isensor_msgs:/opt/ros/melodic/share/sensor_msgs/cmake/../msg -Iactionlib_msgs:/opt/ros/melodic/share/actionlib_msgs/cmake/../msg -p rolo -o /home/yuaqi/3D_slam/rolo/devel/include/rolo -e /opt/ros/melodic/share/gencpp/cmake/..

rolo_generate_messages_cpp: ROLO/CMakeFiles/rolo_generate_messages_cpp
rolo_generate_messages_cpp: /home/yuaqi/3D_slam/rolo/devel/include/rolo/CloudInfoStamp.h
rolo_generate_messages_cpp: ROLO/CMakeFiles/rolo_generate_messages_cpp.dir/build.make

.PHONY : rolo_generate_messages_cpp

# Rule to build all files generated by this target.
ROLO/CMakeFiles/rolo_generate_messages_cpp.dir/build: rolo_generate_messages_cpp

.PHONY : ROLO/CMakeFiles/rolo_generate_messages_cpp.dir/build

ROLO/CMakeFiles/rolo_generate_messages_cpp.dir/clean:
	cd /home/yuaqi/3D_slam/rolo/build/ROLO && $(CMAKE_COMMAND) -P CMakeFiles/rolo_generate_messages_cpp.dir/cmake_clean.cmake
.PHONY : ROLO/CMakeFiles/rolo_generate_messages_cpp.dir/clean

ROLO/CMakeFiles/rolo_generate_messages_cpp.dir/depend:
	cd /home/yuaqi/3D_slam/rolo/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/yuaqi/3D_slam/rolo/src /home/yuaqi/3D_slam/rolo/src/ROLO /home/yuaqi/3D_slam/rolo/build /home/yuaqi/3D_slam/rolo/build/ROLO /home/yuaqi/3D_slam/rolo/build/ROLO/CMakeFiles/rolo_generate_messages_cpp.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : ROLO/CMakeFiles/rolo_generate_messages_cpp.dir/depend

