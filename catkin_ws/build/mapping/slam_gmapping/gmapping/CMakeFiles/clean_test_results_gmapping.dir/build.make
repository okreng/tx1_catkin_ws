# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/cmake-gui

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/ubuntu/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ubuntu/catkin_ws/build

# Utility rule file for clean_test_results_gmapping.

# Include the progress variables for this target.
include mapping/slam_gmapping/gmapping/CMakeFiles/clean_test_results_gmapping.dir/progress.make

mapping/slam_gmapping/gmapping/CMakeFiles/clean_test_results_gmapping:
	cd /home/ubuntu/catkin_ws/build/mapping/slam_gmapping/gmapping && /usr/bin/python /opt/ros/indigo/share/catkin/cmake/test/remove_test_results.py /home/ubuntu/catkin_ws/build/test_results/gmapping

clean_test_results_gmapping: mapping/slam_gmapping/gmapping/CMakeFiles/clean_test_results_gmapping
clean_test_results_gmapping: mapping/slam_gmapping/gmapping/CMakeFiles/clean_test_results_gmapping.dir/build.make
.PHONY : clean_test_results_gmapping

# Rule to build all files generated by this target.
mapping/slam_gmapping/gmapping/CMakeFiles/clean_test_results_gmapping.dir/build: clean_test_results_gmapping
.PHONY : mapping/slam_gmapping/gmapping/CMakeFiles/clean_test_results_gmapping.dir/build

mapping/slam_gmapping/gmapping/CMakeFiles/clean_test_results_gmapping.dir/clean:
	cd /home/ubuntu/catkin_ws/build/mapping/slam_gmapping/gmapping && $(CMAKE_COMMAND) -P CMakeFiles/clean_test_results_gmapping.dir/cmake_clean.cmake
.PHONY : mapping/slam_gmapping/gmapping/CMakeFiles/clean_test_results_gmapping.dir/clean

mapping/slam_gmapping/gmapping/CMakeFiles/clean_test_results_gmapping.dir/depend:
	cd /home/ubuntu/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ubuntu/catkin_ws/src /home/ubuntu/catkin_ws/src/mapping/slam_gmapping/gmapping /home/ubuntu/catkin_ws/build /home/ubuntu/catkin_ws/build/mapping/slam_gmapping/gmapping /home/ubuntu/catkin_ws/build/mapping/slam_gmapping/gmapping/CMakeFiles/clean_test_results_gmapping.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : mapping/slam_gmapping/gmapping/CMakeFiles/clean_test_results_gmapping.dir/depend

