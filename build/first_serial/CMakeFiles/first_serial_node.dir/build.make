# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/odroid/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/odroid/catkin_ws/build

# Include any dependencies generated for this target.
include first_serial/CMakeFiles/first_serial_node.dir/depend.make

# Include the progress variables for this target.
include first_serial/CMakeFiles/first_serial_node.dir/progress.make

# Include the compile flags for this target's objects.
include first_serial/CMakeFiles/first_serial_node.dir/flags.make

first_serial/CMakeFiles/first_serial_node.dir/first_serial.cpp.o: first_serial/CMakeFiles/first_serial_node.dir/flags.make
first_serial/CMakeFiles/first_serial_node.dir/first_serial.cpp.o: /home/odroid/catkin_ws/src/first_serial/first_serial.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/odroid/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object first_serial/CMakeFiles/first_serial_node.dir/first_serial.cpp.o"
	cd /home/odroid/catkin_ws/build/first_serial && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/first_serial_node.dir/first_serial.cpp.o -c /home/odroid/catkin_ws/src/first_serial/first_serial.cpp

first_serial/CMakeFiles/first_serial_node.dir/first_serial.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/first_serial_node.dir/first_serial.cpp.i"
	cd /home/odroid/catkin_ws/build/first_serial && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/odroid/catkin_ws/src/first_serial/first_serial.cpp > CMakeFiles/first_serial_node.dir/first_serial.cpp.i

first_serial/CMakeFiles/first_serial_node.dir/first_serial.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/first_serial_node.dir/first_serial.cpp.s"
	cd /home/odroid/catkin_ws/build/first_serial && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/odroid/catkin_ws/src/first_serial/first_serial.cpp -o CMakeFiles/first_serial_node.dir/first_serial.cpp.s

first_serial/CMakeFiles/first_serial_node.dir/first_serial.cpp.o.requires:

.PHONY : first_serial/CMakeFiles/first_serial_node.dir/first_serial.cpp.o.requires

first_serial/CMakeFiles/first_serial_node.dir/first_serial.cpp.o.provides: first_serial/CMakeFiles/first_serial_node.dir/first_serial.cpp.o.requires
	$(MAKE) -f first_serial/CMakeFiles/first_serial_node.dir/build.make first_serial/CMakeFiles/first_serial_node.dir/first_serial.cpp.o.provides.build
.PHONY : first_serial/CMakeFiles/first_serial_node.dir/first_serial.cpp.o.provides

first_serial/CMakeFiles/first_serial_node.dir/first_serial.cpp.o.provides.build: first_serial/CMakeFiles/first_serial_node.dir/first_serial.cpp.o


# Object files for target first_serial_node
first_serial_node_OBJECTS = \
"CMakeFiles/first_serial_node.dir/first_serial.cpp.o"

# External object files for target first_serial_node
first_serial_node_EXTERNAL_OBJECTS =

/home/odroid/catkin_ws/devel/lib/first_serial/first_serial_node: first_serial/CMakeFiles/first_serial_node.dir/first_serial.cpp.o
/home/odroid/catkin_ws/devel/lib/first_serial/first_serial_node: first_serial/CMakeFiles/first_serial_node.dir/build.make
/home/odroid/catkin_ws/devel/lib/first_serial/first_serial_node: /opt/ros/kinetic/lib/libtf.so
/home/odroid/catkin_ws/devel/lib/first_serial/first_serial_node: /opt/ros/kinetic/lib/libtf2_ros.so
/home/odroid/catkin_ws/devel/lib/first_serial/first_serial_node: /opt/ros/kinetic/lib/libactionlib.so
/home/odroid/catkin_ws/devel/lib/first_serial/first_serial_node: /opt/ros/kinetic/lib/libmessage_filters.so
/home/odroid/catkin_ws/devel/lib/first_serial/first_serial_node: /opt/ros/kinetic/lib/libroscpp.so
/home/odroid/catkin_ws/devel/lib/first_serial/first_serial_node: /usr/lib/arm-linux-gnueabihf/libboost_signals.so
/home/odroid/catkin_ws/devel/lib/first_serial/first_serial_node: /usr/lib/arm-linux-gnueabihf/libboost_filesystem.so
/home/odroid/catkin_ws/devel/lib/first_serial/first_serial_node: /opt/ros/kinetic/lib/libxmlrpcpp.so
/home/odroid/catkin_ws/devel/lib/first_serial/first_serial_node: /opt/ros/kinetic/lib/libtf2.so
/home/odroid/catkin_ws/devel/lib/first_serial/first_serial_node: /opt/ros/kinetic/lib/libroscpp_serialization.so
/home/odroid/catkin_ws/devel/lib/first_serial/first_serial_node: /opt/ros/kinetic/lib/librosconsole.so
/home/odroid/catkin_ws/devel/lib/first_serial/first_serial_node: /opt/ros/kinetic/lib/librosconsole_log4cxx.so
/home/odroid/catkin_ws/devel/lib/first_serial/first_serial_node: /opt/ros/kinetic/lib/librosconsole_backend_interface.so
/home/odroid/catkin_ws/devel/lib/first_serial/first_serial_node: /usr/lib/arm-linux-gnueabihf/liblog4cxx.so
/home/odroid/catkin_ws/devel/lib/first_serial/first_serial_node: /usr/lib/arm-linux-gnueabihf/libboost_regex.so
/home/odroid/catkin_ws/devel/lib/first_serial/first_serial_node: /opt/ros/kinetic/lib/librostime.so
/home/odroid/catkin_ws/devel/lib/first_serial/first_serial_node: /opt/ros/kinetic/lib/libcpp_common.so
/home/odroid/catkin_ws/devel/lib/first_serial/first_serial_node: /usr/lib/arm-linux-gnueabihf/libboost_system.so
/home/odroid/catkin_ws/devel/lib/first_serial/first_serial_node: /usr/lib/arm-linux-gnueabihf/libboost_thread.so
/home/odroid/catkin_ws/devel/lib/first_serial/first_serial_node: /usr/lib/arm-linux-gnueabihf/libboost_chrono.so
/home/odroid/catkin_ws/devel/lib/first_serial/first_serial_node: /usr/lib/arm-linux-gnueabihf/libboost_date_time.so
/home/odroid/catkin_ws/devel/lib/first_serial/first_serial_node: /usr/lib/arm-linux-gnueabihf/libboost_atomic.so
/home/odroid/catkin_ws/devel/lib/first_serial/first_serial_node: /usr/lib/arm-linux-gnueabihf/libpthread.so
/home/odroid/catkin_ws/devel/lib/first_serial/first_serial_node: /usr/lib/arm-linux-gnueabihf/libconsole_bridge.so
/home/odroid/catkin_ws/devel/lib/first_serial/first_serial_node: first_serial/CMakeFiles/first_serial_node.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/odroid/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/odroid/catkin_ws/devel/lib/first_serial/first_serial_node"
	cd /home/odroid/catkin_ws/build/first_serial && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/first_serial_node.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
first_serial/CMakeFiles/first_serial_node.dir/build: /home/odroid/catkin_ws/devel/lib/first_serial/first_serial_node

.PHONY : first_serial/CMakeFiles/first_serial_node.dir/build

first_serial/CMakeFiles/first_serial_node.dir/requires: first_serial/CMakeFiles/first_serial_node.dir/first_serial.cpp.o.requires

.PHONY : first_serial/CMakeFiles/first_serial_node.dir/requires

first_serial/CMakeFiles/first_serial_node.dir/clean:
	cd /home/odroid/catkin_ws/build/first_serial && $(CMAKE_COMMAND) -P CMakeFiles/first_serial_node.dir/cmake_clean.cmake
.PHONY : first_serial/CMakeFiles/first_serial_node.dir/clean

first_serial/CMakeFiles/first_serial_node.dir/depend:
	cd /home/odroid/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/odroid/catkin_ws/src /home/odroid/catkin_ws/src/first_serial /home/odroid/catkin_ws/build /home/odroid/catkin_ws/build/first_serial /home/odroid/catkin_ws/build/first_serial/CMakeFiles/first_serial_node.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : first_serial/CMakeFiles/first_serial_node.dir/depend

