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
include map_server/CMakeFiles/rtest.dir/depend.make

# Include the progress variables for this target.
include map_server/CMakeFiles/rtest.dir/progress.make

# Include the compile flags for this target's objects.
include map_server/CMakeFiles/rtest.dir/flags.make

map_server/CMakeFiles/rtest.dir/test/rtest.cpp.o: map_server/CMakeFiles/rtest.dir/flags.make
map_server/CMakeFiles/rtest.dir/test/rtest.cpp.o: /home/odroid/catkin_ws/src/map_server/test/rtest.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/odroid/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object map_server/CMakeFiles/rtest.dir/test/rtest.cpp.o"
	cd /home/odroid/catkin_ws/build/map_server && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/rtest.dir/test/rtest.cpp.o -c /home/odroid/catkin_ws/src/map_server/test/rtest.cpp

map_server/CMakeFiles/rtest.dir/test/rtest.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rtest.dir/test/rtest.cpp.i"
	cd /home/odroid/catkin_ws/build/map_server && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/odroid/catkin_ws/src/map_server/test/rtest.cpp > CMakeFiles/rtest.dir/test/rtest.cpp.i

map_server/CMakeFiles/rtest.dir/test/rtest.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rtest.dir/test/rtest.cpp.s"
	cd /home/odroid/catkin_ws/build/map_server && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/odroid/catkin_ws/src/map_server/test/rtest.cpp -o CMakeFiles/rtest.dir/test/rtest.cpp.s

map_server/CMakeFiles/rtest.dir/test/rtest.cpp.o.requires:

.PHONY : map_server/CMakeFiles/rtest.dir/test/rtest.cpp.o.requires

map_server/CMakeFiles/rtest.dir/test/rtest.cpp.o.provides: map_server/CMakeFiles/rtest.dir/test/rtest.cpp.o.requires
	$(MAKE) -f map_server/CMakeFiles/rtest.dir/build.make map_server/CMakeFiles/rtest.dir/test/rtest.cpp.o.provides.build
.PHONY : map_server/CMakeFiles/rtest.dir/test/rtest.cpp.o.provides

map_server/CMakeFiles/rtest.dir/test/rtest.cpp.o.provides.build: map_server/CMakeFiles/rtest.dir/test/rtest.cpp.o


map_server/CMakeFiles/rtest.dir/test/test_constants.cpp.o: map_server/CMakeFiles/rtest.dir/flags.make
map_server/CMakeFiles/rtest.dir/test/test_constants.cpp.o: /home/odroid/catkin_ws/src/map_server/test/test_constants.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/odroid/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object map_server/CMakeFiles/rtest.dir/test/test_constants.cpp.o"
	cd /home/odroid/catkin_ws/build/map_server && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/rtest.dir/test/test_constants.cpp.o -c /home/odroid/catkin_ws/src/map_server/test/test_constants.cpp

map_server/CMakeFiles/rtest.dir/test/test_constants.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rtest.dir/test/test_constants.cpp.i"
	cd /home/odroid/catkin_ws/build/map_server && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/odroid/catkin_ws/src/map_server/test/test_constants.cpp > CMakeFiles/rtest.dir/test/test_constants.cpp.i

map_server/CMakeFiles/rtest.dir/test/test_constants.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rtest.dir/test/test_constants.cpp.s"
	cd /home/odroid/catkin_ws/build/map_server && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/odroid/catkin_ws/src/map_server/test/test_constants.cpp -o CMakeFiles/rtest.dir/test/test_constants.cpp.s

map_server/CMakeFiles/rtest.dir/test/test_constants.cpp.o.requires:

.PHONY : map_server/CMakeFiles/rtest.dir/test/test_constants.cpp.o.requires

map_server/CMakeFiles/rtest.dir/test/test_constants.cpp.o.provides: map_server/CMakeFiles/rtest.dir/test/test_constants.cpp.o.requires
	$(MAKE) -f map_server/CMakeFiles/rtest.dir/build.make map_server/CMakeFiles/rtest.dir/test/test_constants.cpp.o.provides.build
.PHONY : map_server/CMakeFiles/rtest.dir/test/test_constants.cpp.o.provides

map_server/CMakeFiles/rtest.dir/test/test_constants.cpp.o.provides.build: map_server/CMakeFiles/rtest.dir/test/test_constants.cpp.o


# Object files for target rtest
rtest_OBJECTS = \
"CMakeFiles/rtest.dir/test/rtest.cpp.o" \
"CMakeFiles/rtest.dir/test/test_constants.cpp.o"

# External object files for target rtest
rtest_EXTERNAL_OBJECTS =

/home/odroid/catkin_ws/devel/lib/map_server/rtest: map_server/CMakeFiles/rtest.dir/test/rtest.cpp.o
/home/odroid/catkin_ws/devel/lib/map_server/rtest: map_server/CMakeFiles/rtest.dir/test/test_constants.cpp.o
/home/odroid/catkin_ws/devel/lib/map_server/rtest: map_server/CMakeFiles/rtest.dir/build.make
/home/odroid/catkin_ws/devel/lib/map_server/rtest: gtest/libgtest.so
/home/odroid/catkin_ws/devel/lib/map_server/rtest: /opt/ros/kinetic/lib/libtf.so
/home/odroid/catkin_ws/devel/lib/map_server/rtest: /opt/ros/kinetic/lib/libtf2_ros.so
/home/odroid/catkin_ws/devel/lib/map_server/rtest: /opt/ros/kinetic/lib/libactionlib.so
/home/odroid/catkin_ws/devel/lib/map_server/rtest: /opt/ros/kinetic/lib/libmessage_filters.so
/home/odroid/catkin_ws/devel/lib/map_server/rtest: /opt/ros/kinetic/lib/libroscpp.so
/home/odroid/catkin_ws/devel/lib/map_server/rtest: /usr/lib/arm-linux-gnueabihf/libboost_signals.so
/home/odroid/catkin_ws/devel/lib/map_server/rtest: /usr/lib/arm-linux-gnueabihf/libboost_filesystem.so
/home/odroid/catkin_ws/devel/lib/map_server/rtest: /opt/ros/kinetic/lib/libxmlrpcpp.so
/home/odroid/catkin_ws/devel/lib/map_server/rtest: /opt/ros/kinetic/lib/libtf2.so
/home/odroid/catkin_ws/devel/lib/map_server/rtest: /opt/ros/kinetic/lib/librosconsole.so
/home/odroid/catkin_ws/devel/lib/map_server/rtest: /opt/ros/kinetic/lib/librosconsole_log4cxx.so
/home/odroid/catkin_ws/devel/lib/map_server/rtest: /opt/ros/kinetic/lib/librosconsole_backend_interface.so
/home/odroid/catkin_ws/devel/lib/map_server/rtest: /usr/lib/arm-linux-gnueabihf/liblog4cxx.so
/home/odroid/catkin_ws/devel/lib/map_server/rtest: /usr/lib/arm-linux-gnueabihf/libboost_regex.so
/home/odroid/catkin_ws/devel/lib/map_server/rtest: /opt/ros/kinetic/lib/libroscpp_serialization.so
/home/odroid/catkin_ws/devel/lib/map_server/rtest: /opt/ros/kinetic/lib/librostime.so
/home/odroid/catkin_ws/devel/lib/map_server/rtest: /opt/ros/kinetic/lib/libcpp_common.so
/home/odroid/catkin_ws/devel/lib/map_server/rtest: /usr/lib/arm-linux-gnueabihf/libboost_system.so
/home/odroid/catkin_ws/devel/lib/map_server/rtest: /usr/lib/arm-linux-gnueabihf/libboost_thread.so
/home/odroid/catkin_ws/devel/lib/map_server/rtest: /usr/lib/arm-linux-gnueabihf/libboost_chrono.so
/home/odroid/catkin_ws/devel/lib/map_server/rtest: /usr/lib/arm-linux-gnueabihf/libboost_date_time.so
/home/odroid/catkin_ws/devel/lib/map_server/rtest: /usr/lib/arm-linux-gnueabihf/libboost_atomic.so
/home/odroid/catkin_ws/devel/lib/map_server/rtest: /usr/lib/arm-linux-gnueabihf/libpthread.so
/home/odroid/catkin_ws/devel/lib/map_server/rtest: /usr/lib/arm-linux-gnueabihf/libconsole_bridge.so
/home/odroid/catkin_ws/devel/lib/map_server/rtest: map_server/CMakeFiles/rtest.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/odroid/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable /home/odroid/catkin_ws/devel/lib/map_server/rtest"
	cd /home/odroid/catkin_ws/build/map_server && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/rtest.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
map_server/CMakeFiles/rtest.dir/build: /home/odroid/catkin_ws/devel/lib/map_server/rtest

.PHONY : map_server/CMakeFiles/rtest.dir/build

map_server/CMakeFiles/rtest.dir/requires: map_server/CMakeFiles/rtest.dir/test/rtest.cpp.o.requires
map_server/CMakeFiles/rtest.dir/requires: map_server/CMakeFiles/rtest.dir/test/test_constants.cpp.o.requires

.PHONY : map_server/CMakeFiles/rtest.dir/requires

map_server/CMakeFiles/rtest.dir/clean:
	cd /home/odroid/catkin_ws/build/map_server && $(CMAKE_COMMAND) -P CMakeFiles/rtest.dir/cmake_clean.cmake
.PHONY : map_server/CMakeFiles/rtest.dir/clean

map_server/CMakeFiles/rtest.dir/depend:
	cd /home/odroid/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/odroid/catkin_ws/src /home/odroid/catkin_ws/src/map_server /home/odroid/catkin_ws/build /home/odroid/catkin_ws/build/map_server /home/odroid/catkin_ws/build/map_server/CMakeFiles/rtest.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : map_server/CMakeFiles/rtest.dir/depend

