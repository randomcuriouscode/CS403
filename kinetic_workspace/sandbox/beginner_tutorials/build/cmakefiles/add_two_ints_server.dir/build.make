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
CMAKE_SOURCE_DIR = /home/rpg711/kinetic_workspace/sandbox/beginner_tutorials

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/rpg711/kinetic_workspace/sandbox/beginner_tutorials/build

# Include any dependencies generated for this target.
include CMakeFiles/add_two_ints_server.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/add_two_ints_server.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/add_two_ints_server.dir/flags.make

CMakeFiles/add_two_ints_server.dir/src/add_two_ints_server.cpp.o: CMakeFiles/add_two_ints_server.dir/flags.make
CMakeFiles/add_two_ints_server.dir/src/add_two_ints_server.cpp.o: ../src/add_two_ints_server.cpp
CMakeFiles/add_two_ints_server.dir/src/add_two_ints_server.cpp.o: ../manifest.xml
CMakeFiles/add_two_ints_server.dir/src/add_two_ints_server.cpp.o: /opt/ros/kinetic/share/cpp_common/package.xml
CMakeFiles/add_two_ints_server.dir/src/add_two_ints_server.cpp.o: /opt/ros/kinetic/share/rostime/package.xml
CMakeFiles/add_two_ints_server.dir/src/add_two_ints_server.cpp.o: /opt/ros/kinetic/share/roscpp_traits/package.xml
CMakeFiles/add_two_ints_server.dir/src/add_two_ints_server.cpp.o: /opt/ros/kinetic/share/roscpp_serialization/package.xml
CMakeFiles/add_two_ints_server.dir/src/add_two_ints_server.cpp.o: /opt/ros/kinetic/share/catkin/package.xml
CMakeFiles/add_two_ints_server.dir/src/add_two_ints_server.cpp.o: /opt/ros/kinetic/share/genmsg/package.xml
CMakeFiles/add_two_ints_server.dir/src/add_two_ints_server.cpp.o: /opt/ros/kinetic/share/genpy/package.xml
CMakeFiles/add_two_ints_server.dir/src/add_two_ints_server.cpp.o: /opt/ros/kinetic/share/message_runtime/package.xml
CMakeFiles/add_two_ints_server.dir/src/add_two_ints_server.cpp.o: /opt/ros/kinetic/share/std_msgs/package.xml
CMakeFiles/add_two_ints_server.dir/src/add_two_ints_server.cpp.o: /opt/ros/kinetic/share/gencpp/package.xml
CMakeFiles/add_two_ints_server.dir/src/add_two_ints_server.cpp.o: /opt/ros/kinetic/share/geneus/package.xml
CMakeFiles/add_two_ints_server.dir/src/add_two_ints_server.cpp.o: /opt/ros/kinetic/share/gennodejs/package.xml
CMakeFiles/add_two_ints_server.dir/src/add_two_ints_server.cpp.o: /opt/ros/kinetic/share/genlisp/package.xml
CMakeFiles/add_two_ints_server.dir/src/add_two_ints_server.cpp.o: /opt/ros/kinetic/share/message_generation/package.xml
CMakeFiles/add_two_ints_server.dir/src/add_two_ints_server.cpp.o: /opt/ros/kinetic/share/rosbuild/package.xml
CMakeFiles/add_two_ints_server.dir/src/add_two_ints_server.cpp.o: /opt/ros/kinetic/share/rosconsole/package.xml
CMakeFiles/add_two_ints_server.dir/src/add_two_ints_server.cpp.o: /opt/ros/kinetic/share/rosgraph_msgs/package.xml
CMakeFiles/add_two_ints_server.dir/src/add_two_ints_server.cpp.o: /opt/ros/kinetic/share/xmlrpcpp/package.xml
CMakeFiles/add_two_ints_server.dir/src/add_two_ints_server.cpp.o: /opt/ros/kinetic/share/roscpp/package.xml
CMakeFiles/add_two_ints_server.dir/src/add_two_ints_server.cpp.o: /opt/ros/kinetic/share/rosgraph/package.xml
CMakeFiles/add_two_ints_server.dir/src/add_two_ints_server.cpp.o: /opt/ros/kinetic/share/rospack/package.xml
CMakeFiles/add_two_ints_server.dir/src/add_two_ints_server.cpp.o: /opt/ros/kinetic/share/roslib/package.xml
CMakeFiles/add_two_ints_server.dir/src/add_two_ints_server.cpp.o: /opt/ros/kinetic/share/rospy/package.xml
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/rpg711/kinetic_workspace/sandbox/beginner_tutorials/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/add_two_ints_server.dir/src/add_two_ints_server.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/add_two_ints_server.dir/src/add_two_ints_server.cpp.o -c /home/rpg711/kinetic_workspace/sandbox/beginner_tutorials/src/add_two_ints_server.cpp

CMakeFiles/add_two_ints_server.dir/src/add_two_ints_server.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/add_two_ints_server.dir/src/add_two_ints_server.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rpg711/kinetic_workspace/sandbox/beginner_tutorials/src/add_two_ints_server.cpp > CMakeFiles/add_two_ints_server.dir/src/add_two_ints_server.cpp.i

CMakeFiles/add_two_ints_server.dir/src/add_two_ints_server.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/add_two_ints_server.dir/src/add_two_ints_server.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rpg711/kinetic_workspace/sandbox/beginner_tutorials/src/add_two_ints_server.cpp -o CMakeFiles/add_two_ints_server.dir/src/add_two_ints_server.cpp.s

CMakeFiles/add_two_ints_server.dir/src/add_two_ints_server.cpp.o.requires:

.PHONY : CMakeFiles/add_two_ints_server.dir/src/add_two_ints_server.cpp.o.requires

CMakeFiles/add_two_ints_server.dir/src/add_two_ints_server.cpp.o.provides: CMakeFiles/add_two_ints_server.dir/src/add_two_ints_server.cpp.o.requires
	$(MAKE) -f CMakeFiles/add_two_ints_server.dir/build.make CMakeFiles/add_two_ints_server.dir/src/add_two_ints_server.cpp.o.provides.build
.PHONY : CMakeFiles/add_two_ints_server.dir/src/add_two_ints_server.cpp.o.provides

CMakeFiles/add_two_ints_server.dir/src/add_two_ints_server.cpp.o.provides.build: CMakeFiles/add_two_ints_server.dir/src/add_two_ints_server.cpp.o


# Object files for target add_two_ints_server
add_two_ints_server_OBJECTS = \
"CMakeFiles/add_two_ints_server.dir/src/add_two_ints_server.cpp.o"

# External object files for target add_two_ints_server
add_two_ints_server_EXTERNAL_OBJECTS =

../bin/add_two_ints_server: CMakeFiles/add_two_ints_server.dir/src/add_two_ints_server.cpp.o
../bin/add_two_ints_server: CMakeFiles/add_two_ints_server.dir/build.make
../bin/add_two_ints_server: CMakeFiles/add_two_ints_server.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/rpg711/kinetic_workspace/sandbox/beginner_tutorials/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../bin/add_two_ints_server"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/add_two_ints_server.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/add_two_ints_server.dir/build: ../bin/add_two_ints_server

.PHONY : CMakeFiles/add_two_ints_server.dir/build

CMakeFiles/add_two_ints_server.dir/requires: CMakeFiles/add_two_ints_server.dir/src/add_two_ints_server.cpp.o.requires

.PHONY : CMakeFiles/add_two_ints_server.dir/requires

CMakeFiles/add_two_ints_server.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/add_two_ints_server.dir/cmake_clean.cmake
.PHONY : CMakeFiles/add_two_ints_server.dir/clean

CMakeFiles/add_two_ints_server.dir/depend:
	cd /home/rpg711/kinetic_workspace/sandbox/beginner_tutorials/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/rpg711/kinetic_workspace/sandbox/beginner_tutorials /home/rpg711/kinetic_workspace/sandbox/beginner_tutorials /home/rpg711/kinetic_workspace/sandbox/beginner_tutorials/build /home/rpg711/kinetic_workspace/sandbox/beginner_tutorials/build /home/rpg711/kinetic_workspace/sandbox/beginner_tutorials/build/CMakeFiles/add_two_ints_server.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/add_two_ints_server.dir/depend

