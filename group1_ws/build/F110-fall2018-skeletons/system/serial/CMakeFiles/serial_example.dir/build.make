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
CMAKE_SOURCE_DIR = /home/nvidia/group1_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/nvidia/group1_ws/build

# Include any dependencies generated for this target.
include F110-fall2018-skeletons/system/serial/CMakeFiles/serial_example.dir/depend.make

# Include the progress variables for this target.
include F110-fall2018-skeletons/system/serial/CMakeFiles/serial_example.dir/progress.make

# Include the compile flags for this target's objects.
include F110-fall2018-skeletons/system/serial/CMakeFiles/serial_example.dir/flags.make

F110-fall2018-skeletons/system/serial/CMakeFiles/serial_example.dir/examples/serial_example.cc.o: F110-fall2018-skeletons/system/serial/CMakeFiles/serial_example.dir/flags.make
F110-fall2018-skeletons/system/serial/CMakeFiles/serial_example.dir/examples/serial_example.cc.o: /home/nvidia/group1_ws/src/F110-fall2018-skeletons/system/serial/examples/serial_example.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nvidia/group1_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object F110-fall2018-skeletons/system/serial/CMakeFiles/serial_example.dir/examples/serial_example.cc.o"
	cd /home/nvidia/group1_ws/build/F110-fall2018-skeletons/system/serial && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/serial_example.dir/examples/serial_example.cc.o -c /home/nvidia/group1_ws/src/F110-fall2018-skeletons/system/serial/examples/serial_example.cc

F110-fall2018-skeletons/system/serial/CMakeFiles/serial_example.dir/examples/serial_example.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/serial_example.dir/examples/serial_example.cc.i"
	cd /home/nvidia/group1_ws/build/F110-fall2018-skeletons/system/serial && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nvidia/group1_ws/src/F110-fall2018-skeletons/system/serial/examples/serial_example.cc > CMakeFiles/serial_example.dir/examples/serial_example.cc.i

F110-fall2018-skeletons/system/serial/CMakeFiles/serial_example.dir/examples/serial_example.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/serial_example.dir/examples/serial_example.cc.s"
	cd /home/nvidia/group1_ws/build/F110-fall2018-skeletons/system/serial && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nvidia/group1_ws/src/F110-fall2018-skeletons/system/serial/examples/serial_example.cc -o CMakeFiles/serial_example.dir/examples/serial_example.cc.s

F110-fall2018-skeletons/system/serial/CMakeFiles/serial_example.dir/examples/serial_example.cc.o.requires:

.PHONY : F110-fall2018-skeletons/system/serial/CMakeFiles/serial_example.dir/examples/serial_example.cc.o.requires

F110-fall2018-skeletons/system/serial/CMakeFiles/serial_example.dir/examples/serial_example.cc.o.provides: F110-fall2018-skeletons/system/serial/CMakeFiles/serial_example.dir/examples/serial_example.cc.o.requires
	$(MAKE) -f F110-fall2018-skeletons/system/serial/CMakeFiles/serial_example.dir/build.make F110-fall2018-skeletons/system/serial/CMakeFiles/serial_example.dir/examples/serial_example.cc.o.provides.build
.PHONY : F110-fall2018-skeletons/system/serial/CMakeFiles/serial_example.dir/examples/serial_example.cc.o.provides

F110-fall2018-skeletons/system/serial/CMakeFiles/serial_example.dir/examples/serial_example.cc.o.provides.build: F110-fall2018-skeletons/system/serial/CMakeFiles/serial_example.dir/examples/serial_example.cc.o


# Object files for target serial_example
serial_example_OBJECTS = \
"CMakeFiles/serial_example.dir/examples/serial_example.cc.o"

# External object files for target serial_example
serial_example_EXTERNAL_OBJECTS =

/home/nvidia/group1_ws/devel/lib/serial/serial_example: F110-fall2018-skeletons/system/serial/CMakeFiles/serial_example.dir/examples/serial_example.cc.o
/home/nvidia/group1_ws/devel/lib/serial/serial_example: F110-fall2018-skeletons/system/serial/CMakeFiles/serial_example.dir/build.make
/home/nvidia/group1_ws/devel/lib/serial/serial_example: /home/nvidia/group1_ws/devel/lib/libserial.so
/home/nvidia/group1_ws/devel/lib/serial/serial_example: F110-fall2018-skeletons/system/serial/CMakeFiles/serial_example.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/nvidia/group1_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/nvidia/group1_ws/devel/lib/serial/serial_example"
	cd /home/nvidia/group1_ws/build/F110-fall2018-skeletons/system/serial && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/serial_example.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
F110-fall2018-skeletons/system/serial/CMakeFiles/serial_example.dir/build: /home/nvidia/group1_ws/devel/lib/serial/serial_example

.PHONY : F110-fall2018-skeletons/system/serial/CMakeFiles/serial_example.dir/build

F110-fall2018-skeletons/system/serial/CMakeFiles/serial_example.dir/requires: F110-fall2018-skeletons/system/serial/CMakeFiles/serial_example.dir/examples/serial_example.cc.o.requires

.PHONY : F110-fall2018-skeletons/system/serial/CMakeFiles/serial_example.dir/requires

F110-fall2018-skeletons/system/serial/CMakeFiles/serial_example.dir/clean:
	cd /home/nvidia/group1_ws/build/F110-fall2018-skeletons/system/serial && $(CMAKE_COMMAND) -P CMakeFiles/serial_example.dir/cmake_clean.cmake
.PHONY : F110-fall2018-skeletons/system/serial/CMakeFiles/serial_example.dir/clean

F110-fall2018-skeletons/system/serial/CMakeFiles/serial_example.dir/depend:
	cd /home/nvidia/group1_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/nvidia/group1_ws/src /home/nvidia/group1_ws/src/F110-fall2018-skeletons/system/serial /home/nvidia/group1_ws/build /home/nvidia/group1_ws/build/F110-fall2018-skeletons/system/serial /home/nvidia/group1_ws/build/F110-fall2018-skeletons/system/serial/CMakeFiles/serial_example.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : F110-fall2018-skeletons/system/serial/CMakeFiles/serial_example.dir/depend
