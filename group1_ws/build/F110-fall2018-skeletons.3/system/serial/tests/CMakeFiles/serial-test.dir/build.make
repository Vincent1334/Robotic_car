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
include F110-fall2018-skeletons.3/system/serial/tests/CMakeFiles/serial-test.dir/depend.make

# Include the progress variables for this target.
include F110-fall2018-skeletons.3/system/serial/tests/CMakeFiles/serial-test.dir/progress.make

# Include the compile flags for this target's objects.
include F110-fall2018-skeletons.3/system/serial/tests/CMakeFiles/serial-test.dir/flags.make

F110-fall2018-skeletons.3/system/serial/tests/CMakeFiles/serial-test.dir/unix_serial_tests.cc.o: F110-fall2018-skeletons.3/system/serial/tests/CMakeFiles/serial-test.dir/flags.make
F110-fall2018-skeletons.3/system/serial/tests/CMakeFiles/serial-test.dir/unix_serial_tests.cc.o: /home/nvidia/group1_ws/src/F110-fall2018-skeletons.3/system/serial/tests/unix_serial_tests.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nvidia/group1_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object F110-fall2018-skeletons.3/system/serial/tests/CMakeFiles/serial-test.dir/unix_serial_tests.cc.o"
	cd /home/nvidia/group1_ws/build/F110-fall2018-skeletons.3/system/serial/tests && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/serial-test.dir/unix_serial_tests.cc.o -c /home/nvidia/group1_ws/src/F110-fall2018-skeletons.3/system/serial/tests/unix_serial_tests.cc

F110-fall2018-skeletons.3/system/serial/tests/CMakeFiles/serial-test.dir/unix_serial_tests.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/serial-test.dir/unix_serial_tests.cc.i"
	cd /home/nvidia/group1_ws/build/F110-fall2018-skeletons.3/system/serial/tests && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nvidia/group1_ws/src/F110-fall2018-skeletons.3/system/serial/tests/unix_serial_tests.cc > CMakeFiles/serial-test.dir/unix_serial_tests.cc.i

F110-fall2018-skeletons.3/system/serial/tests/CMakeFiles/serial-test.dir/unix_serial_tests.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/serial-test.dir/unix_serial_tests.cc.s"
	cd /home/nvidia/group1_ws/build/F110-fall2018-skeletons.3/system/serial/tests && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nvidia/group1_ws/src/F110-fall2018-skeletons.3/system/serial/tests/unix_serial_tests.cc -o CMakeFiles/serial-test.dir/unix_serial_tests.cc.s

F110-fall2018-skeletons.3/system/serial/tests/CMakeFiles/serial-test.dir/unix_serial_tests.cc.o.requires:

.PHONY : F110-fall2018-skeletons.3/system/serial/tests/CMakeFiles/serial-test.dir/unix_serial_tests.cc.o.requires

F110-fall2018-skeletons.3/system/serial/tests/CMakeFiles/serial-test.dir/unix_serial_tests.cc.o.provides: F110-fall2018-skeletons.3/system/serial/tests/CMakeFiles/serial-test.dir/unix_serial_tests.cc.o.requires
	$(MAKE) -f F110-fall2018-skeletons.3/system/serial/tests/CMakeFiles/serial-test.dir/build.make F110-fall2018-skeletons.3/system/serial/tests/CMakeFiles/serial-test.dir/unix_serial_tests.cc.o.provides.build
.PHONY : F110-fall2018-skeletons.3/system/serial/tests/CMakeFiles/serial-test.dir/unix_serial_tests.cc.o.provides

F110-fall2018-skeletons.3/system/serial/tests/CMakeFiles/serial-test.dir/unix_serial_tests.cc.o.provides.build: F110-fall2018-skeletons.3/system/serial/tests/CMakeFiles/serial-test.dir/unix_serial_tests.cc.o


# Object files for target serial-test
serial__test_OBJECTS = \
"CMakeFiles/serial-test.dir/unix_serial_tests.cc.o"

# External object files for target serial-test
serial__test_EXTERNAL_OBJECTS =

/home/nvidia/group1_ws/devel/lib/serial/serial-test: F110-fall2018-skeletons.3/system/serial/tests/CMakeFiles/serial-test.dir/unix_serial_tests.cc.o
/home/nvidia/group1_ws/devel/lib/serial/serial-test: F110-fall2018-skeletons.3/system/serial/tests/CMakeFiles/serial-test.dir/build.make
/home/nvidia/group1_ws/devel/lib/serial/serial-test: gtest/gtest/libgtest.so
/home/nvidia/group1_ws/devel/lib/serial/serial-test: /home/nvidia/group1_ws/devel/lib/libserial.so
/home/nvidia/group1_ws/devel/lib/serial/serial-test: F110-fall2018-skeletons.3/system/serial/tests/CMakeFiles/serial-test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/nvidia/group1_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/nvidia/group1_ws/devel/lib/serial/serial-test"
	cd /home/nvidia/group1_ws/build/F110-fall2018-skeletons.3/system/serial/tests && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/serial-test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
F110-fall2018-skeletons.3/system/serial/tests/CMakeFiles/serial-test.dir/build: /home/nvidia/group1_ws/devel/lib/serial/serial-test

.PHONY : F110-fall2018-skeletons.3/system/serial/tests/CMakeFiles/serial-test.dir/build

F110-fall2018-skeletons.3/system/serial/tests/CMakeFiles/serial-test.dir/requires: F110-fall2018-skeletons.3/system/serial/tests/CMakeFiles/serial-test.dir/unix_serial_tests.cc.o.requires

.PHONY : F110-fall2018-skeletons.3/system/serial/tests/CMakeFiles/serial-test.dir/requires

F110-fall2018-skeletons.3/system/serial/tests/CMakeFiles/serial-test.dir/clean:
	cd /home/nvidia/group1_ws/build/F110-fall2018-skeletons.3/system/serial/tests && $(CMAKE_COMMAND) -P CMakeFiles/serial-test.dir/cmake_clean.cmake
.PHONY : F110-fall2018-skeletons.3/system/serial/tests/CMakeFiles/serial-test.dir/clean

F110-fall2018-skeletons.3/system/serial/tests/CMakeFiles/serial-test.dir/depend:
	cd /home/nvidia/group1_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/nvidia/group1_ws/src /home/nvidia/group1_ws/src/F110-fall2018-skeletons.3/system/serial/tests /home/nvidia/group1_ws/build /home/nvidia/group1_ws/build/F110-fall2018-skeletons.3/system/serial/tests /home/nvidia/group1_ws/build/F110-fall2018-skeletons.3/system/serial/tests/CMakeFiles/serial-test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : F110-fall2018-skeletons.3/system/serial/tests/CMakeFiles/serial-test.dir/depend

