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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /usr/local/driveworks-0.6/samples

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /usr/local/driveworks/samples

# Include any dependencies generated for this target.
include src/laneDetection/laneDetection/CMakeFiles/sample_lane_detection.dir/depend.make

# Include the progress variables for this target.
include src/laneDetection/laneDetection/CMakeFiles/sample_lane_detection.dir/progress.make

# Include the compile flags for this target's objects.
include src/laneDetection/laneDetection/CMakeFiles/sample_lane_detection.dir/flags.make

src/laneDetection/laneDetection/CMakeFiles/sample_lane_detection.dir/main.cpp.o: src/laneDetection/laneDetection/CMakeFiles/sample_lane_detection.dir/flags.make
src/laneDetection/laneDetection/CMakeFiles/sample_lane_detection.dir/main.cpp.o: /usr/local/driveworks-0.6/samples/src/laneDetection/laneDetection/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/usr/local/driveworks/samples/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/laneDetection/laneDetection/CMakeFiles/sample_lane_detection.dir/main.cpp.o"
	cd /usr/local/driveworks/samples/src/laneDetection/laneDetection && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/sample_lane_detection.dir/main.cpp.o -c /usr/local/driveworks-0.6/samples/src/laneDetection/laneDetection/main.cpp

src/laneDetection/laneDetection/CMakeFiles/sample_lane_detection.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sample_lane_detection.dir/main.cpp.i"
	cd /usr/local/driveworks/samples/src/laneDetection/laneDetection && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /usr/local/driveworks-0.6/samples/src/laneDetection/laneDetection/main.cpp > CMakeFiles/sample_lane_detection.dir/main.cpp.i

src/laneDetection/laneDetection/CMakeFiles/sample_lane_detection.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sample_lane_detection.dir/main.cpp.s"
	cd /usr/local/driveworks/samples/src/laneDetection/laneDetection && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /usr/local/driveworks-0.6/samples/src/laneDetection/laneDetection/main.cpp -o CMakeFiles/sample_lane_detection.dir/main.cpp.s

src/laneDetection/laneDetection/CMakeFiles/sample_lane_detection.dir/main.cpp.o.requires:

.PHONY : src/laneDetection/laneDetection/CMakeFiles/sample_lane_detection.dir/main.cpp.o.requires

src/laneDetection/laneDetection/CMakeFiles/sample_lane_detection.dir/main.cpp.o.provides: src/laneDetection/laneDetection/CMakeFiles/sample_lane_detection.dir/main.cpp.o.requires
	$(MAKE) -f src/laneDetection/laneDetection/CMakeFiles/sample_lane_detection.dir/build.make src/laneDetection/laneDetection/CMakeFiles/sample_lane_detection.dir/main.cpp.o.provides.build
.PHONY : src/laneDetection/laneDetection/CMakeFiles/sample_lane_detection.dir/main.cpp.o.provides

src/laneDetection/laneDetection/CMakeFiles/sample_lane_detection.dir/main.cpp.o.provides.build: src/laneDetection/laneDetection/CMakeFiles/sample_lane_detection.dir/main.cpp.o


# Object files for target sample_lane_detection
sample_lane_detection_OBJECTS = \
"CMakeFiles/sample_lane_detection.dir/main.cpp.o"

# External object files for target sample_lane_detection
sample_lane_detection_EXTERNAL_OBJECTS =

src/laneDetection/laneDetection/sample_lane_detection: src/laneDetection/laneDetection/CMakeFiles/sample_lane_detection.dir/main.cpp.o
src/laneDetection/laneDetection/sample_lane_detection: src/laneDetection/laneDetection/CMakeFiles/sample_lane_detection.dir/build.make
src/laneDetection/laneDetection/sample_lane_detection: src/laneDetection/laneDetection_common/libdw_samples_laneDetection_common.a
src/laneDetection/laneDetection/sample_lane_detection: src/dnn/dnn_common/libdw_samples_dnn_common.a
src/laneDetection/laneDetection/sample_lane_detection: src/framework/libdw_samples_framework.a
src/laneDetection/laneDetection/sample_lane_detection: /usr/local/driveworks-0.6/targets/x86_64-linux/lib/libdriveworks.so
src/laneDetection/laneDetection/sample_lane_detection: /usr/local/cuda/lib64/libcudart.so
src/laneDetection/laneDetection/sample_lane_detection: /usr/local/cuda/lib64/libcublas.so
src/laneDetection/laneDetection/sample_lane_detection: /usr/local/driveworks-0.6/samples/3rdparty/linux/glfw-3.1.1/lib/libglfw3.a
src/laneDetection/laneDetection/sample_lane_detection: /usr/local/driveworks-0.6/samples/3rdparty/linux/glew-1.13.0/lib/libGLEW.a
src/laneDetection/laneDetection/sample_lane_detection: /usr/lib/x86_64-linux-gnu/libSM.so
src/laneDetection/laneDetection/sample_lane_detection: /usr/lib/x86_64-linux-gnu/libICE.so
src/laneDetection/laneDetection/sample_lane_detection: /usr/lib/x86_64-linux-gnu/libX11.so
src/laneDetection/laneDetection/sample_lane_detection: /usr/lib/x86_64-linux-gnu/libXext.so
src/laneDetection/laneDetection/sample_lane_detection: /usr/lib/x86_64-linux-gnu/libXrandr.so
src/laneDetection/laneDetection/sample_lane_detection: /usr/lib/x86_64-linux-gnu/libXcursor.so
src/laneDetection/laneDetection/sample_lane_detection: /usr/lib/x86_64-linux-gnu/libXxf86vm.so
src/laneDetection/laneDetection/sample_lane_detection: /usr/lib/x86_64-linux-gnu/libXinerama.so
src/laneDetection/laneDetection/sample_lane_detection: /usr/lib/x86_64-linux-gnu/libXi.so
src/laneDetection/laneDetection/sample_lane_detection: src/laneDetection/laneDetection/CMakeFiles/sample_lane_detection.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/usr/local/driveworks/samples/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable sample_lane_detection"
	cd /usr/local/driveworks/samples/src/laneDetection/laneDetection && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/sample_lane_detection.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/laneDetection/laneDetection/CMakeFiles/sample_lane_detection.dir/build: src/laneDetection/laneDetection/sample_lane_detection

.PHONY : src/laneDetection/laneDetection/CMakeFiles/sample_lane_detection.dir/build

# Object files for target sample_lane_detection
sample_lane_detection_OBJECTS = \
"CMakeFiles/sample_lane_detection.dir/main.cpp.o"

# External object files for target sample_lane_detection
sample_lane_detection_EXTERNAL_OBJECTS =

src/laneDetection/laneDetection/CMakeFiles/CMakeRelink.dir/sample_lane_detection: src/laneDetection/laneDetection/CMakeFiles/sample_lane_detection.dir/main.cpp.o
src/laneDetection/laneDetection/CMakeFiles/CMakeRelink.dir/sample_lane_detection: src/laneDetection/laneDetection/CMakeFiles/sample_lane_detection.dir/build.make
src/laneDetection/laneDetection/CMakeFiles/CMakeRelink.dir/sample_lane_detection: src/laneDetection/laneDetection_common/libdw_samples_laneDetection_common.a
src/laneDetection/laneDetection/CMakeFiles/CMakeRelink.dir/sample_lane_detection: src/dnn/dnn_common/libdw_samples_dnn_common.a
src/laneDetection/laneDetection/CMakeFiles/CMakeRelink.dir/sample_lane_detection: src/framework/libdw_samples_framework.a
src/laneDetection/laneDetection/CMakeFiles/CMakeRelink.dir/sample_lane_detection: /usr/local/driveworks-0.6/targets/x86_64-linux/lib/libdriveworks.so
src/laneDetection/laneDetection/CMakeFiles/CMakeRelink.dir/sample_lane_detection: /usr/local/cuda/lib64/libcudart.so
src/laneDetection/laneDetection/CMakeFiles/CMakeRelink.dir/sample_lane_detection: /usr/local/cuda/lib64/libcublas.so
src/laneDetection/laneDetection/CMakeFiles/CMakeRelink.dir/sample_lane_detection: /usr/local/driveworks-0.6/samples/3rdparty/linux/glfw-3.1.1/lib/libglfw3.a
src/laneDetection/laneDetection/CMakeFiles/CMakeRelink.dir/sample_lane_detection: /usr/local/driveworks-0.6/samples/3rdparty/linux/glew-1.13.0/lib/libGLEW.a
src/laneDetection/laneDetection/CMakeFiles/CMakeRelink.dir/sample_lane_detection: /usr/lib/x86_64-linux-gnu/libSM.so
src/laneDetection/laneDetection/CMakeFiles/CMakeRelink.dir/sample_lane_detection: /usr/lib/x86_64-linux-gnu/libICE.so
src/laneDetection/laneDetection/CMakeFiles/CMakeRelink.dir/sample_lane_detection: /usr/lib/x86_64-linux-gnu/libX11.so
src/laneDetection/laneDetection/CMakeFiles/CMakeRelink.dir/sample_lane_detection: /usr/lib/x86_64-linux-gnu/libXext.so
src/laneDetection/laneDetection/CMakeFiles/CMakeRelink.dir/sample_lane_detection: /usr/lib/x86_64-linux-gnu/libXrandr.so
src/laneDetection/laneDetection/CMakeFiles/CMakeRelink.dir/sample_lane_detection: /usr/lib/x86_64-linux-gnu/libXcursor.so
src/laneDetection/laneDetection/CMakeFiles/CMakeRelink.dir/sample_lane_detection: /usr/lib/x86_64-linux-gnu/libXxf86vm.so
src/laneDetection/laneDetection/CMakeFiles/CMakeRelink.dir/sample_lane_detection: /usr/lib/x86_64-linux-gnu/libXinerama.so
src/laneDetection/laneDetection/CMakeFiles/CMakeRelink.dir/sample_lane_detection: /usr/lib/x86_64-linux-gnu/libXi.so
src/laneDetection/laneDetection/CMakeFiles/CMakeRelink.dir/sample_lane_detection: src/laneDetection/laneDetection/CMakeFiles/sample_lane_detection.dir/relink.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/usr/local/driveworks/samples/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable CMakeFiles/CMakeRelink.dir/sample_lane_detection"
	cd /usr/local/driveworks/samples/src/laneDetection/laneDetection && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/sample_lane_detection.dir/relink.txt --verbose=$(VERBOSE)

# Rule to relink during preinstall.
src/laneDetection/laneDetection/CMakeFiles/sample_lane_detection.dir/preinstall: src/laneDetection/laneDetection/CMakeFiles/CMakeRelink.dir/sample_lane_detection

.PHONY : src/laneDetection/laneDetection/CMakeFiles/sample_lane_detection.dir/preinstall

src/laneDetection/laneDetection/CMakeFiles/sample_lane_detection.dir/requires: src/laneDetection/laneDetection/CMakeFiles/sample_lane_detection.dir/main.cpp.o.requires

.PHONY : src/laneDetection/laneDetection/CMakeFiles/sample_lane_detection.dir/requires

src/laneDetection/laneDetection/CMakeFiles/sample_lane_detection.dir/clean:
	cd /usr/local/driveworks/samples/src/laneDetection/laneDetection && $(CMAKE_COMMAND) -P CMakeFiles/sample_lane_detection.dir/cmake_clean.cmake
.PHONY : src/laneDetection/laneDetection/CMakeFiles/sample_lane_detection.dir/clean

src/laneDetection/laneDetection/CMakeFiles/sample_lane_detection.dir/depend:
	cd /usr/local/driveworks/samples && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /usr/local/driveworks-0.6/samples /usr/local/driveworks-0.6/samples/src/laneDetection/laneDetection /usr/local/driveworks/samples /usr/local/driveworks/samples/src/laneDetection/laneDetection /usr/local/driveworks/samples/src/laneDetection/laneDetection/CMakeFiles/sample_lane_detection.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/laneDetection/laneDetection/CMakeFiles/sample_lane_detection.dir/depend

