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
CMAKE_SOURCE_DIR = /usr/local/driveworks-0.6/PilotNet

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /usr/local/driveworks-0.6/PilotNet/build-target

# Include any dependencies generated for this target.
include src/dnn/sample_object_dwdetector/CMakeFiles/sample_object_dwdetector.dir/depend.make

# Include the progress variables for this target.
include src/dnn/sample_object_dwdetector/CMakeFiles/sample_object_dwdetector.dir/progress.make

# Include the compile flags for this target's objects.
include src/dnn/sample_object_dwdetector/CMakeFiles/sample_object_dwdetector.dir/flags.make

src/dnn/sample_object_dwdetector/CMakeFiles/sample_object_dwdetector.dir/main.cpp.o: src/dnn/sample_object_dwdetector/CMakeFiles/sample_object_dwdetector.dir/flags.make
src/dnn/sample_object_dwdetector/CMakeFiles/sample_object_dwdetector.dir/main.cpp.o: ../src/dnn/sample_object_dwdetector/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/usr/local/driveworks-0.6/PilotNet/build-target/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/dnn/sample_object_dwdetector/CMakeFiles/sample_object_dwdetector.dir/main.cpp.o"
	cd /usr/local/driveworks-0.6/PilotNet/build-target/src/dnn/sample_object_dwdetector && /home/ubuntu/NVIDIA/Drive/5050bL_SDK/DriveSDK/drive-t186ref-linux/../toolchains/tegra-4.9-nv/usr/bin/aarch64-gnu-linux/aarch64-gnu-linux-g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/sample_object_dwdetector.dir/main.cpp.o -c /usr/local/driveworks-0.6/PilotNet/src/dnn/sample_object_dwdetector/main.cpp

src/dnn/sample_object_dwdetector/CMakeFiles/sample_object_dwdetector.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sample_object_dwdetector.dir/main.cpp.i"
	cd /usr/local/driveworks-0.6/PilotNet/build-target/src/dnn/sample_object_dwdetector && /home/ubuntu/NVIDIA/Drive/5050bL_SDK/DriveSDK/drive-t186ref-linux/../toolchains/tegra-4.9-nv/usr/bin/aarch64-gnu-linux/aarch64-gnu-linux-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /usr/local/driveworks-0.6/PilotNet/src/dnn/sample_object_dwdetector/main.cpp > CMakeFiles/sample_object_dwdetector.dir/main.cpp.i

src/dnn/sample_object_dwdetector/CMakeFiles/sample_object_dwdetector.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sample_object_dwdetector.dir/main.cpp.s"
	cd /usr/local/driveworks-0.6/PilotNet/build-target/src/dnn/sample_object_dwdetector && /home/ubuntu/NVIDIA/Drive/5050bL_SDK/DriveSDK/drive-t186ref-linux/../toolchains/tegra-4.9-nv/usr/bin/aarch64-gnu-linux/aarch64-gnu-linux-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /usr/local/driveworks-0.6/PilotNet/src/dnn/sample_object_dwdetector/main.cpp -o CMakeFiles/sample_object_dwdetector.dir/main.cpp.s

src/dnn/sample_object_dwdetector/CMakeFiles/sample_object_dwdetector.dir/main.cpp.o.requires:

.PHONY : src/dnn/sample_object_dwdetector/CMakeFiles/sample_object_dwdetector.dir/main.cpp.o.requires

src/dnn/sample_object_dwdetector/CMakeFiles/sample_object_dwdetector.dir/main.cpp.o.provides: src/dnn/sample_object_dwdetector/CMakeFiles/sample_object_dwdetector.dir/main.cpp.o.requires
	$(MAKE) -f src/dnn/sample_object_dwdetector/CMakeFiles/sample_object_dwdetector.dir/build.make src/dnn/sample_object_dwdetector/CMakeFiles/sample_object_dwdetector.dir/main.cpp.o.provides.build
.PHONY : src/dnn/sample_object_dwdetector/CMakeFiles/sample_object_dwdetector.dir/main.cpp.o.provides

src/dnn/sample_object_dwdetector/CMakeFiles/sample_object_dwdetector.dir/main.cpp.o.provides.build: src/dnn/sample_object_dwdetector/CMakeFiles/sample_object_dwdetector.dir/main.cpp.o


# Object files for target sample_object_dwdetector
sample_object_dwdetector_OBJECTS = \
"CMakeFiles/sample_object_dwdetector.dir/main.cpp.o"

# External object files for target sample_object_dwdetector
sample_object_dwdetector_EXTERNAL_OBJECTS =

src/dnn/sample_object_dwdetector/sample_object_dwdetector: src/dnn/sample_object_dwdetector/CMakeFiles/sample_object_dwdetector.dir/main.cpp.o
src/dnn/sample_object_dwdetector/sample_object_dwdetector: src/dnn/sample_object_dwdetector/CMakeFiles/sample_object_dwdetector.dir/build.make
src/dnn/sample_object_dwdetector/sample_object_dwdetector: src/framework/libdw_samples_framework.a
src/dnn/sample_object_dwdetector/sample_object_dwdetector: src/dnn/dnn_common/libdw_samples_dnn_common.a
src/dnn/sample_object_dwdetector/sample_object_dwdetector: /usr/local/driveworks-0.6/targets/aarch64-linux/lib/libdriveworks.so
src/dnn/sample_object_dwdetector/sample_object_dwdetector: /usr/local/cuda-9.0/targets/aarch64-linux/lib/libcudart.so
src/dnn/sample_object_dwdetector/sample_object_dwdetector: /usr/local/cuda-9.0/targets/aarch64-linux/lib/stubs/libcublas.so
src/dnn/sample_object_dwdetector/sample_object_dwdetector: /home/ubuntu/NVIDIA/Drive/5050bL_SDK/DriveSDK/drive-t186ref-linux/lib-target/libEGL.so
src/dnn/sample_object_dwdetector/sample_object_dwdetector: src/framework/libdw_samples_framework.a
src/dnn/sample_object_dwdetector/sample_object_dwdetector: ../3rdparty/linux-aarch64/vibrante/lib/libudev.so
src/dnn/sample_object_dwdetector/sample_object_dwdetector: ../3rdparty/linux-aarch64/vibrante/lib/libusb-1.0.so
src/dnn/sample_object_dwdetector/sample_object_dwdetector: ../3rdparty/linux-aarch64/vibrante_Xlibs/lib/libXrandr.so
src/dnn/sample_object_dwdetector/sample_object_dwdetector: ../3rdparty/linux-aarch64/vibrante_Xlibs/lib/libXinerama.so
src/dnn/sample_object_dwdetector/sample_object_dwdetector: ../3rdparty/linux-aarch64/vibrante_Xlibs/lib/libXi.so
src/dnn/sample_object_dwdetector/sample_object_dwdetector: ../3rdparty/linux-aarch64/vibrante_Xlibs/lib/libXcursor.so
src/dnn/sample_object_dwdetector/sample_object_dwdetector: ../3rdparty/linux-aarch64/glfw-3.1.1/lib/libglfw3.a
src/dnn/sample_object_dwdetector/sample_object_dwdetector: /home/ubuntu/NVIDIA/Drive/5050bL_SDK/DriveSDK/drive-t186ref-linux/lib-target/libEGL.so
src/dnn/sample_object_dwdetector/sample_object_dwdetector: src/dnn/sample_object_dwdetector/CMakeFiles/sample_object_dwdetector.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/usr/local/driveworks-0.6/PilotNet/build-target/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable sample_object_dwdetector"
	cd /usr/local/driveworks-0.6/PilotNet/build-target/src/dnn/sample_object_dwdetector && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/sample_object_dwdetector.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/dnn/sample_object_dwdetector/CMakeFiles/sample_object_dwdetector.dir/build: src/dnn/sample_object_dwdetector/sample_object_dwdetector

.PHONY : src/dnn/sample_object_dwdetector/CMakeFiles/sample_object_dwdetector.dir/build

src/dnn/sample_object_dwdetector/CMakeFiles/sample_object_dwdetector.dir/requires: src/dnn/sample_object_dwdetector/CMakeFiles/sample_object_dwdetector.dir/main.cpp.o.requires

.PHONY : src/dnn/sample_object_dwdetector/CMakeFiles/sample_object_dwdetector.dir/requires

src/dnn/sample_object_dwdetector/CMakeFiles/sample_object_dwdetector.dir/clean:
	cd /usr/local/driveworks-0.6/PilotNet/build-target/src/dnn/sample_object_dwdetector && $(CMAKE_COMMAND) -P CMakeFiles/sample_object_dwdetector.dir/cmake_clean.cmake
.PHONY : src/dnn/sample_object_dwdetector/CMakeFiles/sample_object_dwdetector.dir/clean

src/dnn/sample_object_dwdetector/CMakeFiles/sample_object_dwdetector.dir/depend:
	cd /usr/local/driveworks-0.6/PilotNet/build-target && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /usr/local/driveworks-0.6/PilotNet /usr/local/driveworks-0.6/PilotNet/src/dnn/sample_object_dwdetector /usr/local/driveworks-0.6/PilotNet/build-target /usr/local/driveworks-0.6/PilotNet/build-target/src/dnn/sample_object_dwdetector /usr/local/driveworks-0.6/PilotNet/build-target/src/dnn/sample_object_dwdetector/CMakeFiles/sample_object_dwdetector.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/dnn/sample_object_dwdetector/CMakeFiles/sample_object_dwdetector.dir/depend
