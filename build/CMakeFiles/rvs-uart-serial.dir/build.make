# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/ccong/Desktop/RobotVision_Sirius

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ccong/Desktop/RobotVision_Sirius/build

# Include any dependencies generated for this target.
include CMakeFiles/rvs-uart-serial.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/rvs-uart-serial.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/rvs-uart-serial.dir/flags.make

CMakeFiles/rvs-uart-serial.dir/devices/serial/uart_serial.cpp.o: CMakeFiles/rvs-uart-serial.dir/flags.make
CMakeFiles/rvs-uart-serial.dir/devices/serial/uart_serial.cpp.o: ../devices/serial/uart_serial.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ccong/Desktop/RobotVision_Sirius/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/rvs-uart-serial.dir/devices/serial/uart_serial.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/rvs-uart-serial.dir/devices/serial/uart_serial.cpp.o -c /home/ccong/Desktop/RobotVision_Sirius/devices/serial/uart_serial.cpp

CMakeFiles/rvs-uart-serial.dir/devices/serial/uart_serial.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rvs-uart-serial.dir/devices/serial/uart_serial.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ccong/Desktop/RobotVision_Sirius/devices/serial/uart_serial.cpp > CMakeFiles/rvs-uart-serial.dir/devices/serial/uart_serial.cpp.i

CMakeFiles/rvs-uart-serial.dir/devices/serial/uart_serial.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rvs-uart-serial.dir/devices/serial/uart_serial.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ccong/Desktop/RobotVision_Sirius/devices/serial/uart_serial.cpp -o CMakeFiles/rvs-uart-serial.dir/devices/serial/uart_serial.cpp.s

# Object files for target rvs-uart-serial
rvs__uart__serial_OBJECTS = \
"CMakeFiles/rvs-uart-serial.dir/devices/serial/uart_serial.cpp.o"

# External object files for target rvs-uart-serial
rvs__uart__serial_EXTERNAL_OBJECTS =

lib/librvs-uart-serial.so: CMakeFiles/rvs-uart-serial.dir/devices/serial/uart_serial.cpp.o
lib/librvs-uart-serial.so: CMakeFiles/rvs-uart-serial.dir/build.make
lib/librvs-uart-serial.so: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.4.2.0
lib/librvs-uart-serial.so: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.4.2.0
lib/librvs-uart-serial.so: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.4.2.0
lib/librvs-uart-serial.so: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.4.2.0
lib/librvs-uart-serial.so: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.4.2.0
lib/librvs-uart-serial.so: /usr/lib/x86_64-linux-gnu/libopencv_dnn_objdetect.so.4.2.0
lib/librvs-uart-serial.so: /usr/lib/x86_64-linux-gnu/libopencv_dnn_superres.so.4.2.0
lib/librvs-uart-serial.so: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.4.2.0
lib/librvs-uart-serial.so: /usr/lib/x86_64-linux-gnu/libopencv_face.so.4.2.0
lib/librvs-uart-serial.so: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.4.2.0
lib/librvs-uart-serial.so: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.4.2.0
lib/librvs-uart-serial.so: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.4.2.0
lib/librvs-uart-serial.so: /usr/lib/x86_64-linux-gnu/libopencv_hfs.so.4.2.0
lib/librvs-uart-serial.so: /usr/lib/x86_64-linux-gnu/libopencv_img_hash.so.4.2.0
lib/librvs-uart-serial.so: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.4.2.0
lib/librvs-uart-serial.so: /usr/lib/x86_64-linux-gnu/libopencv_quality.so.4.2.0
lib/librvs-uart-serial.so: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.4.2.0
lib/librvs-uart-serial.so: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.4.2.0
lib/librvs-uart-serial.so: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.4.2.0
lib/librvs-uart-serial.so: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.4.2.0
lib/librvs-uart-serial.so: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.4.2.0
lib/librvs-uart-serial.so: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.4.2.0
lib/librvs-uart-serial.so: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.4.2.0
lib/librvs-uart-serial.so: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.4.2.0
lib/librvs-uart-serial.so: /usr/lib/x86_64-linux-gnu/libopencv_tracking.so.4.2.0
lib/librvs-uart-serial.so: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.4.2.0
lib/librvs-uart-serial.so: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.4.2.0
lib/librvs-uart-serial.so: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.4.2.0
lib/librvs-uart-serial.so: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.4.2.0
lib/librvs-uart-serial.so: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.4.2.0
lib/librvs-uart-serial.so: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.4.2.0
lib/librvs-uart-serial.so: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.4.2.0
lib/librvs-uart-serial.so: /usr/lib/x86_64-linux-gnu/libopencv_text.so.4.2.0
lib/librvs-uart-serial.so: /usr/lib/x86_64-linux-gnu/libopencv_dnn.so.4.2.0
lib/librvs-uart-serial.so: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.4.2.0
lib/librvs-uart-serial.so: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.4.2.0
lib/librvs-uart-serial.so: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.4.2.0
lib/librvs-uart-serial.so: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.4.2.0
lib/librvs-uart-serial.so: /usr/lib/x86_64-linux-gnu/libopencv_video.so.4.2.0
lib/librvs-uart-serial.so: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.4.2.0
lib/librvs-uart-serial.so: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.4.2.0
lib/librvs-uart-serial.so: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.4.2.0
lib/librvs-uart-serial.so: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.4.2.0
lib/librvs-uart-serial.so: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.4.2.0
lib/librvs-uart-serial.so: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.4.2.0
lib/librvs-uart-serial.so: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.4.2.0
lib/librvs-uart-serial.so: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.4.2.0
lib/librvs-uart-serial.so: /usr/lib/x86_64-linux-gnu/libopencv_core.so.4.2.0
lib/librvs-uart-serial.so: CMakeFiles/rvs-uart-serial.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ccong/Desktop/RobotVision_Sirius/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library lib/librvs-uart-serial.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/rvs-uart-serial.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/rvs-uart-serial.dir/build: lib/librvs-uart-serial.so

.PHONY : CMakeFiles/rvs-uart-serial.dir/build

CMakeFiles/rvs-uart-serial.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/rvs-uart-serial.dir/cmake_clean.cmake
.PHONY : CMakeFiles/rvs-uart-serial.dir/clean

CMakeFiles/rvs-uart-serial.dir/depend:
	cd /home/ccong/Desktop/RobotVision_Sirius/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ccong/Desktop/RobotVision_Sirius /home/ccong/Desktop/RobotVision_Sirius /home/ccong/Desktop/RobotVision_Sirius/build /home/ccong/Desktop/RobotVision_Sirius/build /home/ccong/Desktop/RobotVision_Sirius/build/CMakeFiles/rvs-uart-serial.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/rvs-uart-serial.dir/depend

