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
include CMakeFiles/RobotVision_Sirius.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/RobotVision_Sirius.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/RobotVision_Sirius.dir/flags.make

CMakeFiles/RobotVision_Sirius.dir/base/RobotVision_Sirius.cpp.o: CMakeFiles/RobotVision_Sirius.dir/flags.make
CMakeFiles/RobotVision_Sirius.dir/base/RobotVision_Sirius.cpp.o: ../base/RobotVision_Sirius.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ccong/Desktop/RobotVision_Sirius/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/RobotVision_Sirius.dir/base/RobotVision_Sirius.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/RobotVision_Sirius.dir/base/RobotVision_Sirius.cpp.o -c /home/ccong/Desktop/RobotVision_Sirius/base/RobotVision_Sirius.cpp

CMakeFiles/RobotVision_Sirius.dir/base/RobotVision_Sirius.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/RobotVision_Sirius.dir/base/RobotVision_Sirius.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ccong/Desktop/RobotVision_Sirius/base/RobotVision_Sirius.cpp > CMakeFiles/RobotVision_Sirius.dir/base/RobotVision_Sirius.cpp.i

CMakeFiles/RobotVision_Sirius.dir/base/RobotVision_Sirius.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/RobotVision_Sirius.dir/base/RobotVision_Sirius.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ccong/Desktop/RobotVision_Sirius/base/RobotVision_Sirius.cpp -o CMakeFiles/RobotVision_Sirius.dir/base/RobotVision_Sirius.cpp.s

# Object files for target RobotVision_Sirius
RobotVision_Sirius_OBJECTS = \
"CMakeFiles/RobotVision_Sirius.dir/base/RobotVision_Sirius.cpp.o"

# External object files for target RobotVision_Sirius
RobotVision_Sirius_EXTERNAL_OBJECTS =

bin/RobotVision_Sirius: CMakeFiles/RobotVision_Sirius.dir/base/RobotVision_Sirius.cpp.o
bin/RobotVision_Sirius: CMakeFiles/RobotVision_Sirius.dir/build.make
bin/RobotVision_Sirius: 3rdparty/fmt/libfmtd.a
bin/RobotVision_Sirius: lib/librvs-mv-video-capture.so
bin/RobotVision_Sirius: lib/librvs-uart-serial.so
bin/RobotVision_Sirius: lib/librvs-basic-pnp.so
bin/RobotVision_Sirius: lib/librvs-basic-armor.so
bin/RobotVision_Sirius: lib/librvs-basic-buff.so
bin/RobotVision_Sirius: lib/librvs-basic-kalman.so
bin/RobotVision_Sirius: lib/librvs-video-record.so
bin/RobotVision_Sirius: lib/librvs-onnx-inferring.so
bin/RobotVision_Sirius: lib/librvs-basic-roi.so
bin/RobotVision_Sirius: ../3rdparty/mindvision/linux/lib/x64/libMVSDK.so
bin/RobotVision_Sirius: lib/librvs-fan-armor.so
bin/RobotVision_Sirius: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.4.2.0
bin/RobotVision_Sirius: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.4.2.0
bin/RobotVision_Sirius: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.4.2.0
bin/RobotVision_Sirius: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.4.2.0
bin/RobotVision_Sirius: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.4.2.0
bin/RobotVision_Sirius: /usr/lib/x86_64-linux-gnu/libopencv_dnn_objdetect.so.4.2.0
bin/RobotVision_Sirius: /usr/lib/x86_64-linux-gnu/libopencv_dnn_superres.so.4.2.0
bin/RobotVision_Sirius: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.4.2.0
bin/RobotVision_Sirius: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.4.2.0
bin/RobotVision_Sirius: /usr/lib/x86_64-linux-gnu/libopencv_face.so.4.2.0
bin/RobotVision_Sirius: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.4.2.0
bin/RobotVision_Sirius: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.4.2.0
bin/RobotVision_Sirius: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.4.2.0
bin/RobotVision_Sirius: /usr/lib/x86_64-linux-gnu/libopencv_hfs.so.4.2.0
bin/RobotVision_Sirius: /usr/lib/x86_64-linux-gnu/libopencv_img_hash.so.4.2.0
bin/RobotVision_Sirius: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.4.2.0
bin/RobotVision_Sirius: /usr/lib/x86_64-linux-gnu/libopencv_quality.so.4.2.0
bin/RobotVision_Sirius: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.4.2.0
bin/RobotVision_Sirius: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.4.2.0
bin/RobotVision_Sirius: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.4.2.0
bin/RobotVision_Sirius: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.4.2.0
bin/RobotVision_Sirius: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.4.2.0
bin/RobotVision_Sirius: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.4.2.0
bin/RobotVision_Sirius: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.4.2.0
bin/RobotVision_Sirius: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.4.2.0
bin/RobotVision_Sirius: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.4.2.0
bin/RobotVision_Sirius: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.4.2.0
bin/RobotVision_Sirius: /usr/lib/x86_64-linux-gnu/libopencv_tracking.so.4.2.0
bin/RobotVision_Sirius: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.4.2.0
bin/RobotVision_Sirius: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.4.2.0
bin/RobotVision_Sirius: /usr/lib/x86_64-linux-gnu/libopencv_text.so.4.2.0
bin/RobotVision_Sirius: /usr/lib/x86_64-linux-gnu/libopencv_dnn.so.4.2.0
bin/RobotVision_Sirius: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.4.2.0
bin/RobotVision_Sirius: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.4.2.0
bin/RobotVision_Sirius: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.4.2.0
bin/RobotVision_Sirius: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.4.2.0
bin/RobotVision_Sirius: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.4.2.0
bin/RobotVision_Sirius: /usr/lib/x86_64-linux-gnu/libopencv_video.so.4.2.0
bin/RobotVision_Sirius: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.4.2.0
bin/RobotVision_Sirius: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.4.2.0
bin/RobotVision_Sirius: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.4.2.0
bin/RobotVision_Sirius: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.4.2.0
bin/RobotVision_Sirius: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.4.2.0
bin/RobotVision_Sirius: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.4.2.0
bin/RobotVision_Sirius: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.4.2.0
bin/RobotVision_Sirius: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.4.2.0
bin/RobotVision_Sirius: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.4.2.0
bin/RobotVision_Sirius: /usr/lib/x86_64-linux-gnu/libopencv_core.so.4.2.0
bin/RobotVision_Sirius: CMakeFiles/RobotVision_Sirius.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ccong/Desktop/RobotVision_Sirius/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable bin/RobotVision_Sirius"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/RobotVision_Sirius.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/RobotVision_Sirius.dir/build: bin/RobotVision_Sirius

.PHONY : CMakeFiles/RobotVision_Sirius.dir/build

CMakeFiles/RobotVision_Sirius.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/RobotVision_Sirius.dir/cmake_clean.cmake
.PHONY : CMakeFiles/RobotVision_Sirius.dir/clean

CMakeFiles/RobotVision_Sirius.dir/depend:
	cd /home/ccong/Desktop/RobotVision_Sirius/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ccong/Desktop/RobotVision_Sirius /home/ccong/Desktop/RobotVision_Sirius /home/ccong/Desktop/RobotVision_Sirius/build /home/ccong/Desktop/RobotVision_Sirius/build /home/ccong/Desktop/RobotVision_Sirius/build/CMakeFiles/RobotVision_Sirius.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/RobotVision_Sirius.dir/depend

