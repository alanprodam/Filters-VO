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
CMAKE_SOURCE_DIR = /home/acta/Filters-VO

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/acta/Filters-VO/build

# Include any dependencies generated for this target.
include CMakeFiles/motionTracking.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/motionTracking.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/motionTracking.dir/flags.make

CMakeFiles/motionTracking.dir/src/motionTracking.cpp.o: CMakeFiles/motionTracking.dir/flags.make
CMakeFiles/motionTracking.dir/src/motionTracking.cpp.o: ../src/motionTracking.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/acta/Filters-VO/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/motionTracking.dir/src/motionTracking.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/motionTracking.dir/src/motionTracking.cpp.o -c /home/acta/Filters-VO/src/motionTracking.cpp

CMakeFiles/motionTracking.dir/src/motionTracking.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/motionTracking.dir/src/motionTracking.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/acta/Filters-VO/src/motionTracking.cpp > CMakeFiles/motionTracking.dir/src/motionTracking.cpp.i

CMakeFiles/motionTracking.dir/src/motionTracking.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/motionTracking.dir/src/motionTracking.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/acta/Filters-VO/src/motionTracking.cpp -o CMakeFiles/motionTracking.dir/src/motionTracking.cpp.s

CMakeFiles/motionTracking.dir/src/motionTracking.cpp.o.requires:

.PHONY : CMakeFiles/motionTracking.dir/src/motionTracking.cpp.o.requires

CMakeFiles/motionTracking.dir/src/motionTracking.cpp.o.provides: CMakeFiles/motionTracking.dir/src/motionTracking.cpp.o.requires
	$(MAKE) -f CMakeFiles/motionTracking.dir/build.make CMakeFiles/motionTracking.dir/src/motionTracking.cpp.o.provides.build
.PHONY : CMakeFiles/motionTracking.dir/src/motionTracking.cpp.o.provides

CMakeFiles/motionTracking.dir/src/motionTracking.cpp.o.provides.build: CMakeFiles/motionTracking.dir/src/motionTracking.cpp.o


# Object files for target motionTracking
motionTracking_OBJECTS = \
"CMakeFiles/motionTracking.dir/src/motionTracking.cpp.o"

# External object files for target motionTracking
motionTracking_EXTERNAL_OBJECTS =

motionTracking: CMakeFiles/motionTracking.dir/src/motionTracking.cpp.o
motionTracking: CMakeFiles/motionTracking.dir/build.make
motionTracking: /usr/local/lib/libopencv_stitching.so.3.4.4
motionTracking: /usr/local/lib/libopencv_superres.so.3.4.4
motionTracking: /usr/local/lib/libopencv_videostab.so.3.4.4
motionTracking: /usr/local/lib/libopencv_aruco.so.3.4.4
motionTracking: /usr/local/lib/libopencv_img_hash.so.3.4.4
motionTracking: /usr/local/lib/libopencv_hdf.so.3.4.4
motionTracking: /usr/local/lib/libopencv_xobjdetect.so.3.4.4
motionTracking: /usr/local/lib/libopencv_rgbd.so.3.4.4
motionTracking: /usr/local/lib/libopencv_line_descriptor.so.3.4.4
motionTracking: /usr/local/lib/libopencv_ccalib.so.3.4.4
motionTracking: /usr/local/lib/libopencv_surface_matching.so.3.4.4
motionTracking: /usr/local/lib/libopencv_xphoto.so.3.4.4
motionTracking: /usr/local/lib/libopencv_tracking.so.3.4.4
motionTracking: /usr/local/lib/libopencv_sfm.so.3.4.4
motionTracking: /usr/local/lib/libopencv_fuzzy.so.3.4.4
motionTracking: /usr/local/lib/libopencv_reg.so.3.4.4
motionTracking: /usr/local/lib/libopencv_freetype.so.3.4.4
motionTracking: /usr/local/lib/libopencv_dnn_objdetect.so.3.4.4
motionTracking: /usr/local/lib/libopencv_stereo.so.3.4.4
motionTracking: /usr/local/lib/libopencv_hfs.so.3.4.4
motionTracking: /usr/local/lib/libopencv_face.so.3.4.4
motionTracking: /usr/local/lib/libopencv_saliency.so.3.4.4
motionTracking: /usr/local/lib/libopencv_datasets.so.3.4.4
motionTracking: /usr/local/lib/libopencv_bgsegm.so.3.4.4
motionTracking: /usr/local/lib/libopencv_xfeatures2d.so.3.4.4
motionTracking: /usr/local/lib/libopencv_plot.so.3.4.4
motionTracking: /usr/local/lib/libopencv_structured_light.so.3.4.4
motionTracking: /usr/local/lib/libopencv_optflow.so.3.4.4
motionTracking: /usr/local/lib/libopencv_dpm.so.3.4.4
motionTracking: /usr/local/lib/libopencv_bioinspired.so.3.4.4
motionTracking: /usr/local/lib/libopencv_text.so.3.4.4
motionTracking: /usr/local/lib/libopencv_ml.so.3.4.4
motionTracking: /usr/local/lib/libopencv_shape.so.3.4.4
motionTracking: /usr/local/lib/libopencv_dnn.so.3.4.4
motionTracking: /usr/local/lib/libopencv_photo.so.3.4.4
motionTracking: /usr/local/lib/libopencv_viz.so.3.4.4
motionTracking: /usr/local/lib/libopencv_phase_unwrapping.so.3.4.4
motionTracking: /usr/local/lib/libopencv_video.so.3.4.4
motionTracking: /usr/local/lib/libopencv_ximgproc.so.3.4.4
motionTracking: /usr/local/lib/libopencv_objdetect.so.3.4.4
motionTracking: /usr/local/lib/libopencv_calib3d.so.3.4.4
motionTracking: /usr/local/lib/libopencv_features2d.so.3.4.4
motionTracking: /usr/local/lib/libopencv_flann.so.3.4.4
motionTracking: /usr/local/lib/libopencv_highgui.so.3.4.4
motionTracking: /usr/local/lib/libopencv_videoio.so.3.4.4
motionTracking: /usr/local/lib/libopencv_imgcodecs.so.3.4.4
motionTracking: /usr/local/lib/libopencv_imgproc.so.3.4.4
motionTracking: /usr/local/lib/libopencv_core.so.3.4.4
motionTracking: CMakeFiles/motionTracking.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/acta/Filters-VO/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable motionTracking"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/motionTracking.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/motionTracking.dir/build: motionTracking

.PHONY : CMakeFiles/motionTracking.dir/build

CMakeFiles/motionTracking.dir/requires: CMakeFiles/motionTracking.dir/src/motionTracking.cpp.o.requires

.PHONY : CMakeFiles/motionTracking.dir/requires

CMakeFiles/motionTracking.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/motionTracking.dir/cmake_clean.cmake
.PHONY : CMakeFiles/motionTracking.dir/clean

CMakeFiles/motionTracking.dir/depend:
	cd /home/acta/Filters-VO/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/acta/Filters-VO /home/acta/Filters-VO /home/acta/Filters-VO/build /home/acta/Filters-VO/build /home/acta/Filters-VO/build/CMakeFiles/motionTracking.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/motionTracking.dir/depend

