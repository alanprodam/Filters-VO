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
CMAKE_SOURCE_DIR = /home/alantavares/Filters-VO

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/alantavares/Filters-VO/build

# Include any dependencies generated for this target.
include CMakeFiles/motionTracking.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/motionTracking.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/motionTracking.dir/flags.make

CMakeFiles/motionTracking.dir/src/motionTracking.cpp.o: CMakeFiles/motionTracking.dir/flags.make
CMakeFiles/motionTracking.dir/src/motionTracking.cpp.o: ../src/motionTracking.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/alantavares/Filters-VO/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/motionTracking.dir/src/motionTracking.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/motionTracking.dir/src/motionTracking.cpp.o -c /home/alantavares/Filters-VO/src/motionTracking.cpp

CMakeFiles/motionTracking.dir/src/motionTracking.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/motionTracking.dir/src/motionTracking.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/alantavares/Filters-VO/src/motionTracking.cpp > CMakeFiles/motionTracking.dir/src/motionTracking.cpp.i

CMakeFiles/motionTracking.dir/src/motionTracking.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/motionTracking.dir/src/motionTracking.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/alantavares/Filters-VO/src/motionTracking.cpp -o CMakeFiles/motionTracking.dir/src/motionTracking.cpp.s

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
motionTracking: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_stitching3.so.3.3.1
motionTracking: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_superres3.so.3.3.1
motionTracking: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_videostab3.so.3.3.1
motionTracking: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_aruco3.so.3.3.1
motionTracking: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_bgsegm3.so.3.3.1
motionTracking: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_bioinspired3.so.3.3.1
motionTracking: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ccalib3.so.3.3.1
motionTracking: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_cvv3.so.3.3.1
motionTracking: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_dpm3.so.3.3.1
motionTracking: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_face3.so.3.3.1
motionTracking: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_fuzzy3.so.3.3.1
motionTracking: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_hdf3.so.3.3.1
motionTracking: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_img_hash3.so.3.3.1
motionTracking: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_line_descriptor3.so.3.3.1
motionTracking: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_optflow3.so.3.3.1
motionTracking: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_reg3.so.3.3.1
motionTracking: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_rgbd3.so.3.3.1
motionTracking: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_saliency3.so.3.3.1
motionTracking: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_stereo3.so.3.3.1
motionTracking: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_structured_light3.so.3.3.1
motionTracking: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_surface_matching3.so.3.3.1
motionTracking: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_tracking3.so.3.3.1
motionTracking: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xfeatures2d3.so.3.3.1
motionTracking: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ximgproc3.so.3.3.1
motionTracking: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xobjdetect3.so.3.3.1
motionTracking: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xphoto3.so.3.3.1
motionTracking: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_shape3.so.3.3.1
motionTracking: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_photo3.so.3.3.1
motionTracking: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_datasets3.so.3.3.1
motionTracking: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_plot3.so.3.3.1
motionTracking: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_text3.so.3.3.1
motionTracking: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_dnn3.so.3.3.1
motionTracking: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ml3.so.3.3.1
motionTracking: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_video3.so.3.3.1
motionTracking: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_calib3d3.so.3.3.1
motionTracking: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_features2d3.so.3.3.1
motionTracking: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_highgui3.so.3.3.1
motionTracking: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_videoio3.so.3.3.1
motionTracking: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_viz3.so.3.3.1
motionTracking: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_phase_unwrapping3.so.3.3.1
motionTracking: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_flann3.so.3.3.1
motionTracking: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgcodecs3.so.3.3.1
motionTracking: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_objdetect3.so.3.3.1
motionTracking: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgproc3.so.3.3.1
motionTracking: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_core3.so.3.3.1
motionTracking: CMakeFiles/motionTracking.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/alantavares/Filters-VO/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable motionTracking"
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
	cd /home/alantavares/Filters-VO/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/alantavares/Filters-VO /home/alantavares/Filters-VO /home/alantavares/Filters-VO/build /home/alantavares/Filters-VO/build /home/alantavares/Filters-VO/build/CMakeFiles/motionTracking.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/motionTracking.dir/depend

