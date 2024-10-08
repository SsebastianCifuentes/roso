# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.28

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
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
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/robosoccer/roso

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/robosoccer/roso/build

# Include any dependencies generated for this target.
include CMakeFiles/calibration.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/calibration.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/calibration.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/calibration.dir/flags.make

CMakeFiles/calibration.dir/src/calibration.cpp.o: CMakeFiles/calibration.dir/flags.make
CMakeFiles/calibration.dir/src/calibration.cpp.o: /home/robosoccer/roso/src/calibration.cpp
CMakeFiles/calibration.dir/src/calibration.cpp.o: CMakeFiles/calibration.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/robosoccer/roso/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/calibration.dir/src/calibration.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/calibration.dir/src/calibration.cpp.o -MF CMakeFiles/calibration.dir/src/calibration.cpp.o.d -o CMakeFiles/calibration.dir/src/calibration.cpp.o -c /home/robosoccer/roso/src/calibration.cpp

CMakeFiles/calibration.dir/src/calibration.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/calibration.dir/src/calibration.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/robosoccer/roso/src/calibration.cpp > CMakeFiles/calibration.dir/src/calibration.cpp.i

CMakeFiles/calibration.dir/src/calibration.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/calibration.dir/src/calibration.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/robosoccer/roso/src/calibration.cpp -o CMakeFiles/calibration.dir/src/calibration.cpp.s

# Object files for target calibration
calibration_OBJECTS = \
"CMakeFiles/calibration.dir/src/calibration.cpp.o"

# External object files for target calibration
calibration_EXTERNAL_OBJECTS =

calibration: CMakeFiles/calibration.dir/src/calibration.cpp.o
calibration: CMakeFiles/calibration.dir/build.make
calibration: /usr/local/lib/libopencv_gapi.so.4.10.0
calibration: /usr/local/lib/libopencv_stitching.so.4.10.0
calibration: /usr/local/lib/libopencv_alphamat.so.4.10.0
calibration: /usr/local/lib/libopencv_aruco.so.4.10.0
calibration: /usr/local/lib/libopencv_bgsegm.so.4.10.0
calibration: /usr/local/lib/libopencv_bioinspired.so.4.10.0
calibration: /usr/local/lib/libopencv_ccalib.so.4.10.0
calibration: /usr/local/lib/libopencv_cvv.so.4.10.0
calibration: /usr/local/lib/libopencv_dnn_objdetect.so.4.10.0
calibration: /usr/local/lib/libopencv_dnn_superres.so.4.10.0
calibration: /usr/local/lib/libopencv_dpm.so.4.10.0
calibration: /usr/local/lib/libopencv_face.so.4.10.0
calibration: /usr/local/lib/libopencv_freetype.so.4.10.0
calibration: /usr/local/lib/libopencv_fuzzy.so.4.10.0
calibration: /usr/local/lib/libopencv_hdf.so.4.10.0
calibration: /usr/local/lib/libopencv_hfs.so.4.10.0
calibration: /usr/local/lib/libopencv_img_hash.so.4.10.0
calibration: /usr/local/lib/libopencv_intensity_transform.so.4.10.0
calibration: /usr/local/lib/libopencv_line_descriptor.so.4.10.0
calibration: /usr/local/lib/libopencv_mcc.so.4.10.0
calibration: /usr/local/lib/libopencv_quality.so.4.10.0
calibration: /usr/local/lib/libopencv_rapid.so.4.10.0
calibration: /usr/local/lib/libopencv_reg.so.4.10.0
calibration: /usr/local/lib/libopencv_rgbd.so.4.10.0
calibration: /usr/local/lib/libopencv_saliency.so.4.10.0
calibration: /usr/local/lib/libopencv_signal.so.4.10.0
calibration: /usr/local/lib/libopencv_stereo.so.4.10.0
calibration: /usr/local/lib/libopencv_structured_light.so.4.10.0
calibration: /usr/local/lib/libopencv_superres.so.4.10.0
calibration: /usr/local/lib/libopencv_surface_matching.so.4.10.0
calibration: /usr/local/lib/libopencv_tracking.so.4.10.0
calibration: /usr/local/lib/libopencv_videostab.so.4.10.0
calibration: /usr/local/lib/libopencv_wechat_qrcode.so.4.10.0
calibration: /usr/local/lib/libopencv_xfeatures2d.so.4.10.0
calibration: /usr/local/lib/libopencv_xobjdetect.so.4.10.0
calibration: /usr/local/lib/libopencv_xphoto.so.4.10.0
calibration: /usr/local/lib/libopencv_shape.so.4.10.0
calibration: /usr/local/lib/libopencv_highgui.so.4.10.0
calibration: /usr/local/lib/libopencv_datasets.so.4.10.0
calibration: /usr/local/lib/libopencv_plot.so.4.10.0
calibration: /usr/local/lib/libopencv_text.so.4.10.0
calibration: /usr/local/lib/libopencv_ml.so.4.10.0
calibration: /usr/local/lib/libopencv_phase_unwrapping.so.4.10.0
calibration: /usr/local/lib/libopencv_optflow.so.4.10.0
calibration: /usr/local/lib/libopencv_ximgproc.so.4.10.0
calibration: /usr/local/lib/libopencv_video.so.4.10.0
calibration: /usr/local/lib/libopencv_videoio.so.4.10.0
calibration: /usr/local/lib/libopencv_imgcodecs.so.4.10.0
calibration: /usr/local/lib/libopencv_objdetect.so.4.10.0
calibration: /usr/local/lib/libopencv_calib3d.so.4.10.0
calibration: /usr/local/lib/libopencv_dnn.so.4.10.0
calibration: /usr/local/lib/libopencv_features2d.so.4.10.0
calibration: /usr/local/lib/libopencv_flann.so.4.10.0
calibration: /usr/local/lib/libopencv_photo.so.4.10.0
calibration: /usr/local/lib/libopencv_imgproc.so.4.10.0
calibration: /usr/local/lib/libopencv_core.so.4.10.0
calibration: /usr/lib/gcc/x86_64-linux-gnu/13/libgomp.so
calibration: /usr/lib/x86_64-linux-gnu/libpthread.a
calibration: CMakeFiles/calibration.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/robosoccer/roso/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable calibration"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/calibration.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/calibration.dir/build: calibration
.PHONY : CMakeFiles/calibration.dir/build

CMakeFiles/calibration.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/calibration.dir/cmake_clean.cmake
.PHONY : CMakeFiles/calibration.dir/clean

CMakeFiles/calibration.dir/depend:
	cd /home/robosoccer/roso/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/robosoccer/roso /home/robosoccer/roso /home/robosoccer/roso/build /home/robosoccer/roso/build /home/robosoccer/roso/build/CMakeFiles/calibration.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/calibration.dir/depend
