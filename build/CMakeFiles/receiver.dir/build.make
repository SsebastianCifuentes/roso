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
include CMakeFiles/receiver.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/receiver.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/receiver.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/receiver.dir/flags.make

detection.pb.h: /home/robosoccer/roso/src/detection.proto
detection.pb.h: /usr/bin/protoc
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --blue --bold --progress-dir=/home/robosoccer/roso/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Running cpp protocol buffer compiler on src/detection.proto"
	/usr/bin/protoc --cpp_out :/home/robosoccer/roso/build -I /home/robosoccer/roso/src /home/robosoccer/roso/src/detection.proto

detection.pb.cc: detection.pb.h
	@$(CMAKE_COMMAND) -E touch_nocreate detection.pb.cc

CMakeFiles/receiver.dir/src/receiver.cpp.o: CMakeFiles/receiver.dir/flags.make
CMakeFiles/receiver.dir/src/receiver.cpp.o: /home/robosoccer/roso/src/receiver.cpp
CMakeFiles/receiver.dir/src/receiver.cpp.o: CMakeFiles/receiver.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/robosoccer/roso/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/receiver.dir/src/receiver.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/receiver.dir/src/receiver.cpp.o -MF CMakeFiles/receiver.dir/src/receiver.cpp.o.d -o CMakeFiles/receiver.dir/src/receiver.cpp.o -c /home/robosoccer/roso/src/receiver.cpp

CMakeFiles/receiver.dir/src/receiver.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/receiver.dir/src/receiver.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/robosoccer/roso/src/receiver.cpp > CMakeFiles/receiver.dir/src/receiver.cpp.i

CMakeFiles/receiver.dir/src/receiver.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/receiver.dir/src/receiver.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/robosoccer/roso/src/receiver.cpp -o CMakeFiles/receiver.dir/src/receiver.cpp.s

CMakeFiles/receiver.dir/detection.pb.cc.o: CMakeFiles/receiver.dir/flags.make
CMakeFiles/receiver.dir/detection.pb.cc.o: detection.pb.cc
CMakeFiles/receiver.dir/detection.pb.cc.o: CMakeFiles/receiver.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/robosoccer/roso/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/receiver.dir/detection.pb.cc.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/receiver.dir/detection.pb.cc.o -MF CMakeFiles/receiver.dir/detection.pb.cc.o.d -o CMakeFiles/receiver.dir/detection.pb.cc.o -c /home/robosoccer/roso/build/detection.pb.cc

CMakeFiles/receiver.dir/detection.pb.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/receiver.dir/detection.pb.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/robosoccer/roso/build/detection.pb.cc > CMakeFiles/receiver.dir/detection.pb.cc.i

CMakeFiles/receiver.dir/detection.pb.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/receiver.dir/detection.pb.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/robosoccer/roso/build/detection.pb.cc -o CMakeFiles/receiver.dir/detection.pb.cc.s

# Object files for target receiver
receiver_OBJECTS = \
"CMakeFiles/receiver.dir/src/receiver.cpp.o" \
"CMakeFiles/receiver.dir/detection.pb.cc.o"

# External object files for target receiver
receiver_EXTERNAL_OBJECTS =

receiver: CMakeFiles/receiver.dir/src/receiver.cpp.o
receiver: CMakeFiles/receiver.dir/detection.pb.cc.o
receiver: CMakeFiles/receiver.dir/build.make
receiver: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.83.0
receiver: /usr/lib/x86_64-linux-gnu/libprotobuf.so
receiver: /usr/lib/gcc/x86_64-linux-gnu/13/libgomp.so
receiver: /usr/lib/x86_64-linux-gnu/libpthread.a
receiver: CMakeFiles/receiver.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/robosoccer/roso/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable receiver"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/receiver.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/receiver.dir/build: receiver
.PHONY : CMakeFiles/receiver.dir/build

CMakeFiles/receiver.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/receiver.dir/cmake_clean.cmake
.PHONY : CMakeFiles/receiver.dir/clean

CMakeFiles/receiver.dir/depend: detection.pb.cc
CMakeFiles/receiver.dir/depend: detection.pb.h
	cd /home/robosoccer/roso/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/robosoccer/roso /home/robosoccer/roso /home/robosoccer/roso/build /home/robosoccer/roso/build /home/robosoccer/roso/build/CMakeFiles/receiver.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/receiver.dir/depend

