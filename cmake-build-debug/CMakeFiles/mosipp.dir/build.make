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
CMAKE_COMMAND = /snap/clion/284/bin/cmake/linux/x64/bin/cmake

# The command to remove a file.
RM = /snap/clion/284/bin/cmake/linux/x64/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/david/文档/GitHub/Mosipp

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/david/文档/GitHub/Mosipp/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/mosipp.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/mosipp.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/mosipp.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/mosipp.dir/flags.make

CMakeFiles/mosipp.dir/source/Priority-sipp.cpp.o: CMakeFiles/mosipp.dir/flags.make
CMakeFiles/mosipp.dir/source/Priority-sipp.cpp.o: /home/david/文档/GitHub/Mosipp/source/Priority-sipp.cpp
CMakeFiles/mosipp.dir/source/Priority-sipp.cpp.o: CMakeFiles/mosipp.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/david/文档/GitHub/Mosipp/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/mosipp.dir/source/Priority-sipp.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/mosipp.dir/source/Priority-sipp.cpp.o -MF CMakeFiles/mosipp.dir/source/Priority-sipp.cpp.o.d -o CMakeFiles/mosipp.dir/source/Priority-sipp.cpp.o -c /home/david/文档/GitHub/Mosipp/source/Priority-sipp.cpp

CMakeFiles/mosipp.dir/source/Priority-sipp.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/mosipp.dir/source/Priority-sipp.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/david/文档/GitHub/Mosipp/source/Priority-sipp.cpp > CMakeFiles/mosipp.dir/source/Priority-sipp.cpp.i

CMakeFiles/mosipp.dir/source/Priority-sipp.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/mosipp.dir/source/Priority-sipp.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/david/文档/GitHub/Mosipp/source/Priority-sipp.cpp -o CMakeFiles/mosipp.dir/source/Priority-sipp.cpp.s

CMakeFiles/mosipp.dir/source/avltree.cpp.o: CMakeFiles/mosipp.dir/flags.make
CMakeFiles/mosipp.dir/source/avltree.cpp.o: /home/david/文档/GitHub/Mosipp/source/avltree.cpp
CMakeFiles/mosipp.dir/source/avltree.cpp.o: CMakeFiles/mosipp.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/david/文档/GitHub/Mosipp/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/mosipp.dir/source/avltree.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/mosipp.dir/source/avltree.cpp.o -MF CMakeFiles/mosipp.dir/source/avltree.cpp.o.d -o CMakeFiles/mosipp.dir/source/avltree.cpp.o -c /home/david/文档/GitHub/Mosipp/source/avltree.cpp

CMakeFiles/mosipp.dir/source/avltree.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/mosipp.dir/source/avltree.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/david/文档/GitHub/Mosipp/source/avltree.cpp > CMakeFiles/mosipp.dir/source/avltree.cpp.i

CMakeFiles/mosipp.dir/source/avltree.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/mosipp.dir/source/avltree.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/david/文档/GitHub/Mosipp/source/avltree.cpp -o CMakeFiles/mosipp.dir/source/avltree.cpp.s

CMakeFiles/mosipp.dir/source/dijkstra.cpp.o: CMakeFiles/mosipp.dir/flags.make
CMakeFiles/mosipp.dir/source/dijkstra.cpp.o: /home/david/文档/GitHub/Mosipp/source/dijkstra.cpp
CMakeFiles/mosipp.dir/source/dijkstra.cpp.o: CMakeFiles/mosipp.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/david/文档/GitHub/Mosipp/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/mosipp.dir/source/dijkstra.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/mosipp.dir/source/dijkstra.cpp.o -MF CMakeFiles/mosipp.dir/source/dijkstra.cpp.o.d -o CMakeFiles/mosipp.dir/source/dijkstra.cpp.o -c /home/david/文档/GitHub/Mosipp/source/dijkstra.cpp

CMakeFiles/mosipp.dir/source/dijkstra.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/mosipp.dir/source/dijkstra.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/david/文档/GitHub/Mosipp/source/dijkstra.cpp > CMakeFiles/mosipp.dir/source/dijkstra.cpp.i

CMakeFiles/mosipp.dir/source/dijkstra.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/mosipp.dir/source/dijkstra.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/david/文档/GitHub/Mosipp/source/dijkstra.cpp -o CMakeFiles/mosipp.dir/source/dijkstra.cpp.s

CMakeFiles/mosipp.dir/source/graph.cpp.o: CMakeFiles/mosipp.dir/flags.make
CMakeFiles/mosipp.dir/source/graph.cpp.o: /home/david/文档/GitHub/Mosipp/source/graph.cpp
CMakeFiles/mosipp.dir/source/graph.cpp.o: CMakeFiles/mosipp.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/david/文档/GitHub/Mosipp/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/mosipp.dir/source/graph.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/mosipp.dir/source/graph.cpp.o -MF CMakeFiles/mosipp.dir/source/graph.cpp.o.d -o CMakeFiles/mosipp.dir/source/graph.cpp.o -c /home/david/文档/GitHub/Mosipp/source/graph.cpp

CMakeFiles/mosipp.dir/source/graph.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/mosipp.dir/source/graph.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/david/文档/GitHub/Mosipp/source/graph.cpp > CMakeFiles/mosipp.dir/source/graph.cpp.i

CMakeFiles/mosipp.dir/source/graph.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/mosipp.dir/source/graph.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/david/文档/GitHub/Mosipp/source/graph.cpp -o CMakeFiles/mosipp.dir/source/graph.cpp.s

CMakeFiles/mosipp.dir/source/graph_io.cpp.o: CMakeFiles/mosipp.dir/flags.make
CMakeFiles/mosipp.dir/source/graph_io.cpp.o: /home/david/文档/GitHub/Mosipp/source/graph_io.cpp
CMakeFiles/mosipp.dir/source/graph_io.cpp.o: CMakeFiles/mosipp.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/david/文档/GitHub/Mosipp/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/mosipp.dir/source/graph_io.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/mosipp.dir/source/graph_io.cpp.o -MF CMakeFiles/mosipp.dir/source/graph_io.cpp.o.d -o CMakeFiles/mosipp.dir/source/graph_io.cpp.o -c /home/david/文档/GitHub/Mosipp/source/graph_io.cpp

CMakeFiles/mosipp.dir/source/graph_io.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/mosipp.dir/source/graph_io.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/david/文档/GitHub/Mosipp/source/graph_io.cpp > CMakeFiles/mosipp.dir/source/graph_io.cpp.i

CMakeFiles/mosipp.dir/source/graph_io.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/mosipp.dir/source/graph_io.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/david/文档/GitHub/Mosipp/source/graph_io.cpp -o CMakeFiles/mosipp.dir/source/graph_io.cpp.s

CMakeFiles/mosipp.dir/source/mosipp.cpp.o: CMakeFiles/mosipp.dir/flags.make
CMakeFiles/mosipp.dir/source/mosipp.cpp.o: /home/david/文档/GitHub/Mosipp/source/mosipp.cpp
CMakeFiles/mosipp.dir/source/mosipp.cpp.o: CMakeFiles/mosipp.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/david/文档/GitHub/Mosipp/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/mosipp.dir/source/mosipp.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/mosipp.dir/source/mosipp.cpp.o -MF CMakeFiles/mosipp.dir/source/mosipp.cpp.o.d -o CMakeFiles/mosipp.dir/source/mosipp.cpp.o -c /home/david/文档/GitHub/Mosipp/source/mosipp.cpp

CMakeFiles/mosipp.dir/source/mosipp.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/mosipp.dir/source/mosipp.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/david/文档/GitHub/Mosipp/source/mosipp.cpp > CMakeFiles/mosipp.dir/source/mosipp.cpp.i

CMakeFiles/mosipp.dir/source/mosipp.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/mosipp.dir/source/mosipp.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/david/文档/GitHub/Mosipp/source/mosipp.cpp -o CMakeFiles/mosipp.dir/source/mosipp.cpp.s

CMakeFiles/mosipp.dir/source/mospp_util.cpp.o: CMakeFiles/mosipp.dir/flags.make
CMakeFiles/mosipp.dir/source/mospp_util.cpp.o: /home/david/文档/GitHub/Mosipp/source/mospp_util.cpp
CMakeFiles/mosipp.dir/source/mospp_util.cpp.o: CMakeFiles/mosipp.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/david/文档/GitHub/Mosipp/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/mosipp.dir/source/mospp_util.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/mosipp.dir/source/mospp_util.cpp.o -MF CMakeFiles/mosipp.dir/source/mospp_util.cpp.o.d -o CMakeFiles/mosipp.dir/source/mospp_util.cpp.o -c /home/david/文档/GitHub/Mosipp/source/mospp_util.cpp

CMakeFiles/mosipp.dir/source/mospp_util.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/mosipp.dir/source/mospp_util.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/david/文档/GitHub/Mosipp/source/mospp_util.cpp > CMakeFiles/mosipp.dir/source/mospp_util.cpp.i

CMakeFiles/mosipp.dir/source/mospp_util.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/mosipp.dir/source/mospp_util.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/david/文档/GitHub/Mosipp/source/mospp_util.cpp -o CMakeFiles/mosipp.dir/source/mospp_util.cpp.s

# Object files for target mosipp
mosipp_OBJECTS = \
"CMakeFiles/mosipp.dir/source/Priority-sipp.cpp.o" \
"CMakeFiles/mosipp.dir/source/avltree.cpp.o" \
"CMakeFiles/mosipp.dir/source/dijkstra.cpp.o" \
"CMakeFiles/mosipp.dir/source/graph.cpp.o" \
"CMakeFiles/mosipp.dir/source/graph_io.cpp.o" \
"CMakeFiles/mosipp.dir/source/mosipp.cpp.o" \
"CMakeFiles/mosipp.dir/source/mospp_util.cpp.o"

# External object files for target mosipp
mosipp_EXTERNAL_OBJECTS =

libmosipp.so: CMakeFiles/mosipp.dir/source/Priority-sipp.cpp.o
libmosipp.so: CMakeFiles/mosipp.dir/source/avltree.cpp.o
libmosipp.so: CMakeFiles/mosipp.dir/source/dijkstra.cpp.o
libmosipp.so: CMakeFiles/mosipp.dir/source/graph.cpp.o
libmosipp.so: CMakeFiles/mosipp.dir/source/graph_io.cpp.o
libmosipp.so: CMakeFiles/mosipp.dir/source/mosipp.cpp.o
libmosipp.so: CMakeFiles/mosipp.dir/source/mospp_util.cpp.o
libmosipp.so: CMakeFiles/mosipp.dir/build.make
libmosipp.so: CMakeFiles/mosipp.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/david/文档/GitHub/Mosipp/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Linking CXX shared library libmosipp.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/mosipp.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/mosipp.dir/build: libmosipp.so
.PHONY : CMakeFiles/mosipp.dir/build

CMakeFiles/mosipp.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/mosipp.dir/cmake_clean.cmake
.PHONY : CMakeFiles/mosipp.dir/clean

CMakeFiles/mosipp.dir/depend:
	cd /home/david/文档/GitHub/Mosipp/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/david/文档/GitHub/Mosipp /home/david/文档/GitHub/Mosipp /home/david/文档/GitHub/Mosipp/cmake-build-debug /home/david/文档/GitHub/Mosipp/cmake-build-debug /home/david/文档/GitHub/Mosipp/cmake-build-debug/CMakeFiles/mosipp.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/mosipp.dir/depend

