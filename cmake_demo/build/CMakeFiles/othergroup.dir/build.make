# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.15

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
CMAKE_SOURCE_DIR = /home/yyj/下载/develop_utils/cmake_demo

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/yyj/下载/develop_utils/cmake_demo/build

# Include any dependencies generated for this target.
include CMakeFiles/othergroup.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/othergroup.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/othergroup.dir/flags.make

CMakeFiles/othergroup.dir/src/Other.cpp.o: CMakeFiles/othergroup.dir/flags.make
CMakeFiles/othergroup.dir/src/Other.cpp.o: ../src/Other.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yyj/下载/develop_utils/cmake_demo/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/othergroup.dir/src/Other.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/othergroup.dir/src/Other.cpp.o -c /home/yyj/下载/develop_utils/cmake_demo/src/Other.cpp

CMakeFiles/othergroup.dir/src/Other.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/othergroup.dir/src/Other.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yyj/下载/develop_utils/cmake_demo/src/Other.cpp > CMakeFiles/othergroup.dir/src/Other.cpp.i

CMakeFiles/othergroup.dir/src/Other.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/othergroup.dir/src/Other.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yyj/下载/develop_utils/cmake_demo/src/Other.cpp -o CMakeFiles/othergroup.dir/src/Other.cpp.s

CMakeFiles/othergroup.dir/src/OtherGroup.cpp.o: CMakeFiles/othergroup.dir/flags.make
CMakeFiles/othergroup.dir/src/OtherGroup.cpp.o: ../src/OtherGroup.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yyj/下载/develop_utils/cmake_demo/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/othergroup.dir/src/OtherGroup.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/othergroup.dir/src/OtherGroup.cpp.o -c /home/yyj/下载/develop_utils/cmake_demo/src/OtherGroup.cpp

CMakeFiles/othergroup.dir/src/OtherGroup.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/othergroup.dir/src/OtherGroup.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yyj/下载/develop_utils/cmake_demo/src/OtherGroup.cpp > CMakeFiles/othergroup.dir/src/OtherGroup.cpp.i

CMakeFiles/othergroup.dir/src/OtherGroup.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/othergroup.dir/src/OtherGroup.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yyj/下载/develop_utils/cmake_demo/src/OtherGroup.cpp -o CMakeFiles/othergroup.dir/src/OtherGroup.cpp.s

# Object files for target othergroup
othergroup_OBJECTS = \
"CMakeFiles/othergroup.dir/src/Other.cpp.o" \
"CMakeFiles/othergroup.dir/src/OtherGroup.cpp.o"

# External object files for target othergroup
othergroup_EXTERNAL_OBJECTS =

../lib/libothergroup.so: CMakeFiles/othergroup.dir/src/Other.cpp.o
../lib/libothergroup.so: CMakeFiles/othergroup.dir/src/OtherGroup.cpp.o
../lib/libothergroup.so: CMakeFiles/othergroup.dir/build.make
../lib/libothergroup.so: CMakeFiles/othergroup.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/yyj/下载/develop_utils/cmake_demo/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX shared library ../lib/libothergroup.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/othergroup.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/othergroup.dir/build: ../lib/libothergroup.so

.PHONY : CMakeFiles/othergroup.dir/build

CMakeFiles/othergroup.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/othergroup.dir/cmake_clean.cmake
.PHONY : CMakeFiles/othergroup.dir/clean

CMakeFiles/othergroup.dir/depend:
	cd /home/yyj/下载/develop_utils/cmake_demo/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/yyj/下载/develop_utils/cmake_demo /home/yyj/下载/develop_utils/cmake_demo /home/yyj/下载/develop_utils/cmake_demo/build /home/yyj/下载/develop_utils/cmake_demo/build /home/yyj/下载/develop_utils/cmake_demo/build/CMakeFiles/othergroup.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/othergroup.dir/depend
