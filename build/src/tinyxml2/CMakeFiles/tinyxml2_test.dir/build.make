# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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
CMAKE_SOURCE_DIR = /home/z/work/ecat_master

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/z/work/ecat_master/build

# Include any dependencies generated for this target.
include src/tinyxml2/CMakeFiles/tinyxml2_test.dir/depend.make

# Include the progress variables for this target.
include src/tinyxml2/CMakeFiles/tinyxml2_test.dir/progress.make

# Include the compile flags for this target's objects.
include src/tinyxml2/CMakeFiles/tinyxml2_test.dir/flags.make

src/tinyxml2/CMakeFiles/tinyxml2_test.dir/tinyxml_test.cpp.o: src/tinyxml2/CMakeFiles/tinyxml2_test.dir/flags.make
src/tinyxml2/CMakeFiles/tinyxml2_test.dir/tinyxml_test.cpp.o: ../src/tinyxml2/tinyxml_test.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/z/work/ecat_master/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/tinyxml2/CMakeFiles/tinyxml2_test.dir/tinyxml_test.cpp.o"
	cd /home/z/work/ecat_master/build/src/tinyxml2 && /usr/bin/g++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/tinyxml2_test.dir/tinyxml_test.cpp.o -c /home/z/work/ecat_master/src/tinyxml2/tinyxml_test.cpp

src/tinyxml2/CMakeFiles/tinyxml2_test.dir/tinyxml_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tinyxml2_test.dir/tinyxml_test.cpp.i"
	cd /home/z/work/ecat_master/build/src/tinyxml2 && /usr/bin/g++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/z/work/ecat_master/src/tinyxml2/tinyxml_test.cpp > CMakeFiles/tinyxml2_test.dir/tinyxml_test.cpp.i

src/tinyxml2/CMakeFiles/tinyxml2_test.dir/tinyxml_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tinyxml2_test.dir/tinyxml_test.cpp.s"
	cd /home/z/work/ecat_master/build/src/tinyxml2 && /usr/bin/g++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/z/work/ecat_master/src/tinyxml2/tinyxml_test.cpp -o CMakeFiles/tinyxml2_test.dir/tinyxml_test.cpp.s

src/tinyxml2/CMakeFiles/tinyxml2_test.dir/tinyxml_test.cpp.o.requires:
.PHONY : src/tinyxml2/CMakeFiles/tinyxml2_test.dir/tinyxml_test.cpp.o.requires

src/tinyxml2/CMakeFiles/tinyxml2_test.dir/tinyxml_test.cpp.o.provides: src/tinyxml2/CMakeFiles/tinyxml2_test.dir/tinyxml_test.cpp.o.requires
	$(MAKE) -f src/tinyxml2/CMakeFiles/tinyxml2_test.dir/build.make src/tinyxml2/CMakeFiles/tinyxml2_test.dir/tinyxml_test.cpp.o.provides.build
.PHONY : src/tinyxml2/CMakeFiles/tinyxml2_test.dir/tinyxml_test.cpp.o.provides

src/tinyxml2/CMakeFiles/tinyxml2_test.dir/tinyxml_test.cpp.o.provides.build: src/tinyxml2/CMakeFiles/tinyxml2_test.dir/tinyxml_test.cpp.o

src/tinyxml2/CMakeFiles/tinyxml2_test.dir/tinyxml2.cpp.o: src/tinyxml2/CMakeFiles/tinyxml2_test.dir/flags.make
src/tinyxml2/CMakeFiles/tinyxml2_test.dir/tinyxml2.cpp.o: ../src/tinyxml2/tinyxml2.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/z/work/ecat_master/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/tinyxml2/CMakeFiles/tinyxml2_test.dir/tinyxml2.cpp.o"
	cd /home/z/work/ecat_master/build/src/tinyxml2 && /usr/bin/g++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/tinyxml2_test.dir/tinyxml2.cpp.o -c /home/z/work/ecat_master/src/tinyxml2/tinyxml2.cpp

src/tinyxml2/CMakeFiles/tinyxml2_test.dir/tinyxml2.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tinyxml2_test.dir/tinyxml2.cpp.i"
	cd /home/z/work/ecat_master/build/src/tinyxml2 && /usr/bin/g++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/z/work/ecat_master/src/tinyxml2/tinyxml2.cpp > CMakeFiles/tinyxml2_test.dir/tinyxml2.cpp.i

src/tinyxml2/CMakeFiles/tinyxml2_test.dir/tinyxml2.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tinyxml2_test.dir/tinyxml2.cpp.s"
	cd /home/z/work/ecat_master/build/src/tinyxml2 && /usr/bin/g++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/z/work/ecat_master/src/tinyxml2/tinyxml2.cpp -o CMakeFiles/tinyxml2_test.dir/tinyxml2.cpp.s

src/tinyxml2/CMakeFiles/tinyxml2_test.dir/tinyxml2.cpp.o.requires:
.PHONY : src/tinyxml2/CMakeFiles/tinyxml2_test.dir/tinyxml2.cpp.o.requires

src/tinyxml2/CMakeFiles/tinyxml2_test.dir/tinyxml2.cpp.o.provides: src/tinyxml2/CMakeFiles/tinyxml2_test.dir/tinyxml2.cpp.o.requires
	$(MAKE) -f src/tinyxml2/CMakeFiles/tinyxml2_test.dir/build.make src/tinyxml2/CMakeFiles/tinyxml2_test.dir/tinyxml2.cpp.o.provides.build
.PHONY : src/tinyxml2/CMakeFiles/tinyxml2_test.dir/tinyxml2.cpp.o.provides

src/tinyxml2/CMakeFiles/tinyxml2_test.dir/tinyxml2.cpp.o.provides.build: src/tinyxml2/CMakeFiles/tinyxml2_test.dir/tinyxml2.cpp.o

# Object files for target tinyxml2_test
tinyxml2_test_OBJECTS = \
"CMakeFiles/tinyxml2_test.dir/tinyxml_test.cpp.o" \
"CMakeFiles/tinyxml2_test.dir/tinyxml2.cpp.o"

# External object files for target tinyxml2_test
tinyxml2_test_EXTERNAL_OBJECTS =

bin/tinyxml2_test: src/tinyxml2/CMakeFiles/tinyxml2_test.dir/tinyxml_test.cpp.o
bin/tinyxml2_test: src/tinyxml2/CMakeFiles/tinyxml2_test.dir/tinyxml2.cpp.o
bin/tinyxml2_test: src/tinyxml2/CMakeFiles/tinyxml2_test.dir/build.make
bin/tinyxml2_test: src/tinyxml2/CMakeFiles/tinyxml2_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../../bin/tinyxml2_test"
	cd /home/z/work/ecat_master/build/src/tinyxml2 && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/tinyxml2_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/tinyxml2/CMakeFiles/tinyxml2_test.dir/build: bin/tinyxml2_test
.PHONY : src/tinyxml2/CMakeFiles/tinyxml2_test.dir/build

src/tinyxml2/CMakeFiles/tinyxml2_test.dir/requires: src/tinyxml2/CMakeFiles/tinyxml2_test.dir/tinyxml_test.cpp.o.requires
src/tinyxml2/CMakeFiles/tinyxml2_test.dir/requires: src/tinyxml2/CMakeFiles/tinyxml2_test.dir/tinyxml2.cpp.o.requires
.PHONY : src/tinyxml2/CMakeFiles/tinyxml2_test.dir/requires

src/tinyxml2/CMakeFiles/tinyxml2_test.dir/clean:
	cd /home/z/work/ecat_master/build/src/tinyxml2 && $(CMAKE_COMMAND) -P CMakeFiles/tinyxml2_test.dir/cmake_clean.cmake
.PHONY : src/tinyxml2/CMakeFiles/tinyxml2_test.dir/clean

src/tinyxml2/CMakeFiles/tinyxml2_test.dir/depend:
	cd /home/z/work/ecat_master/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/z/work/ecat_master /home/z/work/ecat_master/src/tinyxml2 /home/z/work/ecat_master/build /home/z/work/ecat_master/build/src/tinyxml2 /home/z/work/ecat_master/build/src/tinyxml2/CMakeFiles/tinyxml2_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/tinyxml2/CMakeFiles/tinyxml2_test.dir/depend

