# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/dev/swarm-cooperative-navigation

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/dev/swarm-cooperative-navigation/build

# Include any dependencies generated for this target.
include embedding/galib/CMakeFiles/galib_phototaxis.dir/depend.make

# Include the progress variables for this target.
include embedding/galib/CMakeFiles/galib_phototaxis.dir/progress.make

# Include the compile flags for this target's objects.
include embedding/galib/CMakeFiles/galib_phototaxis.dir/flags.make

embedding/galib/CMakeFiles/galib_phototaxis.dir/main.cpp.o: embedding/galib/CMakeFiles/galib_phototaxis.dir/flags.make
embedding/galib/CMakeFiles/galib_phototaxis.dir/main.cpp.o: ../embedding/galib/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/dev/swarm-cooperative-navigation/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object embedding/galib/CMakeFiles/galib_phototaxis.dir/main.cpp.o"
	cd /home/dev/swarm-cooperative-navigation/build/embedding/galib && /usr/bin/g++-7  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/galib_phototaxis.dir/main.cpp.o -c /home/dev/swarm-cooperative-navigation/embedding/galib/main.cpp

embedding/galib/CMakeFiles/galib_phototaxis.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/galib_phototaxis.dir/main.cpp.i"
	cd /home/dev/swarm-cooperative-navigation/build/embedding/galib && /usr/bin/g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/dev/swarm-cooperative-navigation/embedding/galib/main.cpp > CMakeFiles/galib_phototaxis.dir/main.cpp.i

embedding/galib/CMakeFiles/galib_phototaxis.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/galib_phototaxis.dir/main.cpp.s"
	cd /home/dev/swarm-cooperative-navigation/build/embedding/galib && /usr/bin/g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/dev/swarm-cooperative-navigation/embedding/galib/main.cpp -o CMakeFiles/galib_phototaxis.dir/main.cpp.s

embedding/galib/CMakeFiles/galib_phototaxis.dir/main.cpp.o.requires:

.PHONY : embedding/galib/CMakeFiles/galib_phototaxis.dir/main.cpp.o.requires

embedding/galib/CMakeFiles/galib_phototaxis.dir/main.cpp.o.provides: embedding/galib/CMakeFiles/galib_phototaxis.dir/main.cpp.o.requires
	$(MAKE) -f embedding/galib/CMakeFiles/galib_phototaxis.dir/build.make embedding/galib/CMakeFiles/galib_phototaxis.dir/main.cpp.o.provides.build
.PHONY : embedding/galib/CMakeFiles/galib_phototaxis.dir/main.cpp.o.provides

embedding/galib/CMakeFiles/galib_phototaxis.dir/main.cpp.o.provides.build: embedding/galib/CMakeFiles/galib_phototaxis.dir/main.cpp.o


embedding/galib/CMakeFiles/galib_phototaxis.dir/galib_phototaxis_autogen/mocs_compilation.cpp.o: embedding/galib/CMakeFiles/galib_phototaxis.dir/flags.make
embedding/galib/CMakeFiles/galib_phototaxis.dir/galib_phototaxis_autogen/mocs_compilation.cpp.o: embedding/galib/galib_phototaxis_autogen/mocs_compilation.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/dev/swarm-cooperative-navigation/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object embedding/galib/CMakeFiles/galib_phototaxis.dir/galib_phototaxis_autogen/mocs_compilation.cpp.o"
	cd /home/dev/swarm-cooperative-navigation/build/embedding/galib && /usr/bin/g++-7  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/galib_phototaxis.dir/galib_phototaxis_autogen/mocs_compilation.cpp.o -c /home/dev/swarm-cooperative-navigation/build/embedding/galib/galib_phototaxis_autogen/mocs_compilation.cpp

embedding/galib/CMakeFiles/galib_phototaxis.dir/galib_phototaxis_autogen/mocs_compilation.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/galib_phototaxis.dir/galib_phototaxis_autogen/mocs_compilation.cpp.i"
	cd /home/dev/swarm-cooperative-navigation/build/embedding/galib && /usr/bin/g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/dev/swarm-cooperative-navigation/build/embedding/galib/galib_phototaxis_autogen/mocs_compilation.cpp > CMakeFiles/galib_phototaxis.dir/galib_phototaxis_autogen/mocs_compilation.cpp.i

embedding/galib/CMakeFiles/galib_phototaxis.dir/galib_phototaxis_autogen/mocs_compilation.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/galib_phototaxis.dir/galib_phototaxis_autogen/mocs_compilation.cpp.s"
	cd /home/dev/swarm-cooperative-navigation/build/embedding/galib && /usr/bin/g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/dev/swarm-cooperative-navigation/build/embedding/galib/galib_phototaxis_autogen/mocs_compilation.cpp -o CMakeFiles/galib_phototaxis.dir/galib_phototaxis_autogen/mocs_compilation.cpp.s

embedding/galib/CMakeFiles/galib_phototaxis.dir/galib_phototaxis_autogen/mocs_compilation.cpp.o.requires:

.PHONY : embedding/galib/CMakeFiles/galib_phototaxis.dir/galib_phototaxis_autogen/mocs_compilation.cpp.o.requires

embedding/galib/CMakeFiles/galib_phototaxis.dir/galib_phototaxis_autogen/mocs_compilation.cpp.o.provides: embedding/galib/CMakeFiles/galib_phototaxis.dir/galib_phototaxis_autogen/mocs_compilation.cpp.o.requires
	$(MAKE) -f embedding/galib/CMakeFiles/galib_phototaxis.dir/build.make embedding/galib/CMakeFiles/galib_phototaxis.dir/galib_phototaxis_autogen/mocs_compilation.cpp.o.provides.build
.PHONY : embedding/galib/CMakeFiles/galib_phototaxis.dir/galib_phototaxis_autogen/mocs_compilation.cpp.o.provides

embedding/galib/CMakeFiles/galib_phototaxis.dir/galib_phototaxis_autogen/mocs_compilation.cpp.o.provides.build: embedding/galib/CMakeFiles/galib_phototaxis.dir/galib_phototaxis_autogen/mocs_compilation.cpp.o


# Object files for target galib_phototaxis
galib_phototaxis_OBJECTS = \
"CMakeFiles/galib_phototaxis.dir/main.cpp.o" \
"CMakeFiles/galib_phototaxis.dir/galib_phototaxis_autogen/mocs_compilation.cpp.o"

# External object files for target galib_phototaxis
galib_phototaxis_EXTERNAL_OBJECTS =

embedding/galib/galib_phototaxis: embedding/galib/CMakeFiles/galib_phototaxis.dir/main.cpp.o
embedding/galib/galib_phototaxis: embedding/galib/CMakeFiles/galib_phototaxis.dir/galib_phototaxis_autogen/mocs_compilation.cpp.o
embedding/galib/galib_phototaxis: embedding/galib/CMakeFiles/galib_phototaxis.dir/build.make
embedding/galib/galib_phototaxis: /usr/lib/libga.so
embedding/galib/galib_phototaxis: loop_functions/galib_phototaxis_loop_functions/libgalib_phototaxis_loop_functions.so
embedding/galib/galib_phototaxis: controllers/footbot_nn/libfootbot_nn.so
embedding/galib/galib_phototaxis: /usr/lib/libga.so
embedding/galib/galib_phototaxis: embedding/galib/CMakeFiles/galib_phototaxis.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/dev/swarm-cooperative-navigation/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable galib_phototaxis"
	cd /home/dev/swarm-cooperative-navigation/build/embedding/galib && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/galib_phototaxis.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
embedding/galib/CMakeFiles/galib_phototaxis.dir/build: embedding/galib/galib_phototaxis

.PHONY : embedding/galib/CMakeFiles/galib_phototaxis.dir/build

embedding/galib/CMakeFiles/galib_phototaxis.dir/requires: embedding/galib/CMakeFiles/galib_phototaxis.dir/main.cpp.o.requires
embedding/galib/CMakeFiles/galib_phototaxis.dir/requires: embedding/galib/CMakeFiles/galib_phototaxis.dir/galib_phototaxis_autogen/mocs_compilation.cpp.o.requires

.PHONY : embedding/galib/CMakeFiles/galib_phototaxis.dir/requires

embedding/galib/CMakeFiles/galib_phototaxis.dir/clean:
	cd /home/dev/swarm-cooperative-navigation/build/embedding/galib && $(CMAKE_COMMAND) -P CMakeFiles/galib_phototaxis.dir/cmake_clean.cmake
.PHONY : embedding/galib/CMakeFiles/galib_phototaxis.dir/clean

embedding/galib/CMakeFiles/galib_phototaxis.dir/depend:
	cd /home/dev/swarm-cooperative-navigation/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/dev/swarm-cooperative-navigation /home/dev/swarm-cooperative-navigation/embedding/galib /home/dev/swarm-cooperative-navigation/build /home/dev/swarm-cooperative-navigation/build/embedding/galib /home/dev/swarm-cooperative-navigation/build/embedding/galib/CMakeFiles/galib_phototaxis.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : embedding/galib/CMakeFiles/galib_phototaxis.dir/depend

