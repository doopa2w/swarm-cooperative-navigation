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
include controllers/footbot_dg/CMakeFiles/footbot_dg.dir/depend.make

# Include the progress variables for this target.
include controllers/footbot_dg/CMakeFiles/footbot_dg.dir/progress.make

# Include the compile flags for this target's objects.
include controllers/footbot_dg/CMakeFiles/footbot_dg.dir/flags.make

controllers/footbot_dg/CMakeFiles/footbot_dg.dir/footbot_dg.cpp.o: controllers/footbot_dg/CMakeFiles/footbot_dg.dir/flags.make
controllers/footbot_dg/CMakeFiles/footbot_dg.dir/footbot_dg.cpp.o: ../controllers/footbot_dg/footbot_dg.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/dev/swarm-cooperative-navigation/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object controllers/footbot_dg/CMakeFiles/footbot_dg.dir/footbot_dg.cpp.o"
	cd /home/dev/swarm-cooperative-navigation/build/controllers/footbot_dg && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/footbot_dg.dir/footbot_dg.cpp.o -c /home/dev/swarm-cooperative-navigation/controllers/footbot_dg/footbot_dg.cpp

controllers/footbot_dg/CMakeFiles/footbot_dg.dir/footbot_dg.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/footbot_dg.dir/footbot_dg.cpp.i"
	cd /home/dev/swarm-cooperative-navigation/build/controllers/footbot_dg && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/dev/swarm-cooperative-navigation/controllers/footbot_dg/footbot_dg.cpp > CMakeFiles/footbot_dg.dir/footbot_dg.cpp.i

controllers/footbot_dg/CMakeFiles/footbot_dg.dir/footbot_dg.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/footbot_dg.dir/footbot_dg.cpp.s"
	cd /home/dev/swarm-cooperative-navigation/build/controllers/footbot_dg && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/dev/swarm-cooperative-navigation/controllers/footbot_dg/footbot_dg.cpp -o CMakeFiles/footbot_dg.dir/footbot_dg.cpp.s

controllers/footbot_dg/CMakeFiles/footbot_dg.dir/footbot_dg.cpp.o.requires:

.PHONY : controllers/footbot_dg/CMakeFiles/footbot_dg.dir/footbot_dg.cpp.o.requires

controllers/footbot_dg/CMakeFiles/footbot_dg.dir/footbot_dg.cpp.o.provides: controllers/footbot_dg/CMakeFiles/footbot_dg.dir/footbot_dg.cpp.o.requires
	$(MAKE) -f controllers/footbot_dg/CMakeFiles/footbot_dg.dir/build.make controllers/footbot_dg/CMakeFiles/footbot_dg.dir/footbot_dg.cpp.o.provides.build
.PHONY : controllers/footbot_dg/CMakeFiles/footbot_dg.dir/footbot_dg.cpp.o.provides

controllers/footbot_dg/CMakeFiles/footbot_dg.dir/footbot_dg.cpp.o.provides.build: controllers/footbot_dg/CMakeFiles/footbot_dg.dir/footbot_dg.cpp.o


controllers/footbot_dg/CMakeFiles/footbot_dg.dir/footbot_dg_autogen/mocs_compilation.cpp.o: controllers/footbot_dg/CMakeFiles/footbot_dg.dir/flags.make
controllers/footbot_dg/CMakeFiles/footbot_dg.dir/footbot_dg_autogen/mocs_compilation.cpp.o: controllers/footbot_dg/footbot_dg_autogen/mocs_compilation.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/dev/swarm-cooperative-navigation/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object controllers/footbot_dg/CMakeFiles/footbot_dg.dir/footbot_dg_autogen/mocs_compilation.cpp.o"
	cd /home/dev/swarm-cooperative-navigation/build/controllers/footbot_dg && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/footbot_dg.dir/footbot_dg_autogen/mocs_compilation.cpp.o -c /home/dev/swarm-cooperative-navigation/build/controllers/footbot_dg/footbot_dg_autogen/mocs_compilation.cpp

controllers/footbot_dg/CMakeFiles/footbot_dg.dir/footbot_dg_autogen/mocs_compilation.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/footbot_dg.dir/footbot_dg_autogen/mocs_compilation.cpp.i"
	cd /home/dev/swarm-cooperative-navigation/build/controllers/footbot_dg && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/dev/swarm-cooperative-navigation/build/controllers/footbot_dg/footbot_dg_autogen/mocs_compilation.cpp > CMakeFiles/footbot_dg.dir/footbot_dg_autogen/mocs_compilation.cpp.i

controllers/footbot_dg/CMakeFiles/footbot_dg.dir/footbot_dg_autogen/mocs_compilation.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/footbot_dg.dir/footbot_dg_autogen/mocs_compilation.cpp.s"
	cd /home/dev/swarm-cooperative-navigation/build/controllers/footbot_dg && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/dev/swarm-cooperative-navigation/build/controllers/footbot_dg/footbot_dg_autogen/mocs_compilation.cpp -o CMakeFiles/footbot_dg.dir/footbot_dg_autogen/mocs_compilation.cpp.s

controllers/footbot_dg/CMakeFiles/footbot_dg.dir/footbot_dg_autogen/mocs_compilation.cpp.o.requires:

.PHONY : controllers/footbot_dg/CMakeFiles/footbot_dg.dir/footbot_dg_autogen/mocs_compilation.cpp.o.requires

controllers/footbot_dg/CMakeFiles/footbot_dg.dir/footbot_dg_autogen/mocs_compilation.cpp.o.provides: controllers/footbot_dg/CMakeFiles/footbot_dg.dir/footbot_dg_autogen/mocs_compilation.cpp.o.requires
	$(MAKE) -f controllers/footbot_dg/CMakeFiles/footbot_dg.dir/build.make controllers/footbot_dg/CMakeFiles/footbot_dg.dir/footbot_dg_autogen/mocs_compilation.cpp.o.provides.build
.PHONY : controllers/footbot_dg/CMakeFiles/footbot_dg.dir/footbot_dg_autogen/mocs_compilation.cpp.o.provides

controllers/footbot_dg/CMakeFiles/footbot_dg.dir/footbot_dg_autogen/mocs_compilation.cpp.o.provides.build: controllers/footbot_dg/CMakeFiles/footbot_dg.dir/footbot_dg_autogen/mocs_compilation.cpp.o


# Object files for target footbot_dg
footbot_dg_OBJECTS = \
"CMakeFiles/footbot_dg.dir/footbot_dg.cpp.o" \
"CMakeFiles/footbot_dg.dir/footbot_dg_autogen/mocs_compilation.cpp.o"

# External object files for target footbot_dg
footbot_dg_EXTERNAL_OBJECTS =

controllers/footbot_dg/libfootbot_dg.so: controllers/footbot_dg/CMakeFiles/footbot_dg.dir/footbot_dg.cpp.o
controllers/footbot_dg/libfootbot_dg.so: controllers/footbot_dg/CMakeFiles/footbot_dg.dir/footbot_dg_autogen/mocs_compilation.cpp.o
controllers/footbot_dg/libfootbot_dg.so: controllers/footbot_dg/CMakeFiles/footbot_dg.dir/build.make
controllers/footbot_dg/libfootbot_dg.so: controllers/footbot_dg/CMakeFiles/footbot_dg.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/dev/swarm-cooperative-navigation/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX shared module libfootbot_dg.so"
	cd /home/dev/swarm-cooperative-navigation/build/controllers/footbot_dg && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/footbot_dg.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
controllers/footbot_dg/CMakeFiles/footbot_dg.dir/build: controllers/footbot_dg/libfootbot_dg.so

.PHONY : controllers/footbot_dg/CMakeFiles/footbot_dg.dir/build

controllers/footbot_dg/CMakeFiles/footbot_dg.dir/requires: controllers/footbot_dg/CMakeFiles/footbot_dg.dir/footbot_dg.cpp.o.requires
controllers/footbot_dg/CMakeFiles/footbot_dg.dir/requires: controllers/footbot_dg/CMakeFiles/footbot_dg.dir/footbot_dg_autogen/mocs_compilation.cpp.o.requires

.PHONY : controllers/footbot_dg/CMakeFiles/footbot_dg.dir/requires

controllers/footbot_dg/CMakeFiles/footbot_dg.dir/clean:
	cd /home/dev/swarm-cooperative-navigation/build/controllers/footbot_dg && $(CMAKE_COMMAND) -P CMakeFiles/footbot_dg.dir/cmake_clean.cmake
.PHONY : controllers/footbot_dg/CMakeFiles/footbot_dg.dir/clean

controllers/footbot_dg/CMakeFiles/footbot_dg.dir/depend:
	cd /home/dev/swarm-cooperative-navigation/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/dev/swarm-cooperative-navigation /home/dev/swarm-cooperative-navigation/controllers/footbot_dg /home/dev/swarm-cooperative-navigation/build /home/dev/swarm-cooperative-navigation/build/controllers/footbot_dg /home/dev/swarm-cooperative-navigation/build/controllers/footbot_dg/CMakeFiles/footbot_dg.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : controllers/footbot_dg/CMakeFiles/footbot_dg.dir/depend

