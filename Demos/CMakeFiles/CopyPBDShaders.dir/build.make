# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

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
CMAKE_SOURCE_DIR = /home/accts/zas8/Documents/temp/PositionBasedDynamics

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/accts/zas8/Documents/temp/PositionBasedDynamics/Demos

# Utility rule file for CopyPBDShaders.

# Include any custom commands dependencies for this target.
include CMakeFiles/CopyPBDShaders.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/CopyPBDShaders.dir/progress.make

CMakeFiles/CopyPBDShaders:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/accts/zas8/Documents/temp/PositionBasedDynamics/Demos/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Copying PBD shaders"
	/usr/bin/cmake -E copy_directory /home/accts/zas8/Documents/temp/PositionBasedDynamics/data/shaders /home/accts/zas8/Documents/temp/PositionBasedDynamics/bin/resources/shaders

CopyPBDShaders: CMakeFiles/CopyPBDShaders
CopyPBDShaders: CMakeFiles/CopyPBDShaders.dir/build.make
.PHONY : CopyPBDShaders

# Rule to build all files generated by this target.
CMakeFiles/CopyPBDShaders.dir/build: CopyPBDShaders
.PHONY : CMakeFiles/CopyPBDShaders.dir/build

CMakeFiles/CopyPBDShaders.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/CopyPBDShaders.dir/cmake_clean.cmake
.PHONY : CMakeFiles/CopyPBDShaders.dir/clean

CMakeFiles/CopyPBDShaders.dir/depend:
	cd /home/accts/zas8/Documents/temp/PositionBasedDynamics/Demos && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/accts/zas8/Documents/temp/PositionBasedDynamics /home/accts/zas8/Documents/temp/PositionBasedDynamics /home/accts/zas8/Documents/temp/PositionBasedDynamics/Demos /home/accts/zas8/Documents/temp/PositionBasedDynamics/Demos /home/accts/zas8/Documents/temp/PositionBasedDynamics/Demos/CMakeFiles/CopyPBDShaders.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/CopyPBDShaders.dir/depend

