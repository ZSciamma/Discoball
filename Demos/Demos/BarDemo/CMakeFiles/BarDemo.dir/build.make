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

# Include any dependencies generated for this target.
include Demos/BarDemo/CMakeFiles/BarDemo.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include Demos/BarDemo/CMakeFiles/BarDemo.dir/compiler_depend.make

# Include the progress variables for this target.
include Demos/BarDemo/CMakeFiles/BarDemo.dir/progress.make

# Include the compile flags for this target's objects.
include Demos/BarDemo/CMakeFiles/BarDemo.dir/flags.make

Demos/BarDemo/CMakeFiles/BarDemo.dir/main.cpp.o: Demos/BarDemo/CMakeFiles/BarDemo.dir/flags.make
Demos/BarDemo/CMakeFiles/BarDemo.dir/main.cpp.o: BarDemo/main.cpp
Demos/BarDemo/CMakeFiles/BarDemo.dir/main.cpp.o: Demos/BarDemo/CMakeFiles/BarDemo.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/accts/zas8/Documents/temp/PositionBasedDynamics/Demos/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object Demos/BarDemo/CMakeFiles/BarDemo.dir/main.cpp.o"
	cd /home/accts/zas8/Documents/temp/PositionBasedDynamics/Demos/Demos/BarDemo && /usr/lib64/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT Demos/BarDemo/CMakeFiles/BarDemo.dir/main.cpp.o -MF CMakeFiles/BarDemo.dir/main.cpp.o.d -o CMakeFiles/BarDemo.dir/main.cpp.o -c /home/accts/zas8/Documents/temp/PositionBasedDynamics/Demos/BarDemo/main.cpp

Demos/BarDemo/CMakeFiles/BarDemo.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/BarDemo.dir/main.cpp.i"
	cd /home/accts/zas8/Documents/temp/PositionBasedDynamics/Demos/Demos/BarDemo && /usr/lib64/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/accts/zas8/Documents/temp/PositionBasedDynamics/Demos/BarDemo/main.cpp > CMakeFiles/BarDemo.dir/main.cpp.i

Demos/BarDemo/CMakeFiles/BarDemo.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/BarDemo.dir/main.cpp.s"
	cd /home/accts/zas8/Documents/temp/PositionBasedDynamics/Demos/Demos/BarDemo && /usr/lib64/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/accts/zas8/Documents/temp/PositionBasedDynamics/Demos/BarDemo/main.cpp -o CMakeFiles/BarDemo.dir/main.cpp.s

Demos/BarDemo/CMakeFiles/BarDemo.dir/__/Common/TweakBarParameters.cpp.o: Demos/BarDemo/CMakeFiles/BarDemo.dir/flags.make
Demos/BarDemo/CMakeFiles/BarDemo.dir/__/Common/TweakBarParameters.cpp.o: Common/TweakBarParameters.cpp
Demos/BarDemo/CMakeFiles/BarDemo.dir/__/Common/TweakBarParameters.cpp.o: Demos/BarDemo/CMakeFiles/BarDemo.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/accts/zas8/Documents/temp/PositionBasedDynamics/Demos/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object Demos/BarDemo/CMakeFiles/BarDemo.dir/__/Common/TweakBarParameters.cpp.o"
	cd /home/accts/zas8/Documents/temp/PositionBasedDynamics/Demos/Demos/BarDemo && /usr/lib64/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT Demos/BarDemo/CMakeFiles/BarDemo.dir/__/Common/TweakBarParameters.cpp.o -MF CMakeFiles/BarDemo.dir/__/Common/TweakBarParameters.cpp.o.d -o CMakeFiles/BarDemo.dir/__/Common/TweakBarParameters.cpp.o -c /home/accts/zas8/Documents/temp/PositionBasedDynamics/Demos/Common/TweakBarParameters.cpp

Demos/BarDemo/CMakeFiles/BarDemo.dir/__/Common/TweakBarParameters.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/BarDemo.dir/__/Common/TweakBarParameters.cpp.i"
	cd /home/accts/zas8/Documents/temp/PositionBasedDynamics/Demos/Demos/BarDemo && /usr/lib64/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/accts/zas8/Documents/temp/PositionBasedDynamics/Demos/Common/TweakBarParameters.cpp > CMakeFiles/BarDemo.dir/__/Common/TweakBarParameters.cpp.i

Demos/BarDemo/CMakeFiles/BarDemo.dir/__/Common/TweakBarParameters.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/BarDemo.dir/__/Common/TweakBarParameters.cpp.s"
	cd /home/accts/zas8/Documents/temp/PositionBasedDynamics/Demos/Demos/BarDemo && /usr/lib64/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/accts/zas8/Documents/temp/PositionBasedDynamics/Demos/Common/TweakBarParameters.cpp -o CMakeFiles/BarDemo.dir/__/Common/TweakBarParameters.cpp.s

Demos/BarDemo/CMakeFiles/BarDemo.dir/__/Common/DemoBase.cpp.o: Demos/BarDemo/CMakeFiles/BarDemo.dir/flags.make
Demos/BarDemo/CMakeFiles/BarDemo.dir/__/Common/DemoBase.cpp.o: Common/DemoBase.cpp
Demos/BarDemo/CMakeFiles/BarDemo.dir/__/Common/DemoBase.cpp.o: Demos/BarDemo/CMakeFiles/BarDemo.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/accts/zas8/Documents/temp/PositionBasedDynamics/Demos/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object Demos/BarDemo/CMakeFiles/BarDemo.dir/__/Common/DemoBase.cpp.o"
	cd /home/accts/zas8/Documents/temp/PositionBasedDynamics/Demos/Demos/BarDemo && /usr/lib64/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT Demos/BarDemo/CMakeFiles/BarDemo.dir/__/Common/DemoBase.cpp.o -MF CMakeFiles/BarDemo.dir/__/Common/DemoBase.cpp.o.d -o CMakeFiles/BarDemo.dir/__/Common/DemoBase.cpp.o -c /home/accts/zas8/Documents/temp/PositionBasedDynamics/Demos/Common/DemoBase.cpp

Demos/BarDemo/CMakeFiles/BarDemo.dir/__/Common/DemoBase.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/BarDemo.dir/__/Common/DemoBase.cpp.i"
	cd /home/accts/zas8/Documents/temp/PositionBasedDynamics/Demos/Demos/BarDemo && /usr/lib64/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/accts/zas8/Documents/temp/PositionBasedDynamics/Demos/Common/DemoBase.cpp > CMakeFiles/BarDemo.dir/__/Common/DemoBase.cpp.i

Demos/BarDemo/CMakeFiles/BarDemo.dir/__/Common/DemoBase.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/BarDemo.dir/__/Common/DemoBase.cpp.s"
	cd /home/accts/zas8/Documents/temp/PositionBasedDynamics/Demos/Demos/BarDemo && /usr/lib64/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/accts/zas8/Documents/temp/PositionBasedDynamics/Demos/Common/DemoBase.cpp -o CMakeFiles/BarDemo.dir/__/Common/DemoBase.cpp.s

Demos/BarDemo/CMakeFiles/BarDemo.dir/__/__/extern/glfw/deps/glad_gl.c.o: Demos/BarDemo/CMakeFiles/BarDemo.dir/flags.make
Demos/BarDemo/CMakeFiles/BarDemo.dir/__/__/extern/glfw/deps/glad_gl.c.o: ../extern/glfw/deps/glad_gl.c
Demos/BarDemo/CMakeFiles/BarDemo.dir/__/__/extern/glfw/deps/glad_gl.c.o: Demos/BarDemo/CMakeFiles/BarDemo.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/accts/zas8/Documents/temp/PositionBasedDynamics/Demos/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building C object Demos/BarDemo/CMakeFiles/BarDemo.dir/__/__/extern/glfw/deps/glad_gl.c.o"
	cd /home/accts/zas8/Documents/temp/PositionBasedDynamics/Demos/Demos/BarDemo && /usr/lib64/ccache/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT Demos/BarDemo/CMakeFiles/BarDemo.dir/__/__/extern/glfw/deps/glad_gl.c.o -MF CMakeFiles/BarDemo.dir/__/__/extern/glfw/deps/glad_gl.c.o.d -o CMakeFiles/BarDemo.dir/__/__/extern/glfw/deps/glad_gl.c.o -c /home/accts/zas8/Documents/temp/PositionBasedDynamics/extern/glfw/deps/glad_gl.c

Demos/BarDemo/CMakeFiles/BarDemo.dir/__/__/extern/glfw/deps/glad_gl.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/BarDemo.dir/__/__/extern/glfw/deps/glad_gl.c.i"
	cd /home/accts/zas8/Documents/temp/PositionBasedDynamics/Demos/Demos/BarDemo && /usr/lib64/ccache/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/accts/zas8/Documents/temp/PositionBasedDynamics/extern/glfw/deps/glad_gl.c > CMakeFiles/BarDemo.dir/__/__/extern/glfw/deps/glad_gl.c.i

Demos/BarDemo/CMakeFiles/BarDemo.dir/__/__/extern/glfw/deps/glad_gl.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/BarDemo.dir/__/__/extern/glfw/deps/glad_gl.c.s"
	cd /home/accts/zas8/Documents/temp/PositionBasedDynamics/Demos/Demos/BarDemo && /usr/lib64/ccache/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/accts/zas8/Documents/temp/PositionBasedDynamics/extern/glfw/deps/glad_gl.c -o CMakeFiles/BarDemo.dir/__/__/extern/glfw/deps/glad_gl.c.s

Demos/BarDemo/CMakeFiles/BarDemo.dir/__/Visualization/MiniGL.cpp.o: Demos/BarDemo/CMakeFiles/BarDemo.dir/flags.make
Demos/BarDemo/CMakeFiles/BarDemo.dir/__/Visualization/MiniGL.cpp.o: Visualization/MiniGL.cpp
Demos/BarDemo/CMakeFiles/BarDemo.dir/__/Visualization/MiniGL.cpp.o: Demos/BarDemo/CMakeFiles/BarDemo.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/accts/zas8/Documents/temp/PositionBasedDynamics/Demos/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object Demos/BarDemo/CMakeFiles/BarDemo.dir/__/Visualization/MiniGL.cpp.o"
	cd /home/accts/zas8/Documents/temp/PositionBasedDynamics/Demos/Demos/BarDemo && /usr/lib64/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT Demos/BarDemo/CMakeFiles/BarDemo.dir/__/Visualization/MiniGL.cpp.o -MF CMakeFiles/BarDemo.dir/__/Visualization/MiniGL.cpp.o.d -o CMakeFiles/BarDemo.dir/__/Visualization/MiniGL.cpp.o -c /home/accts/zas8/Documents/temp/PositionBasedDynamics/Demos/Visualization/MiniGL.cpp

Demos/BarDemo/CMakeFiles/BarDemo.dir/__/Visualization/MiniGL.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/BarDemo.dir/__/Visualization/MiniGL.cpp.i"
	cd /home/accts/zas8/Documents/temp/PositionBasedDynamics/Demos/Demos/BarDemo && /usr/lib64/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/accts/zas8/Documents/temp/PositionBasedDynamics/Demos/Visualization/MiniGL.cpp > CMakeFiles/BarDemo.dir/__/Visualization/MiniGL.cpp.i

Demos/BarDemo/CMakeFiles/BarDemo.dir/__/Visualization/MiniGL.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/BarDemo.dir/__/Visualization/MiniGL.cpp.s"
	cd /home/accts/zas8/Documents/temp/PositionBasedDynamics/Demos/Demos/BarDemo && /usr/lib64/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/accts/zas8/Documents/temp/PositionBasedDynamics/Demos/Visualization/MiniGL.cpp -o CMakeFiles/BarDemo.dir/__/Visualization/MiniGL.cpp.s

Demos/BarDemo/CMakeFiles/BarDemo.dir/__/Visualization/Shader.cpp.o: Demos/BarDemo/CMakeFiles/BarDemo.dir/flags.make
Demos/BarDemo/CMakeFiles/BarDemo.dir/__/Visualization/Shader.cpp.o: Visualization/Shader.cpp
Demos/BarDemo/CMakeFiles/BarDemo.dir/__/Visualization/Shader.cpp.o: Demos/BarDemo/CMakeFiles/BarDemo.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/accts/zas8/Documents/temp/PositionBasedDynamics/Demos/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object Demos/BarDemo/CMakeFiles/BarDemo.dir/__/Visualization/Shader.cpp.o"
	cd /home/accts/zas8/Documents/temp/PositionBasedDynamics/Demos/Demos/BarDemo && /usr/lib64/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT Demos/BarDemo/CMakeFiles/BarDemo.dir/__/Visualization/Shader.cpp.o -MF CMakeFiles/BarDemo.dir/__/Visualization/Shader.cpp.o.d -o CMakeFiles/BarDemo.dir/__/Visualization/Shader.cpp.o -c /home/accts/zas8/Documents/temp/PositionBasedDynamics/Demos/Visualization/Shader.cpp

Demos/BarDemo/CMakeFiles/BarDemo.dir/__/Visualization/Shader.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/BarDemo.dir/__/Visualization/Shader.cpp.i"
	cd /home/accts/zas8/Documents/temp/PositionBasedDynamics/Demos/Demos/BarDemo && /usr/lib64/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/accts/zas8/Documents/temp/PositionBasedDynamics/Demos/Visualization/Shader.cpp > CMakeFiles/BarDemo.dir/__/Visualization/Shader.cpp.i

Demos/BarDemo/CMakeFiles/BarDemo.dir/__/Visualization/Shader.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/BarDemo.dir/__/Visualization/Shader.cpp.s"
	cd /home/accts/zas8/Documents/temp/PositionBasedDynamics/Demos/Demos/BarDemo && /usr/lib64/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/accts/zas8/Documents/temp/PositionBasedDynamics/Demos/Visualization/Shader.cpp -o CMakeFiles/BarDemo.dir/__/Visualization/Shader.cpp.s

# Object files for target BarDemo
BarDemo_OBJECTS = \
"CMakeFiles/BarDemo.dir/main.cpp.o" \
"CMakeFiles/BarDemo.dir/__/Common/TweakBarParameters.cpp.o" \
"CMakeFiles/BarDemo.dir/__/Common/DemoBase.cpp.o" \
"CMakeFiles/BarDemo.dir/__/__/extern/glfw/deps/glad_gl.c.o" \
"CMakeFiles/BarDemo.dir/__/Visualization/MiniGL.cpp.o" \
"CMakeFiles/BarDemo.dir/__/Visualization/Shader.cpp.o"

# External object files for target BarDemo
BarDemo_EXTERNAL_OBJECTS =

../bin/BarDemo: Demos/BarDemo/CMakeFiles/BarDemo.dir/main.cpp.o
../bin/BarDemo: Demos/BarDemo/CMakeFiles/BarDemo.dir/__/Common/TweakBarParameters.cpp.o
../bin/BarDemo: Demos/BarDemo/CMakeFiles/BarDemo.dir/__/Common/DemoBase.cpp.o
../bin/BarDemo: Demos/BarDemo/CMakeFiles/BarDemo.dir/__/__/extern/glfw/deps/glad_gl.c.o
../bin/BarDemo: Demos/BarDemo/CMakeFiles/BarDemo.dir/__/Visualization/MiniGL.cpp.o
../bin/BarDemo: Demos/BarDemo/CMakeFiles/BarDemo.dir/__/Visualization/Shader.cpp.o
../bin/BarDemo: Demos/BarDemo/CMakeFiles/BarDemo.dir/build.make
../bin/BarDemo: ../lib/libAntTweakBar.a
../bin/BarDemo: ../lib/libglfw3.a
../bin/BarDemo: ../lib/libPositionBasedDynamics.a
../bin/BarDemo: ../lib/libSimulation.a
../bin/BarDemo: ../lib/libUtils.a
../bin/BarDemo: /usr/lib64/libOpenGL.so
../bin/BarDemo: /usr/lib64/libGLX.so
../bin/BarDemo: /usr/lib64/libGLU.so
../bin/BarDemo: /usr/lib64/librt.a
../bin/BarDemo: /usr/lib64/libm.so
../bin/BarDemo: /usr/lib64/libX11.so
../bin/BarDemo: ../lib/libPositionBasedDynamics.a
../bin/BarDemo: Demos/BarDemo/CMakeFiles/BarDemo.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/accts/zas8/Documents/temp/PositionBasedDynamics/Demos/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Linking CXX executable ../../../bin/BarDemo"
	cd /home/accts/zas8/Documents/temp/PositionBasedDynamics/Demos/Demos/BarDemo && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/BarDemo.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
Demos/BarDemo/CMakeFiles/BarDemo.dir/build: ../bin/BarDemo
.PHONY : Demos/BarDemo/CMakeFiles/BarDemo.dir/build

Demos/BarDemo/CMakeFiles/BarDemo.dir/clean:
	cd /home/accts/zas8/Documents/temp/PositionBasedDynamics/Demos/Demos/BarDemo && $(CMAKE_COMMAND) -P CMakeFiles/BarDemo.dir/cmake_clean.cmake
.PHONY : Demos/BarDemo/CMakeFiles/BarDemo.dir/clean

Demos/BarDemo/CMakeFiles/BarDemo.dir/depend:
	cd /home/accts/zas8/Documents/temp/PositionBasedDynamics/Demos && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/accts/zas8/Documents/temp/PositionBasedDynamics /home/accts/zas8/Documents/temp/PositionBasedDynamics/Demos/BarDemo /home/accts/zas8/Documents/temp/PositionBasedDynamics/Demos /home/accts/zas8/Documents/temp/PositionBasedDynamics/Demos/Demos/BarDemo /home/accts/zas8/Documents/temp/PositionBasedDynamics/Demos/Demos/BarDemo/CMakeFiles/BarDemo.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : Demos/BarDemo/CMakeFiles/BarDemo.dir/depend

