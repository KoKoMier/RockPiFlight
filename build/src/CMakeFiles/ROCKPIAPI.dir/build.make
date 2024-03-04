# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/cyh/FlightCode/RockPiFlight

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/cyh/FlightCode/RockPiFlight/build

# Include any dependencies generated for this target.
include src/CMakeFiles/ROCKPIAPI.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/ROCKPIAPI.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/ROCKPIAPI.dir/flags.make

src/CMakeFiles/ROCKPIAPI.dir/RockPiAPM.cpp.o: src/CMakeFiles/ROCKPIAPI.dir/flags.make
src/CMakeFiles/ROCKPIAPI.dir/RockPiAPM.cpp.o: ../src/RockPiAPM.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cyh/FlightCode/RockPiFlight/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/CMakeFiles/ROCKPIAPI.dir/RockPiAPM.cpp.o"
	cd /home/cyh/FlightCode/RockPiFlight/build/src && /home/cyh/openwrt-SingleFlight/openwrt-sdk-rockchip-armv8_gcc-11.2.0_musl.Linux-x86_64/staging_dir/toolchain-aarch64_generic_gcc-11.2.0_musl/bin/aarch64-openwrt-linux-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ROCKPIAPI.dir/RockPiAPM.cpp.o -c /home/cyh/FlightCode/RockPiFlight/src/RockPiAPM.cpp

src/CMakeFiles/ROCKPIAPI.dir/RockPiAPM.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ROCKPIAPI.dir/RockPiAPM.cpp.i"
	cd /home/cyh/FlightCode/RockPiFlight/build/src && /home/cyh/openwrt-SingleFlight/openwrt-sdk-rockchip-armv8_gcc-11.2.0_musl.Linux-x86_64/staging_dir/toolchain-aarch64_generic_gcc-11.2.0_musl/bin/aarch64-openwrt-linux-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cyh/FlightCode/RockPiFlight/src/RockPiAPM.cpp > CMakeFiles/ROCKPIAPI.dir/RockPiAPM.cpp.i

src/CMakeFiles/ROCKPIAPI.dir/RockPiAPM.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ROCKPIAPI.dir/RockPiAPM.cpp.s"
	cd /home/cyh/FlightCode/RockPiFlight/build/src && /home/cyh/openwrt-SingleFlight/openwrt-sdk-rockchip-armv8_gcc-11.2.0_musl.Linux-x86_64/staging_dir/toolchain-aarch64_generic_gcc-11.2.0_musl/bin/aarch64-openwrt-linux-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cyh/FlightCode/RockPiFlight/src/RockPiAPM.cpp -o CMakeFiles/ROCKPIAPI.dir/RockPiAPM.cpp.s

# Object files for target ROCKPIAPI
ROCKPIAPI_OBJECTS = \
"CMakeFiles/ROCKPIAPI.dir/RockPiAPM.cpp.o"

# External object files for target ROCKPIAPI
ROCKPIAPI_EXTERNAL_OBJECTS =

src/libROCKPIAPI.a: src/CMakeFiles/ROCKPIAPI.dir/RockPiAPM.cpp.o
src/libROCKPIAPI.a: src/CMakeFiles/ROCKPIAPI.dir/build.make
src/libROCKPIAPI.a: src/CMakeFiles/ROCKPIAPI.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/cyh/FlightCode/RockPiFlight/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libROCKPIAPI.a"
	cd /home/cyh/FlightCode/RockPiFlight/build/src && $(CMAKE_COMMAND) -P CMakeFiles/ROCKPIAPI.dir/cmake_clean_target.cmake
	cd /home/cyh/FlightCode/RockPiFlight/build/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ROCKPIAPI.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/ROCKPIAPI.dir/build: src/libROCKPIAPI.a

.PHONY : src/CMakeFiles/ROCKPIAPI.dir/build

src/CMakeFiles/ROCKPIAPI.dir/clean:
	cd /home/cyh/FlightCode/RockPiFlight/build/src && $(CMAKE_COMMAND) -P CMakeFiles/ROCKPIAPI.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/ROCKPIAPI.dir/clean

src/CMakeFiles/ROCKPIAPI.dir/depend:
	cd /home/cyh/FlightCode/RockPiFlight/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cyh/FlightCode/RockPiFlight /home/cyh/FlightCode/RockPiFlight/src /home/cyh/FlightCode/RockPiFlight/build /home/cyh/FlightCode/RockPiFlight/build/src /home/cyh/FlightCode/RockPiFlight/build/src/CMakeFiles/ROCKPIAPI.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/ROCKPIAPI.dir/depend

