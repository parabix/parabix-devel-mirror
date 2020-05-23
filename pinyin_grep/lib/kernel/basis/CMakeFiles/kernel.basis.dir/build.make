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
CMAKE_COMMAND = /usr/local/Cellar/cmake/3.16.4/bin/cmake

# The command to remove a file.
RM = /usr/local/Cellar/cmake/3.16.4/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/kevinq/Downloads/parabix-devel

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/kevinq/Downloads/parabix-devel/pinyin_grep

# Include any dependencies generated for this target.
include lib/kernel/basis/CMakeFiles/kernel.basis.dir/depend.make

# Include the progress variables for this target.
include lib/kernel/basis/CMakeFiles/kernel.basis.dir/progress.make

# Include the compile flags for this target's objects.
include lib/kernel/basis/CMakeFiles/kernel.basis.dir/flags.make

lib/kernel/basis/CMakeFiles/kernel.basis.dir/p2s_kernel.cpp.o: lib/kernel/basis/CMakeFiles/kernel.basis.dir/flags.make
lib/kernel/basis/CMakeFiles/kernel.basis.dir/p2s_kernel.cpp.o: ../lib/kernel/basis/p2s_kernel.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/kevinq/Downloads/parabix-devel/pinyin_grep/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object lib/kernel/basis/CMakeFiles/kernel.basis.dir/p2s_kernel.cpp.o"
	cd /Users/kevinq/Downloads/parabix-devel/pinyin_grep/lib/kernel/basis && /Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/kernel.basis.dir/p2s_kernel.cpp.o -c /Users/kevinq/Downloads/parabix-devel/lib/kernel/basis/p2s_kernel.cpp

lib/kernel/basis/CMakeFiles/kernel.basis.dir/p2s_kernel.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/kernel.basis.dir/p2s_kernel.cpp.i"
	cd /Users/kevinq/Downloads/parabix-devel/pinyin_grep/lib/kernel/basis && /Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/kevinq/Downloads/parabix-devel/lib/kernel/basis/p2s_kernel.cpp > CMakeFiles/kernel.basis.dir/p2s_kernel.cpp.i

lib/kernel/basis/CMakeFiles/kernel.basis.dir/p2s_kernel.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/kernel.basis.dir/p2s_kernel.cpp.s"
	cd /Users/kevinq/Downloads/parabix-devel/pinyin_grep/lib/kernel/basis && /Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/kevinq/Downloads/parabix-devel/lib/kernel/basis/p2s_kernel.cpp -o CMakeFiles/kernel.basis.dir/p2s_kernel.cpp.s

lib/kernel/basis/CMakeFiles/kernel.basis.dir/s2p_kernel.cpp.o: lib/kernel/basis/CMakeFiles/kernel.basis.dir/flags.make
lib/kernel/basis/CMakeFiles/kernel.basis.dir/s2p_kernel.cpp.o: ../lib/kernel/basis/s2p_kernel.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/kevinq/Downloads/parabix-devel/pinyin_grep/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object lib/kernel/basis/CMakeFiles/kernel.basis.dir/s2p_kernel.cpp.o"
	cd /Users/kevinq/Downloads/parabix-devel/pinyin_grep/lib/kernel/basis && /Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/kernel.basis.dir/s2p_kernel.cpp.o -c /Users/kevinq/Downloads/parabix-devel/lib/kernel/basis/s2p_kernel.cpp

lib/kernel/basis/CMakeFiles/kernel.basis.dir/s2p_kernel.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/kernel.basis.dir/s2p_kernel.cpp.i"
	cd /Users/kevinq/Downloads/parabix-devel/pinyin_grep/lib/kernel/basis && /Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/kevinq/Downloads/parabix-devel/lib/kernel/basis/s2p_kernel.cpp > CMakeFiles/kernel.basis.dir/s2p_kernel.cpp.i

lib/kernel/basis/CMakeFiles/kernel.basis.dir/s2p_kernel.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/kernel.basis.dir/s2p_kernel.cpp.s"
	cd /Users/kevinq/Downloads/parabix-devel/pinyin_grep/lib/kernel/basis && /Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/kevinq/Downloads/parabix-devel/lib/kernel/basis/s2p_kernel.cpp -o CMakeFiles/kernel.basis.dir/s2p_kernel.cpp.s

# Object files for target kernel.basis
kernel_basis_OBJECTS = \
"CMakeFiles/kernel.basis.dir/p2s_kernel.cpp.o" \
"CMakeFiles/kernel.basis.dir/s2p_kernel.cpp.o"

# External object files for target kernel.basis
kernel_basis_EXTERNAL_OBJECTS =

lib/kernel/basis/libparabix_kernel_basis.a: lib/kernel/basis/CMakeFiles/kernel.basis.dir/p2s_kernel.cpp.o
lib/kernel/basis/libparabix_kernel_basis.a: lib/kernel/basis/CMakeFiles/kernel.basis.dir/s2p_kernel.cpp.o
lib/kernel/basis/libparabix_kernel_basis.a: lib/kernel/basis/CMakeFiles/kernel.basis.dir/build.make
lib/kernel/basis/libparabix_kernel_basis.a: lib/kernel/basis/CMakeFiles/kernel.basis.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/kevinq/Downloads/parabix-devel/pinyin_grep/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX static library libparabix_kernel_basis.a"
	cd /Users/kevinq/Downloads/parabix-devel/pinyin_grep/lib/kernel/basis && $(CMAKE_COMMAND) -P CMakeFiles/kernel.basis.dir/cmake_clean_target.cmake
	cd /Users/kevinq/Downloads/parabix-devel/pinyin_grep/lib/kernel/basis && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/kernel.basis.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
lib/kernel/basis/CMakeFiles/kernel.basis.dir/build: lib/kernel/basis/libparabix_kernel_basis.a

.PHONY : lib/kernel/basis/CMakeFiles/kernel.basis.dir/build

lib/kernel/basis/CMakeFiles/kernel.basis.dir/clean:
	cd /Users/kevinq/Downloads/parabix-devel/pinyin_grep/lib/kernel/basis && $(CMAKE_COMMAND) -P CMakeFiles/kernel.basis.dir/cmake_clean.cmake
.PHONY : lib/kernel/basis/CMakeFiles/kernel.basis.dir/clean

lib/kernel/basis/CMakeFiles/kernel.basis.dir/depend:
	cd /Users/kevinq/Downloads/parabix-devel/pinyin_grep && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/kevinq/Downloads/parabix-devel /Users/kevinq/Downloads/parabix-devel/lib/kernel/basis /Users/kevinq/Downloads/parabix-devel/pinyin_grep /Users/kevinq/Downloads/parabix-devel/pinyin_grep/lib/kernel/basis /Users/kevinq/Downloads/parabix-devel/pinyin_grep/lib/kernel/basis/CMakeFiles/kernel.basis.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : lib/kernel/basis/CMakeFiles/kernel.basis.dir/depend
