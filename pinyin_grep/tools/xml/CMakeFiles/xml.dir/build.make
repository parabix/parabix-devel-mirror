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
include tools/xml/CMakeFiles/xml.dir/depend.make

# Include the progress variables for this target.
include tools/xml/CMakeFiles/xml.dir/progress.make

# Include the compile flags for this target's objects.
include tools/xml/CMakeFiles/xml.dir/flags.make

tools/xml/CMakeFiles/xml.dir/post_process.cpp.o: tools/xml/CMakeFiles/xml.dir/flags.make
tools/xml/CMakeFiles/xml.dir/post_process.cpp.o: ../tools/xml/post_process.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/kevinq/Downloads/parabix-devel/pinyin_grep/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object tools/xml/CMakeFiles/xml.dir/post_process.cpp.o"
	cd /Users/kevinq/Downloads/parabix-devel/pinyin_grep/tools/xml && /Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/xml.dir/post_process.cpp.o -c /Users/kevinq/Downloads/parabix-devel/tools/xml/post_process.cpp

tools/xml/CMakeFiles/xml.dir/post_process.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/xml.dir/post_process.cpp.i"
	cd /Users/kevinq/Downloads/parabix-devel/pinyin_grep/tools/xml && /Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/kevinq/Downloads/parabix-devel/tools/xml/post_process.cpp > CMakeFiles/xml.dir/post_process.cpp.i

tools/xml/CMakeFiles/xml.dir/post_process.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/xml.dir/post_process.cpp.s"
	cd /Users/kevinq/Downloads/parabix-devel/pinyin_grep/tools/xml && /Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/kevinq/Downloads/parabix-devel/tools/xml/post_process.cpp -o CMakeFiles/xml.dir/post_process.cpp.s

tools/xml/CMakeFiles/xml.dir/test_suite_error.cpp.o: tools/xml/CMakeFiles/xml.dir/flags.make
tools/xml/CMakeFiles/xml.dir/test_suite_error.cpp.o: ../tools/xml/test_suite_error.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/kevinq/Downloads/parabix-devel/pinyin_grep/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object tools/xml/CMakeFiles/xml.dir/test_suite_error.cpp.o"
	cd /Users/kevinq/Downloads/parabix-devel/pinyin_grep/tools/xml && /Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/xml.dir/test_suite_error.cpp.o -c /Users/kevinq/Downloads/parabix-devel/tools/xml/test_suite_error.cpp

tools/xml/CMakeFiles/xml.dir/test_suite_error.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/xml.dir/test_suite_error.cpp.i"
	cd /Users/kevinq/Downloads/parabix-devel/pinyin_grep/tools/xml && /Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/kevinq/Downloads/parabix-devel/tools/xml/test_suite_error.cpp > CMakeFiles/xml.dir/test_suite_error.cpp.i

tools/xml/CMakeFiles/xml.dir/test_suite_error.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/xml.dir/test_suite_error.cpp.s"
	cd /Users/kevinq/Downloads/parabix-devel/pinyin_grep/tools/xml && /Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/kevinq/Downloads/parabix-devel/tools/xml/test_suite_error.cpp -o CMakeFiles/xml.dir/test_suite_error.cpp.s

tools/xml/CMakeFiles/xml.dir/xml.cpp.o: tools/xml/CMakeFiles/xml.dir/flags.make
tools/xml/CMakeFiles/xml.dir/xml.cpp.o: ../tools/xml/xml.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/kevinq/Downloads/parabix-devel/pinyin_grep/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object tools/xml/CMakeFiles/xml.dir/xml.cpp.o"
	cd /Users/kevinq/Downloads/parabix-devel/pinyin_grep/tools/xml && /Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/xml.dir/xml.cpp.o -c /Users/kevinq/Downloads/parabix-devel/tools/xml/xml.cpp

tools/xml/CMakeFiles/xml.dir/xml.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/xml.dir/xml.cpp.i"
	cd /Users/kevinq/Downloads/parabix-devel/pinyin_grep/tools/xml && /Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/kevinq/Downloads/parabix-devel/tools/xml/xml.cpp > CMakeFiles/xml.dir/xml.cpp.i

tools/xml/CMakeFiles/xml.dir/xml.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/xml.dir/xml.cpp.s"
	cd /Users/kevinq/Downloads/parabix-devel/pinyin_grep/tools/xml && /Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/kevinq/Downloads/parabix-devel/tools/xml/xml.cpp -o CMakeFiles/xml.dir/xml.cpp.s

# Object files for target xml
xml_OBJECTS = \
"CMakeFiles/xml.dir/post_process.cpp.o" \
"CMakeFiles/xml.dir/test_suite_error.cpp.o" \
"CMakeFiles/xml.dir/xml.cpp.o"

# External object files for target xml
xml_EXTERNAL_OBJECTS =

bin/xml: tools/xml/CMakeFiles/xml.dir/post_process.cpp.o
bin/xml: tools/xml/CMakeFiles/xml.dir/test_suite_error.cpp.o
bin/xml: tools/xml/CMakeFiles/xml.dir/xml.cpp.o
bin/xml: tools/xml/CMakeFiles/xml.dir/build.make
bin/xml: lib/pablo/parse/libparabix_pablo_parse.a
bin/xml: lib/kernel/basis/libparabix_kernel_basis.a
bin/xml: lib/kernel/io/libparabix_kernel_io.a
bin/xml: lib/kernel/pipeline/libparabix_kernel_pipeline.a
bin/xml: lib/kernel/scan/libparabix_kernel_scan.a
bin/xml: lib/kernel/streamutils/libparabix_kernel_streamutils.a
bin/xml: lib/kernel/util/libparabix_kernel_util.a
bin/xml: lib/toolchain/libparabix_toolchain.a
bin/xml: lib/pablo/bixnum/libparabix_pablo_bixnum.a
bin/xml: lib/kernel/pipeline/libparabix_kernel_pipeline.a
bin/xml: lib/objcache/libparabix_objcache.a
bin/xml: lib/re/cc/libparabix_re_cc.a
bin/xml: lib/pablo/libparabix_pablo.a
bin/xml: lib/kernel/core/libparabix_kernel_core.a
bin/xml: lib/idisa/libparabix_idisa.a
bin/xml: lib/codegen/libparabix_codegen.a
bin/xml: /usr/local/lib/libboost_system.dylib
bin/xml: /usr/local/lib/libboost_filesystem.dylib
bin/xml: /usr/local/lib/libboost_iostreams.dylib
bin/xml: lib/re/adt/libparabix_re_adt.a
bin/xml: lib/re/alphabet/libparabix_re_alphabet.a
bin/xml: lib/re/toolchain/libparabix_re_toolchain.a
bin/xml: lib/unicode/core/libparabix_unicode_core.a
bin/xml: lib/toolchain/libparabix_toolchain.a
bin/xml: /usr/local/Cellar/llvm/9.0.1/lib/libLLVMX86CodeGen.a
bin/xml: /usr/local/Cellar/llvm/9.0.1/lib/libLLVMAsmPrinter.a
bin/xml: /usr/local/Cellar/llvm/9.0.1/lib/libLLVMDebugInfoDWARF.a
bin/xml: /usr/local/Cellar/llvm/9.0.1/lib/libLLVMGlobalISel.a
bin/xml: /usr/local/Cellar/llvm/9.0.1/lib/libLLVMSelectionDAG.a
bin/xml: /usr/local/Cellar/llvm/9.0.1/lib/libLLVMCodeGen.a
bin/xml: /usr/local/Cellar/llvm/9.0.1/lib/libLLVMBitWriter.a
bin/xml: /usr/local/Cellar/llvm/9.0.1/lib/libLLVMScalarOpts.a
bin/xml: /usr/local/Cellar/llvm/9.0.1/lib/libLLVMAggressiveInstCombine.a
bin/xml: /usr/local/Cellar/llvm/9.0.1/lib/libLLVMInstCombine.a
bin/xml: /usr/local/Cellar/llvm/9.0.1/lib/libLLVMX86AsmParser.a
bin/xml: /usr/local/Cellar/llvm/9.0.1/lib/libLLVMX86Desc.a
bin/xml: /usr/local/Cellar/llvm/9.0.1/lib/libLLVMX86Disassembler.a
bin/xml: /usr/local/Cellar/llvm/9.0.1/lib/libLLVMMCDisassembler.a
bin/xml: /usr/local/Cellar/llvm/9.0.1/lib/libLLVMX86Info.a
bin/xml: /usr/local/Cellar/llvm/9.0.1/lib/libLLVMX86Utils.a
bin/xml: /usr/local/Cellar/llvm/9.0.1/lib/libLLVMMCJIT.a
bin/xml: /usr/local/Cellar/llvm/9.0.1/lib/libLLVMExecutionEngine.a
bin/xml: /usr/local/Cellar/llvm/9.0.1/lib/libLLVMTarget.a
bin/xml: /usr/local/Cellar/llvm/9.0.1/lib/libLLVMRuntimeDyld.a
bin/xml: /usr/local/Cellar/llvm/9.0.1/lib/libLLVMIRReader.a
bin/xml: /usr/local/Cellar/llvm/9.0.1/lib/libLLVMAsmParser.a
bin/xml: /usr/local/Cellar/llvm/9.0.1/lib/libLLVMLinker.a
bin/xml: /usr/local/Cellar/llvm/9.0.1/lib/libLLVMTransformUtils.a
bin/xml: /usr/local/Cellar/llvm/9.0.1/lib/libLLVMAnalysis.a
bin/xml: /usr/local/Cellar/llvm/9.0.1/lib/libLLVMProfileData.a
bin/xml: /usr/local/Cellar/llvm/9.0.1/lib/libLLVMObject.a
bin/xml: /usr/local/Cellar/llvm/9.0.1/lib/libLLVMMCParser.a
bin/xml: /usr/local/Cellar/llvm/9.0.1/lib/libLLVMMC.a
bin/xml: /usr/local/Cellar/llvm/9.0.1/lib/libLLVMDebugInfoCodeView.a
bin/xml: /usr/local/Cellar/llvm/9.0.1/lib/libLLVMDebugInfoMSF.a
bin/xml: /usr/local/Cellar/llvm/9.0.1/lib/libLLVMBitReader.a
bin/xml: /usr/local/Cellar/llvm/9.0.1/lib/libLLVMBitstreamReader.a
bin/xml: /usr/local/Cellar/llvm/9.0.1/lib/libLLVMCore.a
bin/xml: /usr/local/Cellar/llvm/9.0.1/lib/libLLVMBinaryFormat.a
bin/xml: /usr/local/Cellar/llvm/9.0.1/lib/libLLVMRemarks.a
bin/xml: /usr/local/Cellar/llvm/9.0.1/lib/libLLVMSupport.a
bin/xml: /usr/local/Cellar/llvm/9.0.1/lib/libLLVMDemangle.a
bin/xml: tools/xml/CMakeFiles/xml.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/kevinq/Downloads/parabix-devel/pinyin_grep/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable ../../bin/xml"
	cd /Users/kevinq/Downloads/parabix-devel/pinyin_grep/tools/xml && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/xml.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
tools/xml/CMakeFiles/xml.dir/build: bin/xml

.PHONY : tools/xml/CMakeFiles/xml.dir/build

tools/xml/CMakeFiles/xml.dir/clean:
	cd /Users/kevinq/Downloads/parabix-devel/pinyin_grep/tools/xml && $(CMAKE_COMMAND) -P CMakeFiles/xml.dir/cmake_clean.cmake
.PHONY : tools/xml/CMakeFiles/xml.dir/clean

tools/xml/CMakeFiles/xml.dir/depend:
	cd /Users/kevinq/Downloads/parabix-devel/pinyin_grep && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/kevinq/Downloads/parabix-devel /Users/kevinq/Downloads/parabix-devel/tools/xml /Users/kevinq/Downloads/parabix-devel/pinyin_grep /Users/kevinq/Downloads/parabix-devel/pinyin_grep/tools/xml /Users/kevinq/Downloads/parabix-devel/pinyin_grep/tools/xml/CMakeFiles/xml.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : tools/xml/CMakeFiles/xml.dir/depend
