cmake_minimum_required (VERSION 2.8)

project (icgrep)

enable_testing()
# The version number.
set (icgrep_VERSION_MAJOR 1)
set (icgrep_VERSION_MINOR 6)

option(ENABLE_MULTIPLEXING "Compiling the Multiplexing Module")
option(PRINT_TIMING_INFORMATION "Write compilation and execution timing information to standard error stream")
option(DISABLE_DUAL_ABI "Disable GCC Dual ABI support" OFF)
find_package(LLVM REQUIRED CONFIG)

message(STATUS "Found LLVM ${LLVM_PACKAGE_VERSION} in: ${LLVM_DIR}")
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${CMAKE_CURRENT_SOURCE_DIR}/cmake") # ensure custom FindXYZ.cmake files are found

# Now set the header and library paths:
include_directories( ${LLVM_INCLUDE_DIRS} )
link_directories( ${LLVM_LIBRARY_DIRS} )
add_definitions( ${LLVM_DEFINITIONS} )

option(ENABLE_CUDA_COMPILE "Compiling with CUDA")
set(CUDA_LIB "")
if(ENABLE_CUDA_COMPILE)
  find_package(CUDA)
  if(CUDA_FOUND)
    set(CUDA_INCLUDE ${CUDA_INCLUDE} "/usr/local/cuda-7.5/include")
    include_directories(${CUDA_INCLUDE})
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DCUDA_ENABLED")
    SET(CUDA_LIB "cuda")
  endif(CUDA_FOUND)
endif()

# Let's suppose we want to build a JIT compiler with support for
# binary code (no interpreter):
set(LLVM_ALL_TARGETS AArch64 AMDGPU ARM BPF CppBackend Hexagon Mips MSP430 NVPTX PowerPC Sparc SystemZ X86 XCore)
llvm_map_components_to_libnames(REQ_LLVM_LIBRARIES ${LLVM_ALL_TARGETS} mcjit native IRReader Linker)

# Check if boost has been installed on this system.
set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_MULTITHREADED OFF)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost 1.46 REQUIRED COMPONENTS system filesystem iostreams)
message(STATUS "Found Boost_LIBRARY_DIR: ${Boost_LIBRARY_DIR}")

include_directories("${Boost_INCLUDE_DIRS}")
link_directories(${Boost_LIBRARY_DIR})

SET(KERNEL_SRC kernels/kernel.cpp kernels/pipeline.cpp kernels/s2p_kernel.cpp kernels/streamset.cpp kernels/interface.cpp kernels/mmap_kernel.cpp)

SET(IDISA_SRC IR_Gen/CBuilder.cpp IR_Gen/idisa_builder.cpp IR_Gen/idisa_avx_builder.cpp IR_Gen/idisa_i64_builder.cpp IR_Gen/idisa_sse_builder.cpp IR_Gen/idisa_nvptx_builder.cpp IR_Gen/idisa_target.cpp)

SET(PABLO_SRC pablo/pabloAST.cpp pablo/branch.cpp pablo/codegenstate.cpp pablo/builder.cpp pablo/symbol_generator.cpp pablo/printer_pablos.cpp pablo/pablo_toolchain.cpp)
SET(PABLO_SRC ${PABLO_SRC} pablo/pablo_kernel.cpp pablo/pablo_compiler.cpp pablo/carry_manager.cpp)
SET(PABLO_SRC ${PABLO_SRC} pablo/analysis/pabloverifier.cpp)
SET(PABLO_SRC ${PABLO_SRC} pablo/optimizers/pablo_simplifier.cpp pablo/optimizers/codemotionpass.cpp pablo/passes/flattenif.cpp)
IF(ENABLE_MULTIPLEXING)
SET(PABLO_SRC ${PABLO_SRC} pablo/optimizers/booleanreassociationpass.cpp)
SET(PABLO_SRC ${PABLO_SRC} pablo/optimizers/schedulingprepass.cpp pablo/optimizers/pablo_automultiplexing.cpp)
ENDIF()

add_library(CodeGen ${KERNEL_SRC} ${IDISA_SRC} object_cache.cpp)
add_library(PabloADT ${PABLO_SRC})
add_library(RegExpADT re/re_re.cpp re/re_cc.cpp re/re_rep.cpp re/re_diff.cpp re/re_intersect.cpp re/printer_re.cpp)
add_library(RegExpCompiler re/re_parser.cpp re/re_nullable.cpp re/re_simplifier.cpp re/re_compiler.cpp re/re_analysis.cpp re/re_toolchain.cpp re/re_name_resolve.cpp re/re_parser_pcre.cpp re/re_parser_ere.cpp re/re_parser_bre.cpp re/re_parser_prosite.cpp re/re_utility.cpp)
add_library(CCADT cc/cc_compiler.cpp utf8_encoder.cpp utf16_encoder.cpp UCD/CaseFolding_txt.cpp cc/alphabet.cpp)
add_library(UCDlib UCD/unicode_set.cpp UCD/ucd_compiler.cpp UCD/PropertyObjects.cpp UCD/resolve_properties.cpp UCD/UnicodeNameData.cpp)

# add the executable
target_link_libraries (PabloADT CodeGen ${REQ_LLVM_LIBRARIES})
target_link_libraries (CCADT PabloADT)
target_link_libraries (UCDlib RegExpADT PabloADT CCADT)
target_link_libraries (RegExpADT PabloADT CCADT UCDlib)
target_link_libraries (RegExpCompiler RegExpADT)

add_executable(icgrep icgrep.cpp toolchain.cpp grep_engine.cpp kernels/scanmatchgen.cpp kernels/cc_kernel.cpp kernels/cc_scan_kernel.cpp)
add_executable(u8u16 u8u16.cpp toolchain.cpp kernels/p2s_kernel.cpp kernels/deletion.cpp kernels/stdout_kernel.cpp)
add_executable(base64 base64.cpp kernels/radix64.cpp toolchain.cpp kernels/p2s_kernel.cpp kernels/deletion.cpp kernels/stdout_kernel.cpp)
add_executable(wc wc.cpp toolchain.cpp)
add_executable(editd editd/editd.cpp editd/pattern_compiler.cpp toolchain.cpp editd/editdscan_kernel.cpp editd/editd_gpu_kernel.cpp editd/editd_cpu_kernel.cpp)
add_executable(array-test array-test.cpp toolchain.cpp)

## IWYU detects superfluous includes and when the include can be replaced with a forward declaration.
## It can be obtained using "apt-get install iwyu" or from "github.com/include-what-you-use".

#find_program(IWYU_PATH NAMES include-what-you-use iwyu)
#if(IWYU_PATH)
#cmake_minimum_required(VERSION 3.3 FATAL_ERROR)
#execute_process(COMMAND ${CMAKE_CXX_COMPILER} -print-libgcc-file-name OUTPUT_VARIABLE LIBGCC_FILE)
#get_filename_component(LIBGCC_PATH ${LIBGCC_FILE} DIRECTORY)
#include_directories("${LIBGCC_PATH}/include")
#set_property(TARGET CodeGen PROPERTY CXX_INCLUDE_WHAT_YOU_USE ${IWYU_PATH})
#set_property(TARGET PabloADT PROPERTY CXX_INCLUDE_WHAT_YOU_USE ${IWYU_PATH})
#set_property(TARGET RegExpADT PROPERTY CXX_INCLUDE_WHAT_YOU_USE ${IWYU_PATH})
#set_property(TARGET RegExpCompiler PROPERTY CXX_INCLUDE_WHAT_YOU_USE ${IWYU_PATH})
#set_property(TARGET CCADT PROPERTY CXX_INCLUDE_WHAT_YOU_USE ${IWYU_PATH})
#set_property(TARGET UCDlib PROPERTY CXX_INCLUDE_WHAT_YOU_USE ${IWYU_PATH})
#set_property(TARGET icgrep PROPERTY CXX_INCLUDE_WHAT_YOU_USE ${IWYU_PATH})
#set_property(TARGET u8u16 PROPERTY CXX_INCLUDE_WHAT_YOU_USE ${IWYU_PATH})
#set_property(TARGET base64 PROPERTY CXX_INCLUDE_WHAT_YOU_USE ${IWYU_PATH})
#set_property(TARGET wc PROPERTY CXX_INCLUDE_WHAT_YOU_USE ${IWYU_PATH})
#set_property(TARGET editd PROPERTY CXX_INCLUDE_WHAT_YOU_USE ${IWYU_PATH})
#set_property(TARGET array-test PROPERTY CXX_INCLUDE_WHAT_YOU_USE ${IWYU_PATH})
#endif()

IF (PRINT_TIMING_INFORMATION)
    find_package(PAPI REQUIRED)
    include_directories(${PAPI_INCLUDE_DIRS})
    target_link_libraries(icgrep ${PAPI_LIBRARIES})
ENDIF()

target_link_libraries (icgrep UCDlib PabloADT RegExpCompiler CCADT CodeGen ${REQ_LLVM_LIBRARIES} ${Boost_LIBRARIES} ${CUDA_LIB})
target_link_libraries (u8u16 UCDlib PabloADT RegExpCompiler CCADT CodeGen ${REQ_LLVM_LIBRARIES} ${Boost_LIBRARIES} ${CUDA_LIB})
target_link_libraries (base64 UCDlib PabloADT RegExpCompiler CCADT CodeGen ${REQ_LLVM_LIBRARIES} ${Boost_LIBRARIES} ${CUDA_LIB})
target_link_libraries (wc UCDlib PabloADT RegExpCompiler CCADT CodeGen ${REQ_LLVM_LIBRARIES} ${Boost_LIBRARIES} ${CUDA_LIB})
target_link_libraries (editd UCDlib PabloADT RegExpCompiler CCADT CodeGen ${REQ_LLVM_LIBRARIES} ${Boost_LIBRARIES} ${CUDA_LIB})
target_link_libraries (array-test PabloADT CodeGen ${REQ_LLVM_LIBRARIES} ${Boost_LIBRARIES} ${CUDA_LIB})

IF(ENABLE_MULTIPLEXING)
    message(STATUS "Enabling Multiplexing")
    find_package(Z3 REQUIRED)
    include_directories(${Z3_INCLUDE_DIRS})
    target_link_libraries(PabloADT ${Z3_LIBRARIES})
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DENABLE_MULTIPLEXING")
ENDIF()

include_directories("${PROJECT_SOURCE_DIR}")

#Check compiler support for 0x / 11
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else()
  CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
  if(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
  else()
    CHECK_CXX_COMPILER_FLAG("-std=gnu++11" COMPILER_SUPPORTS_GNU11)
    if(COMPILER_SUPPORTS_GNU11)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")
    else()
      CHECK_CXX_COMPILER_FLAG("-std=gnu++0x" COMPILER_SUPPORTS_GNU0X)
      if(COMPILER_SUPPORTS_GNU0X)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++0x")
      endif()
    endif()
  endif()
endif()

#Disable RunTime Type Information
IF (MSVC) # using Visual Studio C++
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /GR-")
ELSE() # using Clang, GCC, Intel C++, etc
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-rtti")
ENDIF()

#Disable Dual ABI support
IF(DISABLE_DUAL_ABI)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_GLIBCXX_USE_CXX11_ABI=0")
ENDIF()

#Enable all warnings
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

SET(CMAKE_REQUIRED_FLAGS)

# detect LLVM version and add compiler flags.
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DLLVM_VERSION_major=${LLVM_VERSION_MAJOR} -DLLVM_VERSION_MINOR=${LLVM_VERSION_MINOR}")
IF (${LLVM_VERSION_MAJOR} EQUAL 3)
  IF (${LLVM_VERSION_MINOR} EQUAL 6)
    message(STATUS "Found LLVM ${LLVM_VERSION_MAJOR}.${LLVM_VERSION_MINOR}")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_LLVM_3_6")
  ELSEIF (${LLVM_VERSION_MINOR} EQUAL 7)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_LLVM_3_7")
  ENDIF()
ENDIF()

IF (PRINT_TIMING_INFORMATION)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DPRINT_TIMING_INFORMATION")
ENDIF()

SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -DNDEBUG")
SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -g -fsanitize=address -fno-omit-frame-pointer")

add_test(
  NAME greptest
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/../QA
  COMMAND python2 greptest.py ${CMAKE_BINARY_DIR}/icgrep)

add_test(
  NAME proptest
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/../QA
  COMMAND python2 greptest.py -d ${PROJECT_SOURCE_DIR}/../QA -t ${PROJECT_SOURCE_DIR}/../QA/proptest.xml ${CMAKE_BINARY_DIR}/icgrep)

add_test(
  NAME abc_test
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/../QA
  COMMAND ./run_abc ${CMAKE_BINARY_DIR}/icgrep)

add_test(
  NAME u8u16_test
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/../QA/u8u16
  COMMAND ./run_all ${CMAKE_BINARY_DIR}/u8u16)

add_test(
  NAME editd_test
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/../QA/editd
  COMMAND python run_all.py -e ${CMAKE_BINARY_DIR})

add_test(
  NAME base64_test
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/../QA/base64
  COMMAND python base64test.py ${CMAKE_BINARY_DIR}/base64)

add_custom_target (u8u16_test
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/../QA/u8u16
  COMMAND ./run_all ${CMAKE_BINARY_DIR}/u8u16)

add_custom_target (editd_test
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/../QA/editd
  COMMAND python run_all.py -e ${CMAKE_BINARY_DIR})

add_custom_target (base64_test
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/../QA/base64
  COMMAND python base64test.py ${CMAKE_BINARY_DIR}/base64)

add_custom_target (abc_test
  COMMAND python2 abc_testgen.py
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/../QA)

add_custom_target (check
  COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure
  DEPENDS icgrep abc_test)

add_custom_target (perf_icgrep
  COMMAND python2 perf.py icgrep
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/../performance
  DEPENDS icgrep)

add_custom_target (viewer
  COMMAND python2 viewer.py
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/../performance)

add_custom_target (text_viewer
  COMMAND python2 viewer.py -t
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/../performance)
