cmake_minimum_required (VERSION 2.8)

project (icgrep)

enable_testing()
# The version number.
set (icgrep_VERSION_MAJOR 1)
set (icgrep_VERSION_MINOR 0)

option(BLOCK_SIZE_256 "Compiling with BLOCK_SIZE=256")
if (BLOCK_SIZE_256)
  message("-- Build with BLOCK_SIZE=256")
else ()
  message("-- Build with BLOCK_SIZE=128")
endif()
option(ENABLE_MULTIPLEXING "Compiling the Multiplexing Module")
option(USE_BOOST_MMAP "Using mmap from Boost.Iostreams")
option(DISABLE_PREGENERATED_UCD_FUNCTIONS "Disable compiling the pregenerated UCD functions")


# configure a header file to pass some of the CMake settings
# to the source code
#configure_file (
#  "${PROJECT_SOURCE_DIR}/icgrepconfig.h.in"
#  "${PROJECT_BINARY_DIR}/icgrepconfig.h"
#  )

# LLVM set up
#
# A convenience variable:
set(LLVM_ROOT "../libllvm" CACHE PATH "Root of LLVM install.")

# A bit of a sanity check:
if( NOT EXISTS ${LLVM_ROOT}/include/llvm )
message(FATAL_ERROR "LLVM_ROOT (${LLVM_ROOT}) is not a valid LLVM install")
endif()

# We incorporate the CMake features provided by LLVM:
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${LLVM_ROOT}/share/llvm/cmake")
include(LLVMConfig)

# Now set the header and library paths:
include_directories( ${LLVM_INCLUDE_DIRS} )
link_directories( ${LLVM_LIBRARY_DIRS} )
add_definitions( ${LLVM_DEFINITIONS} )

# Let's suppose we want to build a JIT compiler with support for
# binary code (no interpreter):
llvm_map_components_to_libnames(REQ_LLVM_LIBRARIES mcjit native IRReader)

message(STATUS "Found LLVM ${LLVM_PACKAGE_VERSION}")

# Check if boost has been installed on this system.
set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_MULTITHREADED OFF)
set(Boost_USE_STATIC_RUNTIME ON)

if(ENABLE_MULTIPLEXING)
  SET(REQ_BOOST_COMPONENTS ${REQ_BOOST_COMPONENTS} system)
endif()
if(USE_BOOST_MMAP)
  SET(REQ_BOOST_COMPONENTS ${REQ_BOOST_COMPONENTS} system iostreams filesystem)
endif()

if(REQ_BOOST_COMPONENTS)
  find_package(Boost COMPONENTS ${REQ_BOOST_COMPONENTS} REQUIRED)
endif()

SET(PABLO_SRC pablo/pabloAST.cpp pablo/ps_if.cpp pablo/ps_while.cpp pablo/function.cpp pablo/codegenstate.cpp pablo/builder.cpp pablo/symbol_generator.cpp pablo/printer_pablos.cpp)
SET(PABLO_SRC ${PABLO_SRC} pablo/pablo_compiler.cpp pablo/carry_manager.cpp pablo/carry_data.cpp IDISA/idisa_builder.cpp)
SET(PABLO_SRC ${PABLO_SRC} pablo/analysis/pabloverifier.cpp)
SET(PABLO_SRC ${PABLO_SRC} pablo/optimizers/pablo_simplifier.cpp pablo/optimizers/codemotionpass.cpp pablo/optimizers/booleanreassociationpass.cpp)
IF (ENABLE_MULTIPLEXING)
SET(PABLO_SRC ${PABLO_SRC} pablo/optimizers/pablo_automultiplexing.cpp pablo/optimizers/pablo_bddminimization.cpp)
ENDIF()

add_library(PabloADT ${PABLO_SRC})
add_library(RegExpADT re/re_re.cpp re/re_cc.cpp re/re_rep.cpp re/re_diff.cpp re/re_intersect.cpp re/printer_re.cpp)
add_library(RegExpCompiler re/re_parser.cpp re/parsefailure.cpp re/re_nullable.cpp re/re_simplifier.cpp re/re_compiler.cpp re/re_analysis.cpp)
add_library(CCADT cc/cc_compiler.cpp utf8_encoder.cpp UCD/CaseFolding_txt.cpp)
add_library(UCDlib UCD/unicode_set.cpp UCD/ucd_compiler.cpp UCD/PropertyObjects.cpp UCD/resolve_properties.cpp)


# add the executable
target_link_libraries (PabloADT ${REQ_LLVM_LIBRARIES})
target_link_libraries (CCADT PabloADT)
target_link_libraries (UCDlib RegExpADT PabloADT CCADT)
target_link_libraries (RegExpADT PabloADT CCADT UCDlib)
target_link_libraries (RegExpCompiler RegExpADT)

set(PRECOMPILED_FILES "")

IF(DISABLE_PREGENERATED_UCD_FUNCTIONS)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DDISABLE_PREGENERATED_UCD_FUNCTIONS")
MESSAGE("Disabling predefined UCD functions...")
ELSE()

# add the executable
add_executable(generate_predefined_ucd_functions generate_predefined_ucd_functions.cpp)
llvm_map_components_to_libnames(REQ_LLVM_TARGETS ${LLVM_TARGETS_TO_BUILD})
target_link_libraries (generate_predefined_ucd_functions RegExpADT UCDlib PabloADT CCADT ${REQ_LLVM_TARGETS})

set(PRECOMPILED_PROPERTIES_OBJ ${PROJECT_BINARY_DIR}/precompiled_properties.o)
set(PRECOMPILED_FILES ${PRECOMPILED_PROPERTIES_OBJ} ${PROJECT_SOURCE_DIR}/UCD/precompiled_properties.cpp)

if(ENABLE_MULTIPLEXING)
set(MULTIPLEXING_FLAG -multiplexing -reassoc -multiplexing-dist=${PROJECT_BINARY_DIR}/ucd-multiplexing.csv) # -multiplexing-dist=${PROJECT_BINARY_DIR}/ucd-multiplexing.csv #-ldc=ldc.csv
endif()

add_custom_command(OUTPUT ${PRECOMPILED_FILES}
  COMMAND generate_predefined_ucd_functions
  ARGS -o ${PRECOMPILED_PROPERTIES_OBJ} -dir ${PROJECT_SOURCE_DIR}/UCD/ ${MULTIPLEXING_FLAG} -DefaultIfHierarchy
  DEPENDS generate_predefined_ucd_functions
  COMMENT "Building predefined UCD functions..."
  VERBATIM)

add_custom_target(run_generate_predefined_ucd_functions DEPENDS ${PRECOMPILED_FILES})

add_dependencies(RegExpCompiler run_generate_predefined_ucd_functions)
ENDIF()

add_executable(icgrep icgrep.cpp toolchain.cpp do_grep.cpp ${PRECOMPILED_FILES})
IF(NOT DISABLE_PREGENERATED_UCD_FUNCTIONS)
add_dependencies(icgrep run_generate_predefined_ucd_functions)
ENDIF()
IF(Boost_FOUND)
    include_directories("${Boost_INCLUDE_DIRS}")
    link_directories(${Boost_LIBRARY_DIR})
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_BOOST")
    target_link_libraries(icgrep ${Boost_LIBRARIES})
ENDIF()

target_link_libraries (icgrep UCDlib PabloADT RegExpCompiler CCADT ${REQ_LLVM_LIBRARIES})

IF(USE_BOOST_MMAP)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_BOOST_MMAP")
ENDIF()


IF (ENABLE_MULTIPLEXING)
message(STATUS "Enabling Multiplexing")
SET(BUDDY_ROOT "${CMAKE_SOURCE_DIR}/../buddy-2.4")
file(GLOB BUDDY_SOURCES "${BUDDY_ROOT}/src/*.cpp")
add_library(BUDDY ${BUDDY_SOURCES})
include_directories(${BUDDY_ROOT}/src)
target_link_libraries (PabloADT BUDDY)
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

#Enable all warnings
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

#Check compiler and system architecture support for SSE or AVX
include(CheckCXXSourceRuns)

set(SSE1_CODE "
#include <xmmintrin.h>
int main()
{
__m128 a;
float vals[4] = {0,0,0,0};
a = _mm_loadu_ps(vals);
return 0;
}")

set(SSE2_CODE "
#include <emmintrin.h>
int main()
{
__m128d a;
double vals[2] = {0,0};
a = _mm_loadu_pd(vals);
return 0;
}")

set(SSE3_CODE "
#include <pmmintrin.h>
int main( )
{
const int vals[4] = {0,0,0,0};
__m128i a;
a = _mm_lddqu_si128( (const __m128i*)vals );
return 0;
}")

set(SSE4_1_CODE "
#include <smmintrin.h>
int main ()
{
__m128i a, b;
__m128i res = _mm_max_epi8(a, b);
return 0;
}")

set(SSE4_2_CODE "
#include <nmmintrin.h>
int main()
{
__m128i a, b, c;
c = _mm_cmpgt_epi64(a, b);
return 0;
}")

set(AVX1_CODE "
#include <immintrin.h>
int main()
{
__m256* a=(__m256*)_mm_malloc(4, 32);
__m256 b = _mm256_load_ps((float*)a);
b = _mm256_or_ps(b, b);
_mm_free(a);
return 0;
}")

set(AVX2_CODE "
#include <immintrin.h>
int main()
{
__m256i* a = (__m256i*)_mm_malloc(4, 32);
__m256i b = _mm256_load_si256(a);
b = _mm256_or_si256(b, b);
_mm_free(a);
return 0;
}")

IF (MSVC) # using Visual Studio C++ 
  SET(ENABLE_SSE1 "/arch:SSE")
  SET(ENABLE_SSE2 "/arch:SSE2")
  SET(ENABLE_SSE3 "/arch:SSE3")
  SET(ENABLE_SSE4_1 "/arch:SSE4")
  SET(ENABLE_SSE4_2 "/arch:SSE4")
  SET(ENABLE_AVX1 "/arch:AVX1")
  SET(ENABLE_AVX2 "/arch:AVX2")
ELSE() # using Clang, GCC, Intel C++, etc  
  SET(ENABLE_SSE1 "-msse")
  SET(ENABLE_SSE2 "-msse2")
  SET(ENABLE_SSE3 "-msse3")
  SET(ENABLE_SSE4_1 "-msse4.1")
  SET(ENABLE_SSE4_2 "-msse4.2")
  IF ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    SET(ENABLE_AVX1 "-mavx -fabi-version=6")
    SET(ENABLE_AVX2 "-mavx2 -fabi-version=6")  
  ELSE()
    SET(ENABLE_AVX1 "-mavx")
    SET(ENABLE_AVX2 "-mavx2")
  ENDIF()
ENDIF()

SET(CMAKE_REQUIRED_FLAGS)

MACRO(CHECK_SIMD_SUPPORT SIMD_TYPE RESULT)
  IF(NOT RESULT)  
    SET(CMAKE_REQUIRED_FLAGS_SAVE ${CMAKE_REQUIRED_FLAGS})
    SET(CMAKE_REQUIRED_FLAGS ${ENABLE_${SIMD_TYPE}})
    SET(SUCCESS "${SIMD_TYPE}")
    check_cxx_source_runs("${${SIMD_TYPE}_CODE}" ${SUCCESS})    
    IF(${SUCCESS})
      SET(${RESULT} "${SIMD_TYPE}")
    ENDIF()
    SET(CMAKE_REQUIRED_FLAGS ${CMAKE_REQUIRED_FLAGS_SAVE})
  ENDIF()
ENDMACRO()

IF(NOT SIMD_SUPPORT)
SET(RESULT)
CHECK_SIMD_SUPPORT("AVX2" RESULT)
CHECK_SIMD_SUPPORT("AVX1" RESULT)
CHECK_SIMD_SUPPORT("SSE4_2" RESULT)
CHECK_SIMD_SUPPORT("SSE4_1" RESULT)
CHECK_SIMD_SUPPORT("SSE3" RESULT)
CHECK_SIMD_SUPPORT("SSE2" RESULT)
CHECK_SIMD_SUPPORT("SSE1" RESULT)
SET(SIMD_SUPPORT ${RESULT} CACHE STRING "SSE2")
ENDIF()
#message("SIMD_SUPPORT=" ${SIMD_SUPPORT})
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ENABLE_${SIMD_SUPPORT}}")

# detect LLVM version and add compiler flags.
IF (${LLVM_PACKAGE_VERSION} MATCHES "3.5")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_LLVM_3_5")
ENDIF()

IF (BLOCK_SIZE_256)
    IF ("${SIMD_SUPPORT}" STREQUAL "AVX1")
        SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_S2P_AVX -DUSE_AVX -DBLOCK_SIZE=256 -mavx -m64")
    ELSEIF ("${SIMD_SUPPORT}" STREQUAL "AVX2")
        SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_S2P_AVX2 -DBLOCK_SIZE=256 -march=core-avx2 -m64")
    ENDIF()
ENDIF()

SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -DNDEBUG")
IF (${CMAKE_SYSTEM} MATCHES "Linux")
    SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -g -fsanitize=address") # -fsanitize=address
ENDIF()

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
  COMMAND ./run_abc ../icgrep-build/icgrep)

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
