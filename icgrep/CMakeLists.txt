cmake_minimum_required (VERSION 2.8)

project (icgrep)

enable_testing()
# The version number.
set (icgrep_VERSION_MAJOR 0)
set (icgrep_VERSION_MINOR 8)

option(BLOCK_SIZE_256 "Compiling with BLOCK_SIZE=256")
if (BLOCK_SIZE_256)
  message("-- Build with BLOCK_SIZE=256")
else ()
  message("-- Build with BLOCK_SIZE=128")
endif()

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
llvm_map_components_to_libraries(REQ_LLVM_LIBRARIES mcjit native)

message(STATUS "Found LLVM ${LLVM_PACKAGE_VERSION}")

# Check if boost has been installed on this system.
set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_MULTITHREADED OFF)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost COMPONENTS system REQUIRED)

add_library(PabloADT pablo/pe_and.cpp pablo/pe_not.cpp pablo/pe_or.cpp  pablo/pabloAST.cpp  pablo/pe_sel.cpp  pablo/pe_xor.cpp pablo/codegenstate.cpp  pablo/symbol_generator.cpp pablo/analysis/useanalysis.cpp pablo/printer_pablos.cpp pablo/pablo_compiler.cpp)
add_library(RegExpADT re/re_re.cpp re/re_cc.cpp re/re_parser.cpp re/re_rep.cpp re/parsefailure.cpp re/re_nullable.cpp re/re_simplifier.cpp re/re_compiler.cpp re/printer_re.cpp re/re_diff.cpp re/re_intersect.cpp)
add_library(CCADT cc/cc_namemap.cpp cc/cc_compiler.cpp utf_encoding.cpp utf8_encoder.cpp unicode_categories.h UCD/CaseFolding_txt.cpp)

include_directories("${PROJECT_SOURCE_DIR}")
include_directories("${PROJECT_SOURCE_DIR}/re")
include_directories("${PROJECT_SOURCE_DIR}/cc")
include_directories("${PROJECT_SOURCE_DIR}/pablo")
include_directories("${PROJECT_SOURCE_DIR}/UCD")
include_directories("${PROJECT_SOURCE_DIR}/include")
include_directories("${PROJECT_SOURCE_DIR}/include/simd-lib")
include_directories("${PROJECT_SOURCE_DIR}/include/simd-lib/idisa_cpp")

# add the executable
add_executable(icgrep icgrep.cpp do_grep.cpp compiler.cpp)

target_link_libraries (CCADT PabloADT RegExpADT)
target_link_libraries (PabloADT ${REQ_LLVM_LIBRARIES})
target_link_libraries (icgrep PabloADT RegExpADT CCADT ${REQ_LLVM_LIBRARIES})
# If Boost is on the system, include the headers and libraries
IF(Boost_FOUND)
    include_directories("${Boost_INCLUDE_DIRS}")
    link_directories(${Boost_LIBRARY_DIR})
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_BOOST")
    target_link_libraries (CCADT ${Boost_LIBRARIES})
    target_link_libraries (PabloADT ${Boost_LIBRARIES})
    target_link_libraries (RegExpADT ${Boost_LIBRARIES})
ENDIF()

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
  #SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /GR-")
ELSE() # using Clang, GCC, Intel C++, etc
  #SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-rtti")
ENDIF()

#Enable all warnings
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

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

set(AVX_1_CODE "
#include <immintrin.h>
int main()
{
__m256* a=(__m256*)_mm_malloc(4, 32);
__m256 b = _mm256_load_ps((float*)a);
b = _mm256_or_ps(b, b);
_mm_free(a);
return 0;
}")

set(AVX_2_CODE "
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
  SET(ENABLE_AVX_1 "/arch:AVX1")
  SET(ENABLE_AVX_2 "/arch:AVX2")
ELSE() # using Clang, GCC, Intel C++, etc  
  SET(ENABLE_SSE1 "-msse")
  SET(ENABLE_SSE2 "-msse2")
  SET(ENABLE_SSE3 "-msse3")
  SET(ENABLE_SSE4_1 "-msse4.1")
  SET(ENABLE_SSE4_2 "-msse4.2")
  IF ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    SET(ENABLE_AVX_1 "-mavx -fabi-version=6")
    SET(ENABLE_AVX_2 "-mavx2 -fabi-version=6")  
  ELSE()
    SET(ENABLE_AVX_1 "-mavx")
    SET(ENABLE_AVX_2 "-mavx2")
  ENDIF()
ENDIF()

set(CMAKE_REQUIRED_FLAGS)
set(SIMD_SUPPORT_FOUND)
set(SIMD_MAX_BLOCK_SIZE)

MACRO(CHECK_SIMD_SUPPORT type maxBlockSize)
  IF(NOT SIMD_SUPPORT_FOUND)  
    set(CMAKE_REQUIRED_FLAGS_SAVE ${CMAKE_REQUIRED_FLAGS})
    set(CMAKE_REQUIRED_FLAGS ${ENABLE_${type}})
    check_cxx_source_runs("${${type}_CODE}" ${type})    
    IF(${type})
      SET(SIMD_SUPPORT_FOUND ${CMAKE_REQUIRED_FLAGS})
      SET(SIMD_MAX_BLOCK_SIZE ${maxBlockSize})
    ENDIF()
    SET(CMAKE_REQUIRED_FLAGS ${CMAKE_REQUIRED_FLAGS_SAVE})
  ENDIF()
ENDMACRO()

CHECK_SIMD_SUPPORT("AVX_2" 256)
CHECK_SIMD_SUPPORT("AVX_1" 128)
CHECK_SIMD_SUPPORT("SSE4_2" 128)
CHECK_SIMD_SUPPORT("SSE4_1" 128)
CHECK_SIMD_SUPPORT("SSE3" 128)
CHECK_SIMD_SUPPORT("SSE2" 128)
CHECK_SIMD_SUPPORT("SSE1" 128)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${SIMD_SUPPORT_FOUND}")

# detect LLVM version and add compiler flags.
if (${LLVM_PACKAGE_VERSION} MATCHES "3.4")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_LLVM_3_4")
elseif (${LLVM_PACKAGE_VERSION} MATCHES "3.5")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_LLVM_3_5")
endif(${LLVM_PACKAGE_VERSION} MATCHES "3.4")

IF (BLOCK_SIZE_256)
  IF(${SIMD_MAX_BLOCK_SIZE} GREATER 128)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_S2P_AVX2 -DBLOCK_SIZE=256 -march=core-avx2 -m64")
  ELSE()
    MESSAGE(FATAL_ERROR "AVX2 or better is required for BLOCK_SIZE=256")
  ENDIF()
ENDIF()

SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -DNDEBUG")

add_test(
  NAME greptest
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/../QA
  COMMAND python greptest.py ${CMAKE_BINARY_DIR}/icgrep)

add_test(
  NAME abc_test
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/../QA
  COMMAND ./run_abc ../icgrep-build/icgrep)

add_custom_target (abc_test
  COMMAND python abc_testgen.py
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/../QA)

add_custom_target (check
  COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure
  DEPENDS icgrep abc_test)

add_custom_target (perf_icgrep
  COMMAND python perf.py icgrep
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/../performance
  DEPENDS icgrep)

add_custom_target (viewer
  COMMAND python viewer.py
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/../performance)

add_custom_target (text_viewer
  COMMAND python viewer.py -t
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/../performance)
