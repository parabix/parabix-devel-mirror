cmake_minimum_required (VERSION 2.8)

project (icgrep)

enable_testing()
# The version number.
set (icgrep_VERSION_MAJOR 1)
set (icgrep_VERSION_MINOR 0)

option(ENABLE_MULTIPLEXING "Compiling the Multiplexing Module")
option(ENABLE_PREGENERATED_UCD_FUNCTIONS "Enable compiling the pregenerated UCD functions")
option(PRINT_TIMING_INFORMATION "Write compilation and execution timing information to standard error stream")

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
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${LLVM_ROOT}/share/llvm/cmake;${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(LLVMConfig)

# Now set the header and library paths:
include_directories( ${LLVM_INCLUDE_DIRS} )
link_directories( ${LLVM_LIBRARY_DIRS} )
add_definitions( ${LLVM_DEFINITIONS} )

# Let's suppose we want to build a JIT compiler with support for
# binary code (no interpreter):
llvm_map_components_to_libnames(REQ_LLVM_LIBRARIES mcjit native IRReader) # ipo

message(STATUS "Found LLVM ${LLVM_PACKAGE_VERSION}")

# Check if boost has been installed on this system.
set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_MULTITHREADED OFF)
set(Boost_USE_STATIC_RUNTIME ON)
find_package(Boost 1.46 REQUIRED COMPONENTS system filesystem iostreams)

include_directories("${Boost_INCLUDE_DIRS}")
link_directories(${Boost_LIBRARY_DIR})
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_BOOST")

SET(PABLO_SRC pablo/pabloAST.cpp pablo/ps_if.cpp pablo/ps_while.cpp pablo/function.cpp pablo/codegenstate.cpp pablo/builder.cpp pablo/symbol_generator.cpp pablo/printer_pablos.cpp pablo/pablo_toolchain.cpp)
SET(PABLO_SRC ${PABLO_SRC} pablo/pablo_compiler.cpp pablo/carry_manager.cpp pablo/carry_data.cpp IDISA/idisa_builder.cpp IDISA/idisa_avx_builder.cpp IDISA/idisa_i64_builder.cpp IDISA/idisa_sse_builder.cpp IDISA/idisa_target.cpp)
SET(PABLO_SRC ${PABLO_SRC} kernels/s2p_kernel.cpp kernels/kernel.cpp kernels/instance.cpp)
SET(PABLO_SRC ${PABLO_SRC} pablo/analysis/pabloverifier.cpp)
SET(PABLO_SRC ${PABLO_SRC} pablo/optimizers/pablo_simplifier.cpp pablo/optimizers/codemotionpass.cpp)
IF(ENABLE_MULTIPLEXING)
SET(PABLO_SRC ${PABLO_SRC} pablo/optimizers/distributivepass.cpp pablo/passes/flattenassociativedfg.cpp pablo/passes/factorizedfg.cpp)
SET(PABLO_SRC ${PABLO_SRC} pablo/optimizers/schedulingprepass.cpp pablo/optimizers/pablo_automultiplexing.cpp pablo/optimizers/pablo_bddminimization.cpp)
ENDIF()

add_library(PabloADT ${PABLO_SRC})
add_library(RegExpADT re/re_re.cpp re/re_cc.cpp re/re_rep.cpp re/re_diff.cpp re/re_intersect.cpp re/printer_re.cpp)
add_library(RegExpCompiler re/re_parser.cpp re/parsefailure.cpp grep_engine.cpp kernels/pipeline.cpp kernels/scanmatchgen.cpp re/re_nullable.cpp re/re_simplifier.cpp re/re_compiler.cpp re/re_analysis.cpp re/re_toolchain.cpp)
add_library(CCADT cc/cc_compiler.cpp utf8_encoder.cpp UCD/CaseFolding_txt.cpp)
add_library(UCDlib UCD/unicode_set.cpp UCD/ucd_compiler.cpp UCD/PropertyObjects.cpp UCD/resolve_properties.cpp UCD/UnicodeNameData.cpp)

# add the executable
target_link_libraries (PabloADT ${REQ_LLVM_LIBRARIES})
target_link_libraries (CCADT PabloADT)
target_link_libraries (UCDlib RegExpADT PabloADT CCADT)
target_link_libraries (RegExpADT PabloADT CCADT UCDlib)
target_link_libraries (RegExpCompiler RegExpADT)

set(PRECOMPILED_FILES "")

IF(NOT ENABLE_PREGENERATED_UCD_FUNCTIONS)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DDISABLE_PREGENERATED_UCD_FUNCTIONS")
ELSE()
MESSAGE("Enabling predefined UCD functions...")
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

add_executable(icgrep icgrep.cpp toolchain.cpp grep_engine.cpp object_cache.cpp kernels/pipeline.cpp kernels/scanmatchgen.cpp ${PRECOMPILED_FILES})
add_executable(casefold casefold.cpp kernels/p2s_kernel.cpp kernels/stdout_kernel.cpp kernels/casefold_pipeline.cpp)
add_executable(u8u16 u8u16.cpp kernels/p2s_kernel.cpp kernels/stdout_kernel.cpp kernels/u8u16_pipeline.cpp kernels/deletion.cpp)
add_executable(symtbl symboltable.cpp kernels/symboltablepipeline.cpp kernels/stdout_kernel.cpp toolchain.cpp grep_engine.cpp object_cache.cpp ${PRECOMPILED_FILES})

IF(ENABLE_PREGENERATED_UCD_FUNCTIONS)
add_dependencies(icgrep run_generate_predefined_ucd_functions)
ENDIF()

IF (PRINT_TIMING_INFORMATION)
    find_package(PAPI REQUIRED)
    include_directories(${PAPI_INCLUDE_DIRS})
    target_link_libraries(icgrep ${PAPI_LIBRARIES})
ENDIF()

target_link_libraries(icgrep ${Boost_LIBRARIES})
target_link_libraries(symtbl ${Boost_LIBRARIES})
target_link_libraries(casefold ${Boost_LIBRARIES})
target_link_libraries(u8u16 ${Boost_LIBRARIES})

target_link_libraries (icgrep UCDlib PabloADT RegExpCompiler CCADT ${REQ_LLVM_LIBRARIES})
target_link_libraries (symtbl UCDlib PabloADT RegExpCompiler CCADT ${REQ_LLVM_LIBRARIES})
target_link_libraries (casefold UCDlib PabloADT RegExpCompiler CCADT ${REQ_LLVM_LIBRARIES})
target_link_libraries (u8u16 UCDlib PabloADT RegExpCompiler CCADT ${REQ_LLVM_LIBRARIES})

IF(ENABLE_MULTIPLEXING)
message(STATUS "Enabling Multiplexing")
SET(BUDDY_ROOT "${CMAKE_SOURCE_DIR}/../buddy-2.4/src")
SET(BUDDY_SOURCES ${BUDDY_ROOT}/bddop.cpp ${BUDDY_ROOT}/cache.cpp ${BUDDY_ROOT}/imatrix.cpp ${BUDDY_ROOT}/kernel.cpp)
SET(BUDDY_SOURCES ${BUDDY_SOURCES} ${BUDDY_ROOT}/prime.cpp ${BUDDY_ROOT}/pairs.cpp ${BUDDY_ROOT}/reorder.cpp ${BUDDY_ROOT}/tree.cpp)
add_library(BUDDY ${BUDDY_SOURCES})
include_directories(${BUDDY_ROOT})
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


SET(CMAKE_REQUIRED_FLAGS)

# detect LLVM version and add compiler flags.
IF (${LLVM_PACKAGE_VERSION} MATCHES "3.5")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_LLVM_3_5")
ENDIF()

IF (PRINT_TIMING_INFORMATION)   
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DPRINT_TIMING_INFORMATION")
ENDIF()

SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -DNDEBUG")
IF (${CMAKE_SYSTEM} MATCHES "Linux")
    SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -g -fsanitize=address -fno-omit-frame-pointer")
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
