cmake_minimum_required (VERSION 2.8)

project (icgrep)

enable_testing()
# The version number.
set (icgrep_VERSION_MAJOR 1)
set (icgrep_VERSION_MINOR 6)

option(ENABLE_MULTIPLEXING "Compiling the Multiplexing Module")
option(ENABLE_PREGENERATED_UCD_FUNCTIONS "Enable compiling the pregenerated UCD functions")
option(PRINT_TIMING_INFORMATION "Write compilation and execution timing information to standard error stream")

find_package(LLVM REQUIRED CONFIG)

message(STATUS "Found LLVM ${LLVM_PACKAGE_VERSION}")
message(STATUS "Using LLVMConfig.cmake in: ${LLVM_DIR}")


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
set(Boost_USE_STATIC_RUNTIME ON)
find_package(Boost 1.46 REQUIRED COMPONENTS system filesystem iostreams)

include_directories("${Boost_INCLUDE_DIRS}")
link_directories(${Boost_LIBRARY_DIR})
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_BOOST")

SET(IDISA_SRC IDISA/idisa_builder.cpp IDISA/idisa_avx_builder.cpp IDISA/idisa_i64_builder.cpp IDISA/idisa_sse_builder.cpp IDISA/idisa_nvptx_builder.cpp IDISA/idisa_target.cpp)

SET(PABLO_SRC pablo/pablo_type.cpp pablo/pabloAST.cpp pablo/ps_if.cpp pablo/ps_while.cpp pablo/function.cpp pablo/codegenstate.cpp pablo/builder.cpp pablo/symbol_generator.cpp pablo/printer_pablos.cpp pablo/pablo_toolchain.cpp pablo/passes/flattenif.cpp)
SET(PABLO_SRC ${PABLO_SRC} pablo/pablo_compiler.cpp pablo/carry_manager.cpp pablo/carry_data.cpp pablo/pablo_kernel.cpp)
SET(PABLO_SRC ${PABLO_SRC} kernels/s2p_kernel.cpp kernels/kernel.cpp kernels/streamset.cpp kernels/interface.cpp)
SET(PABLO_SRC ${PABLO_SRC} pablo/analysis/pabloverifier.cpp)
SET(PABLO_SRC ${PABLO_SRC} pablo/optimizers/pablo_simplifier.cpp pablo/optimizers/codemotionpass.cpp)
IF(ENABLE_MULTIPLEXING)
SET(PABLO_SRC ${PABLO_SRC} pablo/optimizers/booleanreassociationpass.cpp)
SET(PABLO_SRC ${PABLO_SRC} pablo/optimizers/schedulingprepass.cpp pablo/optimizers/pablo_automultiplexing.cpp)
ENDIF()

add_library(PabloADT ${PABLO_SRC})
add_library(RegExpADT re/re_re.cpp re/re_cc.cpp re/re_rep.cpp re/re_diff.cpp re/re_intersect.cpp re/printer_re.cpp)
add_library(RegExpCompiler re/re_parser.cpp re/re_nullable.cpp re/re_simplifier.cpp re/re_compiler.cpp re/re_analysis.cpp re/re_toolchain.cpp re/re_name_resolve.cpp re/re_parser_pcre.cpp re/re_parser_ere.cpp re/re_parser_bre.cpp)
add_library(CCADT cc/cc_compiler.cpp utf8_encoder.cpp utf16_encoder.cpp UCD/CaseFolding_txt.cpp)
add_library(UCDlib UCD/unicode_set.cpp UCD/ucd_compiler.cpp UCD/PropertyObjects.cpp UCD/resolve_properties.cpp UCD/UnicodeNameData.cpp)

add_library(CodeGen object_cache.cpp ${IDISA_SRC})

# add the executable
target_link_libraries (PabloADT CodeGen ${REQ_LLVM_LIBRARIES})
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

add_executable(icgrep icgrep.cpp toolchain.cpp grep_engine.cpp kernels/pipeline.cpp kernels/scanmatchgen.cpp kernels/cc_kernel.cpp ${PRECOMPILED_FILES})
add_executable(u8u16 u8u16.cpp toolchain.cpp kernels/p2s_kernel.cpp kernels/pipeline.cpp kernels/deletion.cpp)
#add_executable(symtbl symboltable.cpp toolchain.cpp kernels/symboltablepipeline.cpp kernels/stdout_kernel.cpp grep_engine.cpp kernels/pipeline.cpp kernels/scanmatchgen.cpp ${PRECOMPILED_FILES})
add_executable(wc wc.cpp toolchain.cpp kernels/pipeline.cpp)

add_executable(editd editd/editd.cpp editd/pattern_compiler.cpp toolchain.cpp kernels/pipeline.cpp editd/editdscan_kernel.cpp)

IF(ENABLE_PREGENERATED_UCD_FUNCTIONS)
add_dependencies(icgrep run_generate_predefined_ucd_functions)
ENDIF()

IF (PRINT_TIMING_INFORMATION)
    find_package(PAPI REQUIRED)
    include_directories(${PAPI_INCLUDE_DIRS})
    target_link_libraries(icgrep ${PAPI_LIBRARIES})
ENDIF()

target_link_libraries(icgrep ${Boost_LIBRARIES})
#target_link_libraries(symtbl ${Boost_LIBRARIES})
target_link_libraries(u8u16 ${Boost_LIBRARIES})
target_link_libraries(wc ${Boost_LIBRARIES})
target_link_libraries(editd ${Boost_LIBRARIES})

target_link_libraries (icgrep UCDlib PabloADT RegExpCompiler CCADT CodeGen ${REQ_LLVM_LIBRARIES} ${CUDA_LIB})
#target_link_libraries (symtbl UCDlib PabloADT RegExpCompiler CCADT CodeGen ${REQ_LLVM_LIBRARIES} ${CUDA_LIB})
target_link_libraries (u8u16 UCDlib PabloADT RegExpCompiler CCADT CodeGen ${REQ_LLVM_LIBRARIES} ${CUDA_LIB})
target_link_libraries (wc UCDlib PabloADT RegExpCompiler CCADT CodeGen ${REQ_LLVM_LIBRARIES} ${CUDA_LIB})
target_link_libraries (editd UCDlib PabloADT RegExpCompiler CCADT CodeGen ${REQ_LLVM_LIBRARIES} ${CUDA_LIB})

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
SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -g") # -fsanitize=address -fno-omit-frame-pointer

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
