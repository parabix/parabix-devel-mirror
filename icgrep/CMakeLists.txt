cmake_minimum_required (VERSION 2.8)

project (icgrep)

enable_testing()
# The version number.
set (icgrep_VERSION_MAJOR 1)
set (icgrep_VERSION_MINOR 6)

option(ENABLE_MULTIPLEXING "Compiling the Multiplexing Module")
option(DISABLE_DUAL_ABI "Disable GCC Dual ABI support" OFF)
option(CARRYPACK_MANAGER "Use CarryPack Manager to reduce space required for carries. For testing only." OFF)
find_package(LLVM REQUIRED CONFIG)

message(STATUS "Found LLVM ${LLVM_PACKAGE_VERSION} in: ${LLVM_DIR}")

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${CMAKE_CURRENT_SOURCE_DIR}/cmake") # ensure custom FindXYZ.cmake files are found

set (Parabix_REVISION "svn")
include(VersionFromVCS)
add_version_info_from_vcs(Parabix_REVISION)
message(STATUS "Parabix_REVISION: ${Parabix_REVISION}")


# Now set the header and library paths:
include_directories( ${LLVM_INCLUDE_DIRS} )
link_directories( ${LLVM_LIBRARY_DIRS} )
add_definitions( ${LLVM_DEFINITIONS} )

# include(FindCUDA) # https://cmake.org/cmake/help/v3.0/module/FindCUDA.html

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

SET(OBJECT_CACHE_SRC toolchain/object_cache.cpp)

SET(GREP_CORE_SRC toolchain/grep_pipeline.cpp kernels/linebreak_kernel.cpp kernels/grep_kernel.cpp kernels/scanmatchgen.cpp )

SET(TOOLCHAIN_SRC toolchain/toolchain.cpp toolchain/pipeline.cpp)

SET(DRIVER_SRC toolchain/driver.cpp toolchain/cpudriver.cpp toolchain/NVPTXDriver.cpp)

SET(KERNEL_SRC kernels/kernel.cpp kernels/streamset.cpp kernels/interface.cpp kernels/kernel_builder.cpp)
SET(KERNEL_SRC ${KERNEL_SRC} kernels/source_kernel.cpp kernels/s2p_kernel.cpp kernels/deletion.cpp kernels/swizzle.cpp kernels/p2s_kernel.cpp kernels/stdout_kernel.cpp)

SET(IDISA_SRC IR_Gen/CBuilder.cpp IR_Gen/idisa_builder.cpp IR_Gen/idisa_avx_builder.cpp IR_Gen/idisa_i64_builder.cpp IR_Gen/idisa_sse_builder.cpp IR_Gen/idisa_nvptx_builder.cpp)
SET(IDISA_SRC ${IDISA_SRC} IR_Gen/idisa_target.cpp)

SET(PABLO_SRC pablo/pabloAST.cpp pablo/branch.cpp pablo/codegenstate.cpp pablo/builder.cpp pablo/symbol_generator.cpp pablo/printer_pablos.cpp pablo/pablo_toolchain.cpp)
SET(PABLO_SRC ${PABLO_SRC} pablo/pablo_kernel.cpp pablo/pablo_compiler.cpp)
IF (CARRYPACK_MANAGER)
SET(PABLO_SRC ${PABLO_SRC} pablo/carrypack_manager.cpp)
ELSE()
SET(PABLO_SRC ${PABLO_SRC} pablo/carry_manager.cpp)
ENDIF()
SET(PABLO_SRC ${PABLO_SRC} pablo/analysis/pabloverifier.cpp)
SET(PABLO_SRC ${PABLO_SRC} pablo/passes/ssapass.cpp)
SET(PABLO_SRC ${PABLO_SRC} pablo/optimizers/pablo_simplifier.cpp pablo/optimizers/codemotionpass.cpp pablo/optimizers/distributivepass.cpp pablo/optimizers/schedulingprepass.cpp)
SET(PABLO_SRC ${PABLO_SRC} pablo/passes/flattenif.cpp)
IF(ENABLE_MULTIPLEXING)
SET(PABLO_SRC ${PABLO_SRC} pablo/optimizers/pablo_automultiplexing.cpp)
ENDIF()

add_library(CodeGen ${TOOLCHAIN_SRC} ${DRIVER_SRC} ${OBJECT_CACHE_SRC} ${KERNEL_SRC} ${IDISA_SRC})
add_library(PabloADT ${PABLO_SRC})
add_library(RegExpADT re/re_re.cpp re/re_cc.cpp re/re_rep.cpp re/re_diff.cpp re/re_intersect.cpp re/printer_re.cpp)
add_library(RegExpCompiler re/re_parser.cpp re/re_memoizer.cpp re/re_nullable.cpp re/re_simplifier.cpp re/re_star_normal.cpp re/re_minimizer.cpp re/re_local.cpp re/re_compiler.cpp re/re_analysis.cpp re/re_toolchain.cpp re/re_name_resolve.cpp re/re_name_gather.cpp re/re_collect_unicodesets.cpp re/re_multiplex.cpp re/re_parser_pcre.cpp re/re_parser_ere.cpp re/re_parser_bre.cpp re/re_parser_prosite.cpp re/re_utility.cpp ${GREP_CORE_SRC})
add_library(CCADT cc/cc_compiler.cpp utf8_encoder.cpp utf16_encoder.cpp UCD/CaseFolding.cpp cc/alphabet.cpp cc/multiplex_CCs.cpp)
add_library(UCDlib UCD/unicode_set.cpp UCD/ucd_compiler.cpp UCD/PropertyObjects.cpp UCD/resolve_properties.cpp)

# force the compiler to compile the object cache to ensure that the versioning information is up to date
add_custom_target(RecompileObjectCache ALL touch ${CMAKE_CURRENT_SOURCE_DIR}/${OBJECT_CACHE_SRC})
add_dependencies(CodeGen RecompileObjectCache)

# add the executable
target_link_libraries (CodeGen ${REQ_LLVM_LIBRARIES})
target_link_libraries (PabloADT CodeGen)
target_link_libraries (CCADT PabloADT)
target_link_libraries (UCDlib RegExpCompiler RegExpADT PabloADT CCADT)
target_link_libraries (RegExpADT PabloADT CCADT UCDlib)
target_link_libraries (RegExpCompiler UCDlib RegExpADT)

add_executable(icgrep icgrep.cpp grep_interface.cpp grep_engine.cpp kernels/u8u32_kernel.cpp kernels/delmask_kernel.cpp kernels/cc_kernel.cpp kernels/cc_scan_kernel.cpp kernels/charclasses.cpp kernels/linebreak_kernel.cpp kernels/streams_merge.cpp kernels/grep_kernel.cpp kernels/until_n.cpp)
add_executable(u8u16 u8u16.cpp)
add_executable(base64 base64.cpp kernels/radix64.cpp)
add_executable(wc wc.cpp)
add_executable(editd editd/editd.cpp editd/pattern_compiler.cpp editd/editdscan_kernel.cpp editd/editd_gpu_kernel.cpp editd/editd_cpu_kernel.cpp kernels/streams_merge.cpp kernels/cc_kernel.cpp)
add_executable(array-test array-test.cpp kernels/alignedprint.cpp)
add_executable(lz4d lz4d.cpp lz4FrameDecoder.cpp kernels/cc_kernel.cpp kernels/lz4_index_decoder.cpp kernels/lz4_bytestream_decoder.cpp)
add_executable(core combine/core.cpp combine/regexGen.cpp combine/stringGen.cpp combine/icgrep-test/icgrep-test.cpp combine/pugixml/src/pugixml.cpp grep_interface.cpp grep_engine.cpp kernels/scanmatchgen.cpp kernels/u8u32_kernel.cpp kernels/delmask_kernel.cpp kernels/cc_kernel.cpp kernels/cc_scan_kernel.cpp kernels/charclasses.cpp kernels/linebreak_kernel.cpp kernels/streams_merge.cpp kernels/grep_kernel.cpp kernels/until_n.cpp)


target_link_libraries (icgrep UCDlib PabloADT RegExpCompiler CCADT CodeGen ${REQ_LLVM_LIBRARIES} ${Boost_LIBRARIES} ${CUDA_LIB})
target_link_libraries (u8u16 UCDlib PabloADT RegExpCompiler CCADT CodeGen ${REQ_LLVM_LIBRARIES} ${Boost_LIBRARIES} ${CUDA_LIB})
target_link_libraries (base64 UCDlib PabloADT RegExpCompiler CCADT CodeGen ${REQ_LLVM_LIBRARIES} ${Boost_LIBRARIES} ${CUDA_LIB})
target_link_libraries (wc UCDlib PabloADT RegExpCompiler CCADT CodeGen ${REQ_LLVM_LIBRARIES} ${Boost_LIBRARIES} ${CUDA_LIB})
target_link_libraries (editd UCDlib PabloADT RegExpCompiler CCADT CodeGen ${REQ_LLVM_LIBRARIES} ${Boost_LIBRARIES} ${CUDA_LIB})
target_link_libraries (array-test PabloADT CodeGen ${REQ_LLVM_LIBRARIES} ${Boost_LIBRARIES} ${CUDA_LIB})
target_link_libraries (lz4d PabloADT UCDlib RegExpCompiler CodeGen ${REQ_LLVM_LIBRARIES} ${Boost_LIBRARIES} ${CUDA_LIB})
target_link_libraries (core RegExpCompiler ${REQ_LLVM_LIBRARIES} ${Boost_LIBRARIES})

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

UNSET(HAS_EXECINFO CACHE)
UNSET(HAS_MACH_VM_TYPES CACHE)
include(CheckIncludeFileCXX)
CHECK_INCLUDE_FILE_CXX(mach/vm_types.h HAS_MACH_VM_TYPES)
IF (HAS_MACH_VM_TYPES)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHAS_MACH_VM_TYPES")
ELSE()
  UNSET(LIBUNWIND_FOUND CACHE)
  find_package(Libunwind)
  IF (LIBUNWIND_FOUND)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHAS_LIBUNWIND")
    include_directories(${LIBUNWIND_INCLUDE_DIR})
    target_link_libraries(CodeGen ${LIBUNWIND_LIBRARIES})
  ELSE()
    CHECK_INCLUDE_FILE_CXX(execinfo.h HAS_EXECINFO)
    IF (HAS_EXECINFO)
       SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHAS_EXECINFO")
    ENDIF()
  ENDIF()
ENDIF()

IF (CARRYPACK_MANAGER)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_CARRYPACK_MANAGER")
ENDIF()

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

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DPARABIX_VERSION='\"${Parabix_REVISION}\"'")
IF (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fuse-ld=gold")
ENDIF()

SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_RELEASE} -O3 -DNDEBUG")

SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_DEBUG} -O1 -g -fno-omit-frame-pointer -fno-optimize-sibling-calls")

UNSET(HAS_ADDRESS_SANITIZER)
CHECK_INCLUDE_FILE_CXX("sanitizer/asan_interface.h" HAS_ADDRESS_SANITIZER)
IF (HAS_ADDRESS_SANITIZER)
SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DHAS_ADDRESS_SANITIZER -fsanitize=address")
ENDIF()


add_test(
  NAME greptest
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/../QA
  COMMAND python greptest.py ${CMAKE_BINARY_DIR}/icgrep)

add_test(
  NAME proptest
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/../QA
  COMMAND python greptest.py -d ${PROJECT_SOURCE_DIR}/../QA -t ${PROJECT_SOURCE_DIR}/../QA/proptest.xml ${CMAKE_BINARY_DIR}/icgrep)

add_test(
  NAME abc_test
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/../QA
  COMMAND ./run_abc ${CMAKE_BINARY_DIR}/icgrep)

add_test(
  NAME u8u16_test
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/../QA/u8u16
  COMMAND ./run_all "${CMAKE_BINARY_DIR}/u8u16 -segment-size=16 -enable-segment-pipeline-parallel")

add_test(
  NAME lz4d_test
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/../QA/lz4d
  COMMAND ./run_all ${CMAKE_BINARY_DIR}/lz4d)

add_test(
  NAME editd_test
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/../QA/editd
  COMMAND python run_all.py -e ${CMAKE_BINARY_DIR})

add_test(
  NAME base64_test
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/../QA/base64
  COMMAND python base64test.py ${CMAKE_BINARY_DIR}/base64)

SET_PROPERTY(TEST greptest PROPERTY TIMEOUT 400)
SET_PROPERTY(TEST proptest PROPERTY TIMEOUT 1500)
SET_PROPERTY(TEST abc_test PROPERTY TIMEOUT 100)
SET_PROPERTY(TEST u8u16_test lz4d_test editd_test base64_test PROPERTY TIMEOUT 40)


add_custom_target (greptest
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/../QA
  COMMAND python greptest.py "${CMAKE_BINARY_DIR}/icgrep -ea")

add_custom_target (proptest
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/../QA
  COMMAND python greptest.py -d ${PROJECT_SOURCE_DIR}/../QA -t ${PROJECT_SOURCE_DIR}/../QA/proptest.xml "${CMAKE_BINARY_DIR}/icgrep -ea")

add_custom_target (u8u16_test
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/../QA/u8u16
  COMMAND ./run_all "${CMAKE_BINARY_DIR}/u8u16 -ea -segment-size=16 -enable-segment-pipeline-parallel")

add_custom_target (lz4d_test
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/../QA/lz4d
  COMMAND ./run_all "${CMAKE_BINARY_DIR}/lz4d -ea")

add_custom_target (editd_test
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/../QA/editd
  COMMAND python run_all.py -e ${CMAKE_BINARY_DIR})

add_custom_target (base64_test
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/../QA/base64
  COMMAND python base64test.py "${CMAKE_BINARY_DIR}/base64 -ea")

add_custom_target (abc_gen
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/../QA
  COMMAND python abc_testgen.py)
  
add_custom_target (abc_test
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/../QA
  COMMAND ./run_abc "${CMAKE_BINARY_DIR}/icgrep -ea")

add_custom_target (check
  COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure
  DEPENDS icgrep abc_gen)

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

add_custom_target (clear_cache
  COMMAND rm ~/.cache/parabix/*)


