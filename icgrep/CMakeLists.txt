cmake_minimum_required (VERSION 2.8)

project (icgrep)

enable_testing()
# The version number.
set (icgrep_VERSION_MAJOR 1)
set (icgrep_VERSION_MINOR 6)

option(ENABLE_MULTIPLEXING "Compiling the Multiplexing Module")
option(DISABLE_DUAL_ABI "Disable GCC Dual ABI support" OFF)
option(CARRYPACK_MANAGER "Use CarryPack Manager to reduce space required for carries. For testing only." OFF)
option(USE_ADDRESS_SANITIZER "Enables use of address sanitizer in debug mode if available" OFF)
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
set(LLVM_ALL_TARGETS X86)
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

SET(GREP_CORE_SRC kernels/linebreak_kernel.cpp kernels/grep_kernel.cpp kernels/scanmatchgen.cpp )

SET(TOOLCHAIN_SRC toolchain/toolchain.cpp toolchain/pipeline.cpp)

SET(DRIVER_SRC toolchain/driver.cpp toolchain/cpudriver.cpp)
if(ENABLE_CUDA_COMPILE)
SET(DRIVER_SRC ${DRIVER_SRC} toolchain/NVPTXDriver.cpp)
endif()

SET(KERNEL_SRC kernels/attributes.cpp kernels/processing_rate.cpp kernels/interface.cpp kernels/kernel.cpp kernels/multiblock_kernel.cpp kernels/block_kernel.cpp kernels/streamset.cpp kernels/kernel_builder.cpp)
SET(KERNEL_SRC ${KERNEL_SRC} kernels/source_kernel.cpp kernels/s2p_kernel.cpp kernels/deletion.cpp kernels/swizzle.cpp kernels/p2s_kernel.cpp kernels/stdout_kernel.cpp kernels/hex_convert.cpp)

SET(IDISA_SRC IR_Gen/CBuilder.cpp IR_Gen/idisa_builder.cpp IR_Gen/idisa_avx_builder.cpp IR_Gen/idisa_i64_builder.cpp IR_Gen/idisa_sse_builder.cpp)
SET(IDISA_SRC ${IDISA_SRC} IR_Gen/idisa_target.cpp)
if(ENABLE_CUDA_COMPILE)
SET(IDISA_SRC ${IDISA_SRC} IR_Gen/idisa_nvptx_builder.cpp)
endif()

SET(PABLO_SRC pablo/pabloAST.cpp pablo/branch.cpp pablo/codegenstate.cpp pablo/builder.cpp pablo/symbol_generator.cpp pablo/printer_pablos.cpp pablo/pablo_toolchain.cpp)
SET(PABLO_SRC ${PABLO_SRC} pablo/pablo_kernel.cpp pablo/pablo_compiler.cpp)
IF (CARRYPACK_MANAGER)
SET(PABLO_SRC ${PABLO_SRC} pablo/carrypack_manager.cpp)
ELSE()
SET(PABLO_SRC ${PABLO_SRC} pablo/carry_manager.cpp)
ENDIF()
SET(PABLO_SRC ${PABLO_SRC} pablo/analysis/pabloverifier.cpp)
SET(PABLO_SRC ${PABLO_SRC} pablo/optimizers/pablo_simplifier.cpp pablo/optimizers/codemotionpass.cpp pablo/optimizers/distributivepass.cpp pablo/optimizers/schedulingprepass.cpp)
SET(PABLO_SRC ${PABLO_SRC} pablo/passes/flattenif.cpp)
IF(ENABLE_MULTIPLEXING)
SET(PABLO_SRC ${PABLO_SRC} pablo/optimizers/pablo_automultiplexing.cpp)
ENDIF()

SET (RE_PARSERS_SRC re/parsers/PCRE_parser.cpp re/parsers/ERE_parser.cpp re/parsers/BRE_parser.cpp re/parsers/Prosite_parser.cpp re/parsers/fixed_string_parser.cpp re/parsers/GLOB_parser.cpp re/parsers/parser.cpp)

add_library(CodeGen ${TOOLCHAIN_SRC} ${DRIVER_SRC} ${OBJECT_CACHE_SRC} ${KERNEL_SRC} ${IDISA_SRC})
add_library(PabloADT ${PABLO_SRC})
# CCADT is the core library for representing and compiling character classes
add_library(CCADT re/re_re.cpp re/re_cc.cpp cc/cc_compiler.cpp cc/alphabet.cpp cc/multiplex_CCs.cpp UCD/unicode_set.cpp)
# RegExpADT is the core library for representing, parsing and printing regular expressions
add_library(RegExpADT re/re_rep.cpp re/re_diff.cpp re/re_intersect.cpp re/re_range.cpp re/re_assertion.cpp re/printer_re.cpp re/re_utility.cpp)
add_library(RegExpCompiler ${RE_PARSERS_SRC} re/casing.cpp re/to_utf8.cpp re/re_memoizer.cpp re/re_nullable.cpp re/re_simplifier.cpp re/re_star_normal.cpp re/re_minimizer.cpp re/re_local.cpp re/re_compiler.cpp re/re_analysis.cpp re/re_toolchain.cpp re/re_name_resolve.cpp re/re_name_gather.cpp re/collect_ccs.cpp re/re_multiplex.cpp re/re_utility.cpp re/grapheme_clusters.cpp re/exclude_CC.cpp re/replaceCC.cpp)
add_library(UCDlib UCD/CaseFolding.cpp utf8_encoder.cpp utf16_encoder.cpp UCD/ucd_compiler.cpp UCD/PropertyObjects.cpp UCD/resolve_properties.cpp)
add_library(GrepEngine  ${GREP_CORE_SRC} grep/grep_engine.cpp kernels/cc_kernel.cpp kernels/cc_scan_kernel.cpp kernels/charclasses.cpp kernels/streams_merge.cpp kernels/until_n.cpp kernels/UCD_property_kernel.cpp kernels/grapheme_kernel.cpp)
add_library(LZ4_Lib lz4/lz4_frame_decoder.cpp kernels/cc_kernel.cpp kernels/pdep_kernel.cpp lz4/lz4_decompression_generator.cpp kernels/lz4/lz4_block_decoder.cpp kernels/bitstream_pdep_kernel.cpp kernels/bitstream_gather_pdep_kernel.cpp kernels/swizzled_multiple_pdep_kernel.cpp kernels/fake_stream_generating_kernel.cpp kernels/lz4/decompression/lz4_bytestream_decompression.cpp kernels/lz4/decompression/lz4_swizzled_decompression.cpp kernels/lz4/decompression/lz4_sequential_decompression_base.cpp kernels/lz4/decompression/lz4_bitstream_decompression.cpp kernels/lz4/decompression/lz4_twist_decompression.cpp kernels/lz4/twist_kernel.cpp kernels/lz4/untwist_kernel.cpp lz4/lz4_base_generator.cpp kernels/lz4/lz4_match_detector.cpp kernels/lz4/lz4_match_detector.h kernels/lz4/lz4_not_kernel.cpp kernels/lz4/lz4_not_kernel.h)

# force the compiler to compile the object cache to ensure that the versioning information is up to date
add_custom_target(RecompileObjectCache ALL touch ${CMAKE_CURRENT_SOURCE_DIR}/${OBJECT_CACHE_SRC})
add_dependencies(CodeGen RecompileObjectCache)

# add the executable
target_link_libraries (CodeGen ${REQ_LLVM_LIBRARIES})
target_link_libraries (PabloADT CodeGen)
target_link_libraries (CCADT PabloADT)
target_link_libraries (RegExpADT PabloADT CCADT)
target_link_libraries (UCDlib GrepEngine RegExpCompiler RegExpADT PabloADT CCADT)
target_link_libraries (RegExpCompiler UCDlib CCADT RegExpADT)
target_link_libraries (GrepEngine RegExpCompiler CodeGen)

add_executable(icgrep icgrep.cpp util/file_select.cpp grep_interface.cpp)
add_executable(u8u16 u8u16.cpp)
add_executable(u32u8 u32u8.cpp kernels/pdep_kernel.cpp)
add_executable(base64 base64.cpp kernels/radix64.cpp)
add_executable(wc wc.cpp util/file_select.cpp)
add_executable(editd editd/editd.cpp editd/pattern_compiler.cpp editd/editdscan_kernel.cpp editd/editd_gpu_kernel.cpp editd/editd_cpu_kernel.cpp kernels/streams_merge.cpp kernels/cc_kernel.cpp)
add_executable(lz4d lz4d.cpp lz4/lz4_frame_decoder.cpp kernels/cc_kernel.cpp kernels/lz4/lz4_index_decoder.cpp kernels/lz4/lz4_bytestream_decoder.cpp)
add_executable(core combine/core.cpp combine/regexGen.cpp combine/stringGen.cpp combine/propGen.cpp combine/icgrep-test/icgrep-test.cpp grep_interface.cpp grep/grep_engine.cpp kernels/scanmatchgen.cpp kernels/u8u32_kernel.cpp kernels/delmask_kernel.cpp kernels/cc_kernel.cpp kernels/cc_scan_kernel.cpp kernels/charclasses.cpp kernels/linebreak_kernel.cpp kernels/streams_merge.cpp kernels/grep_kernel.cpp kernels/until_n.cpp)
add_executable(character_deletion character_deletion.cpp kernels/cc_kernel.cpp)
add_executable(character_deposit character_deposit.cpp kernels/cc_kernel.cpp kernels/pdep_kernel.cpp kernels/bitstream_pdep_kernel.cpp)
add_executable(lz4_decoder lz4_decoder.cpp)
add_executable(lz4_grep grep_interface.cpp util/file_select.cpp lz4_grep.cpp lz4/grep/lz4_grep_base_generator.cpp lz4/grep/lz4_grep_bytestream_generator.cpp lz4/grep/lz4_grep_bytestream_generator.h lz4/grep/lz4_grep_bitstream_generator.cpp lz4/grep/lz4_grep_bitstream_generator.h lz4/grep/lz4_grep_swizzle_generator.cpp lz4/grep/lz4_grep_swizzle_generator.h)
add_executable(idisa_test idisa_test.cpp)


#Exclude CoRE from default build.
set_target_properties(core PROPERTIES EXCLUDE_FROM_ALL 1)

target_link_libraries (icgrep GrepEngine UCDlib PabloADT RegExpCompiler CCADT CodeGen ${REQ_LLVM_LIBRARIES} ${Boost_LIBRARIES} ${CUDA_LIB})
target_link_libraries (u8u16 PabloADT CCADT CodeGen ${REQ_LLVM_LIBRARIES} ${Boost_LIBRARIES} ${CUDA_LIB})
target_link_libraries (u32u8 PabloADT CCADT CodeGen ${REQ_LLVM_LIBRARIES} ${Boost_LIBRARIES} ${CUDA_LIB})
target_link_libraries (base64 PabloADT CCADT CodeGen ${REQ_LLVM_LIBRARIES} ${Boost_LIBRARIES} ${CUDA_LIB})
target_link_libraries (wc GrepEngine UCDlib PabloADT RegExpCompiler  CCADT CodeGen ${REQ_LLVM_LIBRARIES} ${Boost_LIBRARIES} ${CUDA_LIB})
target_link_libraries (editd PabloADT CCADT CodeGen ${REQ_LLVM_LIBRARIES} ${Boost_LIBRARIES} ${CUDA_LIB})
target_link_libraries (lz4d PabloADT CCADT CodeGen ${REQ_LLVM_LIBRARIES} ${Boost_LIBRARIES} ${CUDA_LIB})
target_link_libraries (core RegExpCompiler ${REQ_LLVM_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries (character_deletion PabloADT RegExpCompiler CCADT CodeGen ${REQ_LLVM_LIBRARIES} ${Boost_LIBRARIES} ${CUDA_LIB})
target_link_libraries (character_deposit PabloADT RegExpCompiler CCADT CodeGen ${REQ_LLVM_LIBRARIES} ${Boost_LIBRARIES} ${CUDA_LIB})
target_link_libraries (lz4_decoder LZ4_Lib PabloADT CCADT CodeGen ${REQ_LLVM_LIBRARIES} ${Boost_LIBRARIES} ${CUDA_LIB})
target_link_libraries (lz4_grep LZ4_Lib UCDlib PabloADT RegExpCompiler CodeGen CCADT CodeGen ${REQ_LLVM_LIBRARIES} ${Boost_LIBRARIES} ${CUDA_LIB})
target_link_libraries (idisa_test CodeGen PabloADT ${REQ_LLVM_LIBRARIES} ${Boost_LIBRARIES} ${CUDA_LIB})


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
MATH(EXPR LLVM_VERSION_INTEGER "${LLVM_VERSION_MAJOR} * 10000 + ${LLVM_VERSION_MINOR} * 100 + ${LLVM_VERSION_PATCH}")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DLLVM_VERSION_INTEGER=${LLVM_VERSION_INTEGER}")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DLLVM_VERSION_major=${LLVM_VERSION_MAJOR} -DLLVM_VERSION_MINOR=${LLVM_VERSION_MINOR}")
message(STATUS "LLVM Version Integer: ${LLVM_VERSION_INTEGER}")

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DPARABIX_VERSION='\"${Parabix_REVISION}\"'")
IF (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fuse-ld=gold")
ENDIF()

SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_RELEASE} -O3 -DNDEBUG")

SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_DEBUG} -O1 -g -fno-omit-frame-pointer -fno-optimize-sibling-calls")

UNSET(HAS_ADDRESS_SANITIZER)
IF (USE_ADDRESS_SANITIZER)
CHECK_INCLUDE_FILE_CXX("sanitizer/asan_interface.h" HAS_ADDRESS_SANITIZER)
IF (HAS_ADDRESS_SANITIZER)
SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DHAS_ADDRESS_SANITIZER -fsanitize=address")
ENDIF()
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
  NAME u32u8_test
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/../QA/
  COMMAND ./test_u32u8 "${CMAKE_BINARY_DIR}/u32u8")

#add_test(
#  NAME lz4d_test
#  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/../QA/lz4d
#  COMMAND ./run_all ${CMAKE_BINARY_DIR}/lz4d)

add_test(
  NAME editd_test
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/../QA/editd
  COMMAND python run_all.py -e ${CMAKE_BINARY_DIR})

add_test(
  NAME base64_test
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/../QA/base64
  COMMAND python base64test.py ${CMAKE_BINARY_DIR}/base64)

add_test(
  NAME pdep64_test
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/../QA/IDISA_test
  COMMAND ${CMAKE_BINARY_DIR}/idisa_test simd_pdep 64 randhex65536a randhex65536b)

#add_test(
#  NAME character_deletion_test
#  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/../QA/character_deletion
#  COMMAND python character_deletion_test.py ${CMAKE_BINARY_DIR}/character_deletion
#)

#add_test(
#  NAME character_deposit_test
#  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/../QA/character_deposit
#  COMMAND python character_deposit_test.py ${CMAKE_BINARY_DIR}/character_deposit
#)

#add_test(
#  NAME lz4d_ext_dep_test
#  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/../QA/lz4_decoder
#  COMMAND python main.py ${CMAKE_BINARY_DIR}/lz4_decoder
#)

SET_PROPERTY(TEST greptest PROPERTY TIMEOUT 400)
SET_PROPERTY(TEST proptest PROPERTY TIMEOUT 1500)
SET_PROPERTY(TEST abc_test PROPERTY TIMEOUT 100)
SET_PROPERTY(TEST u8u16_test editd_test base64_test PROPERTY TIMEOUT 40)


add_custom_target (greptest
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/../QA
  COMMAND python greptest.py "${CMAKE_BINARY_DIR}/icgrep")

add_custom_target (proptest
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/../QA
  COMMAND python greptest.py -d ${PROJECT_SOURCE_DIR}/../QA -t ${PROJECT_SOURCE_DIR}/../QA/proptest.xml "${CMAKE_BINARY_DIR}/icgrep")

add_custom_target (u8u16_test
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/../QA/u8u16
  COMMAND ./run_all "${CMAKE_BINARY_DIR}/u8u16 -segment-size=16 -enable-segment-pipeline-parallel")

add_custom_target (u32u8_test
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/../QA
  COMMAND ./test_u32u8 "${CMAKE_BINARY_DIR}/u32u8 -EnableAsserts -VerifyIR")

add_custom_target (lz4d_test
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/../QA/lz4d
  COMMAND ./run_all "${CMAKE_BINARY_DIR}/lz4d")

add_custom_target (editd_test
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/../QA/editd
  COMMAND python run_all.py -e ${CMAKE_BINARY_DIR})

add_custom_target (base64_test
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/../QA/base64
  COMMAND python base64test.py "${CMAKE_BINARY_DIR}/base64")

#add_custom_target(character_deletion_test
#  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/../QA/character_deletion
#  COMMAND python character_deletion_test.py ${CMAKE_BINARY_DIR}/character_deletion
#)

#add_custom_target(character_deposit_test
#  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/../QA/character_deposit
#  COMMAND python character_deposit_test.py ${CMAKE_BINARY_DIR}/character_deposit
#)

#add_custom_target(lz4d_ext_dep_test
#  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/../QA/lz4_decoder
#  COMMAND python main.py ${CMAKE_BINARY_DIR}/lz4_decoder
#)

add_custom_target (abc_gen
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/../QA
  COMMAND python abc_testgen.py)
  
add_custom_target (abc_test
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/../QA
  COMMAND ./run_abc "${CMAKE_BINARY_DIR}/icgrep")

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


