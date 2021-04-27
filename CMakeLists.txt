# Copyright (c) 2019 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# IMPORTANT:
#
# If an explicit link with LLVM is needed, link with the toolchain library indead.
# See lib/toolchain/CMakeLists.txt for more info.
#
# i.e.,
#   DO:
#       target_link_libraries(<mylib> toolchain)
#
#   DON'T:
#       target_link_libraries(<mylib> ${REQ_LLVM_LIBRARIES})

cmake_minimum_required(VERSION 2.8)

project(parabix)
enable_testing()

###   Build Options   ###

option(DISABLE_DUAL_ABI "Disable GCC Dual ABI support" OFF)
option(USE_ADDRESS_SANITIZER "Enables use of address sanitizer in debug mode if available" OFF)
option(LLVM_PATH "LLVM root directory")
option(GENERATE_TOOLS "Controls whether or not it generate tools" ON)

# Build Artifact Configuration
set(ARTIFACT_MODE "Multiple" CACHE STRING "Build framework into a single artifact (Single) or multiple artifacts (Multiple)")
set(ARTIFACT_TYPE "Static" CACHE STRING "Build frameword as shared (Shared) or static (Static) libraries")

# Validate framework build style
set(VALID_BUILD_MODE "Single" "Multiple")
set(VALID_BUILD_TYPE "Shared" "Static")
list(FIND VALID_BUILD_MODE ${ARTIFACT_MODE} __am_index)
if(__am_index EQUAL -1)
    message(FATAL_ERROR "ARTIFACT_MODE must be either Single or Multiple")
endif()
list(FIND VALID_BUILD_TYPE ${ARTIFACT_TYPE} __at_index)
if(__at_index EQUAL -1)
    message(FATAL_ERROR "ARTIFACT_TYPE must be either Shared or Static")
endif()

# Target for single shared library
set(PARABIX_MONO_TARGET parabix)


# Ensure custom FindXYZ.cmake files are found
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${CMAKE_CURRENT_SOURCE_DIR}/cmake")

###   Versioning   ###
set(Parabix_REVISION "svn")
include(VersionFromVCS)
add_version_info_from_vcs(Parabix_REVISION)
message(STATUS "Parabix Revision: ${Parabix_REVISION}")


###   Import Dependency: LLVM   ###
if (LLVM_PATH)
find_package(LLVM REQUIRED CONFIG PATHS ${LLVM_PATH} NO_DEFAULT_PATH)
else()
find_package(LLVM REQUIRED CONFIG)
endif()
message(STATUS "Found LLVM ${LLVM_PACKAGE_VERSION} in: ${LLVM_DIR}")
set(LLVM_ALL_TARGETS X86)
llvm_map_components_to_libnames(REQ_LLVM_LIBRARIES ${LLVM_ALL_TARGETS} mcjit native IRReader Linker)
include_directories(${LLVM_INCLUDE_DIRS})
link_directories(${LLVM_LIBRARY_DIRS})
add_definitions(${LLVM_DEFINITIONS})
math(EXPR LLVM_VERSION_INTEGER "${LLVM_VERSION_MAJOR} * 10000 + ${LLVM_VERSION_MINOR} * 100 + ${LLVM_VERSION_PATCH}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DLLVM_VERSION_INTEGER=${LLVM_VERSION_INTEGER}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DLLVM_VERSION_major=${LLVM_VERSION_MAJOR} -DLLVM_VERSION_MINOR=${LLVM_VERSION_MINOR}")
message(STATUS "LLVM Version Integer: ${LLVM_VERSION_INTEGER}")



###   Import Dependency: Boost   ###

set(Boost_USE_STATIC_LIBS ${USE_STATIC_LIBS})
set(Boost_USE_MULTITHREADED OFF)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost 1.61 REQUIRED COMPONENTS system filesystem iostreams)
message(STATUS "Found Boost_LIBRARY_DIRS: ${Boost_LIBRARY_DIRS}")
include_directories("${Boost_INCLUDE_DIRS}")
link_directories(${Boost_LIBRARY_DIRS})



###   Import Dependency: Z3   ###

find_package(Z3 REQUIRED) # libz3-dev
message(STATUS "Found Z3 ${Z3_VERSION_STRING} in: ${Z3_INCLUDE_DIR}")
include_directories(${Z3_INCLUDE_DIRS})
string(REGEX REPLACE "([0-9]+)[.].*" "\\1" Z3_VERSION_MAJOR "${Z3_VERSION_STRING}")
string(REGEX REPLACE "[0-9]+[.]([0-9]+).*" "\\1" Z3_VERSION_MINOR "${Z3_VERSION_STRING}")
string(REGEX REPLACE "[0-9]+[.][0-9]+[.]([0-9]+).*" "\\1" Z3_VERSION_PATCH "${Z3_VERSION_STRING}")
math(EXPR Z3_VERSION_INTEGER "${Z3_VERSION_MAJOR} * 10000 + ${Z3_VERSION_MINOR} * 100 + ${Z3_VERSION_PATCH}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DZ3_VERSION_INTEGER=${Z3_VERSION_INTEGER}")


###   Project Configuration   ###

# Require that compilers support all features of C11 and C++11.
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 11)

# Define `CXX_STANDARD` to enable conditional compilation based on standard version.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DCXX_STANDARD=${CMAKE_CXX_STANDARD}")

# Add include directories.
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Place compiled executables in build/bin.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin)

# Parabix Object Cache default
set(PARABIX_OBJECT_CACHE "$ENV{HOME}/.cache/parabix/")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DPARABIX_OBJECT_CACHE='\"${PARABIX_OBJECT_CACHE}\"'")

# Use @rpath for dylibs on macOS
set(CMAKE_MACOSX_RPATH ON)

# Enable all warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic")
if (${LLVM_PACKAGE_VERSION} VERSION_LESS "6.0")
# LLVM 5 and earlier generate many warnings about deprecated declarations; suppress these.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")
endif()

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-fvisibility-inlines-hidden" SUPPORTS_FVISIBILITY_INLINES_HIDDEN_FLAG)
if(SUPPORTS_FVISIBILITY_INLINES_HIDDEN_FLAG)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fvisibility-inlines-hidden")
endif()

# no pie disables the generation of position-independent executables, which is a default security feature of newer compiles
# that prevents addr2line from being able to resolve which line corresponds to the code at run time. This in turn prevents
# CreateAssert from being able to provide the compilation call stack for each JIT'ed assertion error.

UNSET(HAS_EXECINFO CACHE)
UNSET(HAS_MACH_VM_TYPES CACHE)

# Disable RunTime Type Information
if(MSVC) # using Visual Studio C++
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /GR-")
else() # using Clang, GCC, Intel C++, etc
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-rtti")
endif()

# Disable Dual ABI support
if(DISABLE_DUAL_ABI)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_GLIBCXX_USE_CXX11_ABI=0")
endif()

set(CMAKE_REQUIRED_FLAGS)

# Define parabix version
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DPARABIX_VERSION='\"${Parabix_REVISION}\"'")

# Use gold linker on Linux:gcc
if (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fuse-ld=gold")
    endif()
endif()


# Use address sanitizer if desiered
unset(HAS_ADDRESS_SANITIZER)
if (USE_ADDRESS_SANITIZER)
    #CHECK_INCLUDE_FILE_CXX("sanitizer/asan_interface.h" HAS_ADDRESS_SANITIZER)
    #if (HAS_ADDRESS_SANITIZER)
        set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DHAS_ADDRESS_SANITIZER -fsanitize=address")
    #endif()
endif()

###   Functions   ###

# Sets the output name for a given target.
# Naming scheme is "parabix_<target>" with any '.' replaced with a '_'
function(parabix_set_library_name NAME)
    string(REPLACE "." "_" DELIMITED_NAME ${NAME})
    string(CONCAT LIBRARY_NAME "parabix_" ${DELIMITED_NAME})
    set_target_properties(${NAME} PROPERTIES OUTPUT_NAME ${LIBRARY_NAME})
endfunction(parabix_set_library_name)

include(CMakeParseArguments)

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    include(CheckIncludeFileCXX)
    CHECK_INCLUDE_FILE_CXX(execinfo.h HAS_EXECINFO)
    if (HAS_EXECINFO)
        ###   Import Dependency: DWARF   ###
        #find_package(LibDwarf) # libelf-dev libdwarf-dev
        #if (LIBDWARF_FOUND)
        #    include_directories(${LIBDWARF_INCLUDE_DIRS})
        #    link_directories(${LIBDWARF_LIBRARIES})
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DENABLE_ASSERTION_TRACE")
        #endif()
    endif()
endif()

# Release & Debug flags

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_RELEASE}")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_DEBUG} -O1 -g -g2 -ggdb -fvisibility=default -fno-omit-frame-pointer -fno-optimize-sibling-calls")

function(parabix_add_module)
    cmake_parse_arguments(
        ARG             # resultant argument prefix
        "USES_Z3"       # boolean args
        "NAME"          # mono-valued arguments:
                        #   NAME: target name
        "SRC;DEPS"      # multi-valued arguments:
                        #   SRC:  source file list
                        #   DEPS: target dependencies
        ${ARGN}         # arguments to parse
    )
    if(NOT ARG_NAME)
        message(FATAL_ERROR "parabix_add_module: A target name is required")
    endif()
    if(ARTIFACT_MODE STREQUAL "Single")
        foreach(SRC ${ARG_SRC})
            target_sources(${PARABIX_MONO_TARGET} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/${SRC})
        endforeach()
    else()
        if(ARTIFACT_TYPE STREQUAL "Static")
            add_library(${ARG_NAME} STATIC ${ARG_SRC})
        else()
            add_library(${ARG_NAME} SHARED ${ARG_SRC})
        endif()
        parabix_set_library_name(${ARG_NAME})
        target_link_libraries(${ARG_NAME} ${ARG_DEPS})
        if (ARG_USES_Z3)
            target_link_libraries(${ARG_NAME} ${Z3_LIBRARIES})
        endif()
    endif()
endfunction(parabix_add_module)

function(parabix_add_executable)
    cmake_parse_arguments(
        ARG         # resultant argument prefix
        ""          # boolean args
        "NAME"      # mono-valued arguments:
                    #   NAME: target name
        "SRC;DEPS"  # multi-valued arguments:
                    #   SRC:  source file list
                    #   DEPS: target dependencies
        ${ARGN}     # arguments to parse
    )
    if(NOT ARG_NAME)
        message(FATAL_ERROR "parabix_add_executable: A target name is required")
    endif()
    add_executable(${ARG_NAME} ${ARG_SRC})
    if(ARTIFACT_MODE STREQUAL "Single")
        target_link_libraries(${ARG_NAME} ${PARABIX_MONO_TARGET})
    else()
        target_link_libraries(${ARG_NAME} ${ARG_DEPS})
    endif()
    if(ARTIFACT_MODE STREQUAL "Single")
        target_link_libraries(${ARG_NAME} ${Z3_LIBRARIES})
    endif()
endfunction(parabix_add_executable)

# Creates a pablo source target which copies pablo source files to the parabix
# cache directory.
#
# Usage: parabix_add_pablo_src(target-name source-files...)
function(parabix_add_pablo_src NAME)
    set(FILE_LIST "")
    foreach(PABLO_FILE ${ARGN})
        list(APPEND FILE_LIST "${CMAKE_CURRENT_SOURCE_DIR}/${PABLO_FILE}")
    endforeach(PABLO_FILE)

    add_custom_target(${NAME}
        COMMAND cp ${FILE_LIST} ${PARABIX_OBJECT_CACHE}
        DEPENDS ${ARGN})
endfunction(parabix_add_pablo_src)


###   Add Subdirectories   ###

if(ARTIFACT_MODE STREQUAL "Single")
    if(ARTIFACT_TYPE STREQUAL "Static")
        add_library(${PARABIX_MONO_TARGET} STATIC "")
    else()
        add_library(${PARABIX_MONO_TARGET} SHARED "")
    endif()
endif()

# Framework Libraries
add_subdirectory(lib/codegen)
add_subdirectory(lib/fileselect)
add_subdirectory(lib/grep)
add_subdirectory(lib/idisa)
add_subdirectory(lib/kernel/core)
add_subdirectory(lib/kernel/pipeline)
add_subdirectory(lib/objcache)
add_subdirectory(lib/pablo)
add_subdirectory(lib/pablo/bixnum)
add_subdirectory(lib/pablo/parse)
add_subdirectory(lib/re/adt)
add_subdirectory(lib/re/alphabet)
add_subdirectory(lib/re/analysis)
add_subdirectory(lib/re/cc)
add_subdirectory(lib/re/compile)
add_subdirectory(lib/re/parse)
add_subdirectory(lib/re/toolchain)
add_subdirectory(lib/re/transforms)
add_subdirectory(lib/re/ucd)
add_subdirectory(lib/re/unicode)
add_subdirectory(lib/testing)
add_subdirectory(lib/toolchain)
add_subdirectory(lib/unicode/core)
add_subdirectory(lib/unicode/data)
add_subdirectory(lib/unicode/utf)

# Kernel Libraries
add_subdirectory(lib/kernel/basis)
add_subdirectory(lib/kernel/io)
add_subdirectory(lib/kernel/scan)
add_subdirectory(lib/kernel/streamutils)
add_subdirectory(lib/kernel/unicode)
add_subdirectory(lib/kernel/util)

# Executables
if (GENERATE_TOOLS)
    add_subdirectory(tools/base64)
    add_subdirectory(tools/editd)
    add_subdirectory(tools/icgrep)
    add_subdirectory(tools/idisa_test)
    add_subdirectory(tools/transcoders)
    add_subdirectory(tools/wc)
    # add_subdirectory(tools/wc/wc-pablo)
    add_subdirectory(tools/xml)
    add_subdirectory(tools/ztf8)
    add_subdirectory(tools/util)
endif()

# Kernel Tests
add_subdirectory(tests)

# Target definition for single library
if(ARTIFACT_MODE STREQUAL "Single")
    target_link_libraries(${PARABIX_MONO_TARGET} ${REQ_LLVM_LIBRARIES} ${Boost_LIBRARIES} ${Z3_LIBRARIES})
endif()

###   Test Suite   ###

set(QA_DIR ${PROJECT_SOURCE_DIR}/QA)
set(BIN_DIR ${PROJECT_BINARY_DIR}/bin)

add_test(
  NAME greptest
  WORKING_DIRECTORY ${QA_DIR}
  COMMAND python greptest.py --random_flag_count=2 --tests_per_grepcase=2 ${BIN_DIR}/icgrep)

add_test(
  NAME proptest
  WORKING_DIRECTORY ${QA_DIR}
  COMMAND python greptest.py -d ${QA_DIR} -t ${QA_DIR}/proptest.xml ${BIN_DIR}/icgrep)

if (EXISTS /usr/local/data/arwiki-20150901-pages-articles.xml)
add_test (NAME perf_test_1
  WORKING_DIRECTORY ${QA_DIR}
  COMMAND python greptest.py -d /usr/local/data/ -t ${QA_DIR}/perftest.xml ${BIN_DIR}/icgrep)
endif()

add_test(
  NAME abc_test
  WORKING_DIRECTORY ${QA_DIR}
  COMMAND ./run_abc ${BIN_DIR}/icgrep)

add_test(
  NAME u8u16_test
  WORKING_DIRECTORY ${QA_DIR}/u8u16
  COMMAND ./run_all "${BIN_DIR}/u8u16 -thread-num=2")

add_test(
  NAME u32u8_test
  WORKING_DIRECTORY ${QA_DIR}/
  COMMAND ./test_u32u8 "${BIN_DIR}/u32u8")

add_test(
    NAME editd_test
    WORKING_DIRECTORY ${QA_DIR}/editd
    COMMAND python run_all.py -e ${BIN_DIR})

add_test(
    NAME base64_test
    WORKING_DIRECTORY ${QA_DIR}/base64
    COMMAND python base64test.py ${BIN_DIR}/base64)

add_test(
  NAME gb18030_single_threaded_test
  WORKING_DIRECTORY ${QA_DIR}/gb18030
  COMMAND ./run_all "${BIN_DIR}/gb18030 -thread-num=1")

add_test(
  NAME gb18030_multithreaded_test
  WORKING_DIRECTORY ${QA_DIR}/gb18030
  COMMAND ./run_all "${BIN_DIR}/gb18030 -thread-num=2")


SET_PROPERTY(TEST proptest PROPERTY TIMEOUT 1500)
SET_PROPERTY(TEST abc_test PROPERTY TIMEOUT 100)
SET_PROPERTY(TEST u8u16_test editd_test base64_test PROPERTY TIMEOUT 40)


add_custom_target (greptest
    WORKING_DIRECTORY ${QA_DIR}
    COMMAND python greptest.py --random_flag_count=2 --tests_per_grepcase=2 "${BIN_DIR}/icgrep")

add_custom_target (proptest
    WORKING_DIRECTORY ${QA_DIR}
    COMMAND python greptest.py -d ${QA_DIR} -t ${QA_DIR}/proptest.xml "${BIN_DIR}/icgrep")

add_custom_target (u8u16_test
    WORKING_DIRECTORY ${QA_DIR}/u8u16
    COMMAND ./run_all "${BIN_DIR}/u8u16 -thread-num=2")

add_custom_target (u32u8_test
    WORKING_DIRECTORY ${QA_DIR}
    COMMAND ./test_u32u8 "${BIN_DIR}/u32u8 -EnableAsserts -VerifyIR")

add_custom_target (editd_test
    WORKING_DIRECTORY ${QA_DIR}/editd
    COMMAND python run_all.py -e ${BIN_DIR})

add_custom_target (base64_test
    WORKING_DIRECTORY ${QA_DIR}/base64
    COMMAND python base64test.py "${BIN_DIR}/base64")

add_custom_target(gb18030_single_threaded_test
  WORKING_DIRECTORY ${QA_DIR}/gb18030
  COMMAND ./run_all "${BIN_DIR}/gb18030 -thread-num=1")

add_custom_target(gb18030_multithreaded_test
  WORKING_DIRECTORY ${QA_DIR}/gb18030
  COMMAND ./run_all "${BIN_DIR}/gb18030 -thread-num=2")

add_custom_target (abc_gen
    WORKING_DIRECTORY ${QA_DIR}
    COMMAND python abc_testgen.py)

add_custom_target (abc_test
    WORKING_DIRECTORY ${QA_DIR}
    COMMAND ./run_abc "${BIN_DIR}/icgrep")

add_custom_target (check
    COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure
    DEPENDS icgrep u8u16 u32u8 base64 editd idisa_test kernel-test-suite)

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

add_custom_target (xml_test
    WORKING_DIRECTORY ${QA_DIR}/xml
    COMMAND python3 run_test_suite.py "${BIN_DIR}/xml")

