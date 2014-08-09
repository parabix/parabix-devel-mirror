cmake_minimum_required (VERSION 2.8)

project (icgrep)

# The version number.
set (icgrep_VERSION_MAJOR 0)
set (icgrep_VERSION_MINOR 8)
 
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

 
add_library(PabloADT pe_advance.cpp  pe_all.cpp  pe_and.cpp pe_call.cpp pe_charclass.cpp  pe_matchstar.cpp pe_scanthru.cpp pe_not.cpp  pe_or.cpp  pe_pabloe.cpp  pe_sel.cpp  pe_var.cpp  pe_xor.cpp ps_assign.cpp  ps_if.cpp  ps_pablos.cpp  ps_while.cpp printer_pablos.cpp)

add_library(RegExpADT re_alt.cpp  re_cc.cpp  re_end.cpp  re_name.cpp re_parser.cpp  re_re.cpp  re_rep.cpp  re_seq.cpp  re_start.cpp parsefailure.cpp  parseresult.cpp  parsesuccess.cpp printer_re.cpp)



#include_directories("${PROJECT_BINARY_DIR}")
include_directories("${PROJECT_SOURCE_DIR}") 
include_directories("${PROJECT_SOURCE_DIR}/include")
include_directories("${PROJECT_SOURCE_DIR}/include/simd-lib")
include_directories("${PROJECT_SOURCE_DIR}/include/simd-lib/idisa_cpp")
 
# add the executable
add_executable(icgrep icgrep.cpp llvm_gen.cpp llvm_gen_helper.cpp utf_encoding.cpp cc_codegenobject.cpp  cc_compiler.cpp  cc_compiler_helper.cpp re_simplifier.cpp re_reducer.cpp re_nullable.cpp re_compiler.cpp pbix_compiler.cpp  symbol_generator.cpp utf8_encoder.cpp unicode_categories.h)

target_link_libraries (icgrep PabloADT RegExpADT ${REQ_LLVM_LIBRARIES})

set(CMAKE_CXX_FLAGS_RELEASE "-std=gnu++0x -O3 -msse2")
