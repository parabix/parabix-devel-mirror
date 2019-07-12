# Copyright (c) 2019 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# module: codegen

add_library(codegen 
    CBuilder.cpp)

add_dependencies(codegen recompile-object-cache)

parabix_set_library_name(codegen)

target_link_libraries(codegen 
    toolchain
    ${REQ_LLVM_LIBRARIES}
    ${Boost_LIBRARIES})
