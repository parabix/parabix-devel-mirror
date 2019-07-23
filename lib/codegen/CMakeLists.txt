# Copyright (c) 2019 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# module: codegen

parabix_add_module(codegen 
    CBuilder.cpp)

add_dependencies(codegen recompile-object-cache)

target_link_libraries(codegen
    toolchain
    ${Boost_LIBRARIES})
