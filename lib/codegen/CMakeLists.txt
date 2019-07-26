# Copyright (c) 2019 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# module: codegen

parabix_add_module(
NAME
    codegen
SRC
    CBuilder.cpp
DEPS
    toolchain
    ${Boost_LIBRARIES}
)

if(BUILD_SINGLE_SHARED_LIB)
    add_dependencies(${PARABIX_MONO_TARGET} recompile-object-cache)
else()
    add_dependencies(codegen recompile-object-cache)
endif()
