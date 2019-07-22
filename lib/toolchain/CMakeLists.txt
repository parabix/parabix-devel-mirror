# Copyright (c) 2019 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# module: toolchain

parabix_add_module(toolchain
    toolchain.cpp)

target_link_libraries(toolchain
    pablo)
