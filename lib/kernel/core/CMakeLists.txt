# Copyright (c) 2019 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# module: kerenl.core

parabix_add_module(
NAME    
    kernel.core
SRC
    attributes.cpp
    binding.cpp
    block_kernel.cpp
    callback.cpp
    idisa_target.cpp
    kernel_builder.cpp
    kernel_compiler.cpp
    block_kernel_compiler.cpp
    kernel.cpp
    multiblock_kernel.cpp
    processing_rate.cpp
    relationship.cpp
    streamset.cpp
DEPS
    idisa
)
