# Copyright (c) 2019 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# module: kerenl.core

add_library(kernel.core
    attributes.cpp
    binding.cpp
    block_kernel.cpp
    idisa_target.cpp
    kernel_builder.cpp
    kernel.cpp
    multiblock_kernel.cpp
    processing_rate.cpp
    relationship.cpp
    streamset.cpp)

parabix_set_library_name(kernel.core)

target_link_libraries(kernel.core
    codegen
    idisa)
