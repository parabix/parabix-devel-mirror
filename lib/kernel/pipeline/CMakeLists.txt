# Copyright (c) 2019 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# module: kernel.pipeline

add_library(kernel.pipeline
    internal/popcount_kernel.cpp
    internal/regionselectionkernel.cpp
    optimizationbranch/optimizationbranch.cpp
    pipeline_builder.cpp
    pipeline_kernel.cpp)

parabix_set_library_name(kernel.pipeline)

target_link_libraries(kernel.pipeline
    kernel.core)
