# Copyright (c) 2019 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# module: kernel.pipeline

parabix_add_module(
USES_Z3
NAME
    kernel.pipeline
SRC
    driver/driver.cpp
    driver/cpudriver.cpp
    # driver/NVPTXDriver.cpp
    compiler/internal/popcount_kernel.cpp
    compiler/internal/regionselectionkernel.cpp
    optimizationbranch/optimizationbranch.cpp
    pipeline_builder.cpp
    pipeline_kernel.cpp
DEPS
    objcache
)
