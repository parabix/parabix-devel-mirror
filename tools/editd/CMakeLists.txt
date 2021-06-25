# Copyright (c) 2019 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# module: editd

parabix_add_executable(
NAME
    editd
SRC
    editd_cpu_kernel.cpp
    editd_gpu_kernel.cpp
    editd.cpp
    editdscan_kernel.cpp
    pattern_compiler.cpp
DEPS
    pablo
    kernel.basis
    kernel.io
    kernel.pipeline
    kernel.streamutils
    re.cc
    LLVMVersion
)
