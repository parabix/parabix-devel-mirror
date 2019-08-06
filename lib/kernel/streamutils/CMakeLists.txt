# Copyright (c) 2019 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# module: kernel.streamutils

parabix_add_module(
NAME
    kernel.streamutils
SRC
    collapse.cpp
    deletion.cpp
    pdep_kernel.cpp
    sentinel.cpp
    stream_select.cpp
    stream_shift.cpp
    streams_merge.cpp
    swizzle.cpp
    swizzled_multiple_pdep_kernel.cpp
    until_n.cpp
    zeroextend.cpp
DEPS
    pablo
    kernel.pipeline
)
