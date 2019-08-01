# Copyright (c) 2019 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# module: kernel.util

parabix_add_module(
NAME
    kernel.util
SRC
    error_monitor_kernel.cpp
    debug_display.cpp
    hex_convert.cpp
    linebreak_kernel.cpp
    radix64.cpp
    random_stream.cpp
DEPS
    pablo
    re.cc
    kernel.pipeline
)
