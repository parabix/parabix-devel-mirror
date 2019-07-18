# Copyright (c) 2019 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# module: kernel.util

parabix_add_module(kernel.util
    error_monitor_kernel.cpp
    hex_convert.cpp
    radix64.cpp
    random_stream.cpp)

target_link_libraries(kernel.util
    kernel.core)
