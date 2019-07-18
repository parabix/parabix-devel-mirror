# Copyright (c) 2019 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# module: kernel.util

# TODO: break this module into separate kernel moduels

parabix_add_module(kernel.util
    callback.cpp
    charclasses.cpp
    error_monitor_kernel.cpp
    grapheme_kernel.cpp
    hex_convert.cpp
    linebreak_kernel.cpp
    radix64.cpp
    random_stream.cpp
    scanmatchgen.cpp
    scanning.cpp
    swizzle.cpp
    swizzled_multiple_pdep_kernel.cpp
    UCD_property_kernel.cpp
    until_n.cpp
    zeroextend.cpp)

target_link_libraries(kernel.util
    kernel.core
    re.compile)
