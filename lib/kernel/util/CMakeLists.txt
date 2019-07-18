# Copyright (c) 2019 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# module: kernel.util

# TODO: break this module into separate kernel moduels

add_library(kernel.util
    # alignedprint.cpp
    bitstream_gather_pdep_kernel.cpp
    bitstream_pdep_kernel.cpp
    callback.cpp
    cc_scan_kernel.cpp
    charclasses.cpp
    deletion.cpp
    delmask_kernel.cpp
    # directorysearch.cpp
    error_monitor_kernel.cpp
    evenodd.cpp
    fake_stream_generating_kernel.cpp
    grapheme_kernel.cpp
    hex_convert.cpp
    linebreak_kernel.cpp
    pdep_kernel.cpp
    radix64.cpp
    random_stream.cpp
    scan_kernel.cpp
    scanmatchgen.cpp
    scanning.cpp
    stream_select.cpp
    stream_shift.cpp
    streams_merge.cpp
    streamset_collapse.cpp
    swizzle.cpp
    swizzled_multiple_pdep_kernel.cpp
    u8u32_kernel.cpp
    UCD_property_kernel.cpp
    until_n.cpp
    zeroextend.cpp)

parabix_set_library_name(kernel.util)

target_link_libraries(kernel.util
    kernel.core
    re.compile)
