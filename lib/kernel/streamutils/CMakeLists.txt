# Copyright (c) 2019 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# module: kernel.streamutils

parabix_add_module(kernel.streamutils
    deletion.cpp
    pdep_kernel.cpp
    stream_select.cpp
    stream_shift.cpp
    streams_merge.cpp
    streamset_collapse.cpp)

target_link_libraries(kernel.streamutils
    toolchain # TODO: remove this dependnecy
    pablo
    kernel.pipeline)
