# Copyright (c) 2019 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# module: grep

parabix_add_module(grep
    grep_engine.cpp
    grep_kernel.cpp
    grep_name_resolve.cpp
    resolve_properties.cpp
    regex_passes.cpp)

target_link_libraries(grep
    kernel.pipeline
    kernel.basis
    kernel.io
    kernel.streamutils
    kernel.util
    re.parse
    re.unicode)
