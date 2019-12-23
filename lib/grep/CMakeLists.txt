# Copyright (c) 2019 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# module: grep

parabix_add_module(
NAME
    grep
SRC
    grep_engine.cpp
    grep_kernel.cpp
    grep_name_resolve.cpp
    grep_toolchain.cpp
    nested_grep_engine.cpp
    resolve_properties.cpp
    regex_passes.cpp
DEPS
    kernel.pipeline
    kernel.basis
    kernel.io
    kernel.scan
    kernel.streamutils
    kernel.util
    kernel.unicode
    re.parse
    re.unicode
)
