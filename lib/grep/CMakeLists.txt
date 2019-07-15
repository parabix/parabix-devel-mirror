# Copyright (c) 2019 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# module: grep

add_library(grep
    grep_engine.cpp
    grep_kernel.cpp)

parabix_set_library_name(grep)

target_link_libraries(grep
    kernel.core
    pablo
    re.adt
    re.cc
    re.compile
    toolchain
    unicode.compile
    kernel.pipeline
    kernel.util)
