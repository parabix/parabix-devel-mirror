# Copyright (c) 2019 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# module: wc

add_executable(wc
    wc.cpp)

target_link_libraries(wc
    pablo
    kernel.core
    kernel.pipeline
    kernel.util
    re.adt
    re.cc
    toolchain
    util)
