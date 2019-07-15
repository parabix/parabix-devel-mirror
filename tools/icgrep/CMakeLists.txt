# Copyright (c) 2019 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# module: icgrep

add_executable(icgrep
    grep_interface.cpp
    icgrep.cpp)

target_link_libraries(icgrep
    grep
    pablo
    kernel.core
    kernel.pipeline
    kernel.util
    re.adt
    re.alphabet
    re.cc
    re.compile
    re.parse
    re.toolchain
    unicode.compile
    unicode.core
    unicode.data
    util)
