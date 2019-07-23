# Copyright (c) 2019 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# module: unicode.core

parabix_add_module(unicode.core
    unicode_set.cpp)

target_link_libraries(unicode.core
    toolchain)
