# Copyright (c) 2019 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# module: re.ucd

parabix_add_module(re.ucd
    ucd_compiler.cpp)

target_link_libraries(re.ucd
    re.cc
    unicode.utf)
