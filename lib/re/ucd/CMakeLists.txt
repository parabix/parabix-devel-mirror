# Copyright (c) 2019 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# module: re.ucd

parabix_add_module(
NAME
    re.ucd
SRC
    ucd_compiler.cpp
DEPS
    re.cc
    unicode.utf
)
