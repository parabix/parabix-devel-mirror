# Copyright (c) 2019 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# module: re.compile

parabix_add_module(
NAME
    re.compile
SRC
    re_compiler.cpp
DEPS
    re.analysis
    re.transforms
    re.ucd
)
