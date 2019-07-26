# Copyright (c) 2019 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# module: wc

parabix_add_executable(
NAME
    wc
SRC
    wc.cpp
DEPS
    fileselect
    kernel.basis
    kernel.io
    re.cc
)