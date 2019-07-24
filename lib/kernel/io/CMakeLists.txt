# Copyright (c) 2019 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# module: kernel.io

parabix_add_module(
NAME
    kernel.io
SRC
    source_kernel.cpp
    stdout_kernel.cpp
DEPS
    kernel.core
)
