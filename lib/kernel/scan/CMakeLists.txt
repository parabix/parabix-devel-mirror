# Copyright (c) 2019 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# module: kernel.scan

parabix_add_module(
NAME
    kernel.scan
SRC
    scanning.cpp
    scanmatchgen.cpp
DEPS
    kernel.core
)
