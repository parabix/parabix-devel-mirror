# Copyright (c) 2019 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# module: idisa_test

parabix_add_executable(
NAME
    idisa_test
SRC
    idisa_test.cpp
DEPS
    kernel.basis
    kernel.io
    kernel.pipeline
    kernel.util
)
