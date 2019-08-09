# Copyright (c) 2019 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# module: testing

parabix_add_module(
NAME
    testing
SRC
    assert.cpp
    runtime.cpp
    stream_gen.cpp
DEPS
    kernel.core
    kernel.io
    kernel.pipeline
    kernel.util
)
