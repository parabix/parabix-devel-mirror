# Copyright (c) 2019 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# module: base64

parabix_add_executable(
NAME
    base64
SRC
    base64.cpp
DEPS
    pablo
    kernel.io
    kernel.pipeline
    kernel.util
    re.adt
    re.cc
)
