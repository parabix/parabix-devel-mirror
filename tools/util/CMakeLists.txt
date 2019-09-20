# This software is licensed to the public under the Open Software License 3.0.

# module: util

parabix_add_executable(
NAME
    hexlines
SRC
    hexlines.cpp
DEPS
    pablo
    kernel.basis
    kernel.io
    kernel.pipeline
    kernel.streamutils
    kernel.util
    toolchain
)



