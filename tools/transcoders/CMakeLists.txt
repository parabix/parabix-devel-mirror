# Copyright (c) 2019 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# module: transcoders

parabix_add_executable(
NAME
    gb18030
SRC
    gb18030.cpp
DEPS
    pablo.bixnum
    kernel.basis
    kernel.io
    kernel.pipeline
    kernel.streamutils
    kernel.util
)

parabix_add_executable(
NAME
    u8u16
SRC
    u8u16.cpp
DEPS
    pablo.bixnum
    kernel.basis
    kernel.io
    kernel.pipeline
    kernel.streamutils
    re.cc
    toolchain
)

parabix_add_executable(
NAME
    u32u8
SRC
    u32u8.cpp
DEPS
    pablo.bixnum
    kernel.basis
    kernel.io
    kernel.pipeline
    kernel.streamutils
    re.cc
    toolchain
)

parabix_add_executable(
NAME
    u32u16
SRC
    u32u16.cpp
DEPS
    pablo.bixnum
    kernel.basis
    kernel.io
    kernel.pipeline
    kernel.streamutils
    re.cc
    toolchain
)

parabix_add_executable(
NAME
    x8u16
SRC
    x8u16.cpp
DEPS
    kernel.basis
    kernel.io
    kernel.pipeline
    kernel.streamutils
    re.cc
    toolchain
)

parabix_add_executable(
NAME
    u16u8
SRC
    u16u8.cpp
DEPS
    pablo.bixnum
    kernel.basis
    kernel.io
    kernel.pipeline
    kernel.streamutils
    re.cc
    toolchain
    kernel.util
)