# Copyright (c) 2019 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# module: transcoders

add_executable(gb18030
    gb18030.cpp)

add_executable(u8u16
    u8u16.cpp)

add_executable(u32u8
    u32u8.cpp)

add_executable(u32u16
    u32u16.cpp)

add_executable(x8u16
    x8u16.cpp)

target_link_libraries(gb18030
    kernel.basis
    kernel.io
    kernel.pipeline
    kernel.util
    re.cc)

target_link_libraries(u8u16
    kernel.basis
    kernel.io
    kernel.pipeline
    kernel.util
    re.cc
    toolchain)

target_link_libraries(u32u8
    kernel.basis
    kernel.io
    kernel.pipeline
    kernel.util
    re.cc
    toolchain)

target_link_libraries(u32u16
    kernel.basis
    kernel.io
    kernel.pipeline
    kernel.util
    re.cc
    toolchain)

target_link_libraries(x8u16
    kernel.basis
    kernel.io
    kernel.pipeline
    kernel.util
    re.cc
    toolchain)
