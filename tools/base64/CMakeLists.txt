# Copyright (c) 2019 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# module: base64

add_executable(base64
    base64.cpp)

target_link_libraries(base64
    pablo
    kernel.io
    kernel.pipeline
    kernel.util
    re.adt
    re.cc)
