# Copyright (c) 2019 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# module: json

parabix_add_pablo_src(json.pablosrc json.pablo)

parabix_add_executable(
NAME
    json
SRC
    json.cpp
DEPS
    pablo.parse
    kernel.basis
    kernel.io
    kernel.pipeline
    kernel.scan
    kernel.streamutils
    kernel.util
    toolchain
)

add_dependencies(json json.pablosrc)