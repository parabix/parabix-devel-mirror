# Copyright (c) 2019 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# module: idisa

parabix_add_module(
NAME
    idisa
SRC
    idisa_avx_builder.cpp
    idisa_builder.cpp
    idisa_i64_builder.cpp
    # idisa_nvptx_builder.cpp
    idisa_sse_builder.cpp
DEPS
    codegen
)
