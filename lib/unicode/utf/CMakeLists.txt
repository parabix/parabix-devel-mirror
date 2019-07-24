# Copyright (c) 2019 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# module: unicode.utf

parabix_add_module(
NAME
    unicode.utf
SRC
    utf8_encoder.cpp
    utf16_encoder.cpp
DEPS
    unicode.core
)
