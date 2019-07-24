# Copyright (c) 2019 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# module: kernel.unicode

parabix_add_module(
NAME
    kernel.unicode
SRC
    charclasses.cpp
    grapheme_kernel.cpp
    linebreak_kernel.cpp
    UCD_property_kernel.cpp
DEPS
    kernel.pipeline
    re.unicode
)
