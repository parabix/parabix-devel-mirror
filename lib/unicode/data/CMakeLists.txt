# Copyright (c) 2019 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# module: unicode.data

parabix_add_module(
NAME
    unicode.data
SRC
    CaseFolding.cpp
    Equivalence.cpp
    PropertyAliases.cpp
    PropertyObjects.cpp
    RadicalSets.cpp
DEPS
    unicode.core
)
