# Copyright (c) 2019 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# module: re.unicode

parabix_add_module(
NAME
    re.unicode
SRC
    boundaries.cpp
    casing.cpp
    decomposition.cpp
    equivalence.cpp
    re_name_resolve.cpp
    resolve_properties.cpp
DEPS
    re.compile
    re.parse
    unicode.data
)
