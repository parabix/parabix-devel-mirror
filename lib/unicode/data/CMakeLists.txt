# Copyright (c) 2019 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# module: unicode.data

parabix_add_module(unicode.data
    CaseFolding.cpp
    Equivalence.cpp
    PropertyObjects.cpp)

target_link_libraries(unicode.data
    unicode.core
    grep
    re.adt
    re.compile
    toolchain)
