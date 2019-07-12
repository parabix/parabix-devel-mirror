# Copyright (c) 2019 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# module: ucd.data

add_library(ucd.data
    CaseFolding.cpp
    Equivalence.cpp
    PropertyObjects.cpp)

parabix_set_library_name(ucd.data)

target_link_libraries(ucd.data
    ucd.core
    grep
    re.adt
    re.compile
    toolchain)
