# Copyright (c) 2019 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# module: unicode.compile

parabix_add_module(unicode.compile
    resolve_properties.cpp
    ucd_compiler.cpp)

target_link_libraries(unicode.compile
    pablo
    re.adt
    re.cc
    re.parse
    re.compile
    unicode.core
    unicode.data
    unicode.utf)
