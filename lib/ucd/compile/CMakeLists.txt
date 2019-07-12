# Copyright (c) 2019 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# module: ucd.compile

add_library(ucd.compile
    resolve_properties.cpp
    ucd_compiler.cpp
    utf8_encoder.cpp
    utf16_encoder.cpp)

parabix_set_library_name(ucd.compile)

target_link_libraries(ucd.compile
    pablo
    re.adt
    re.cc
    re.parse
    re.compile
    ucd.core
    ucd.data)
