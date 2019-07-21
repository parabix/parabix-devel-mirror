# Copyright (c) 2019 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# module: pablo.bixnum

parabix_add_module(pablo.bixnum
    bixnum.cpp
    utf8gen.cpp
    utf16gen.cpp)

target_link_libraries(pablo.bixnum
    re.cc)
