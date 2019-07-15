# Copyright (c) 2019 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# module: re.alphabet

parabix_add_module(re.alphabet
    alphabet.cpp)

target_link_libraries(re.alphabet
    ucd.core)
