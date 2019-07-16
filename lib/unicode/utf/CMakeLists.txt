# Copyright (c) 2019 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# module: unicode.utf

parabix_add_module(unicode.utf
    utf8_encoder.cpp
    utf16_encoder.cpp)

target_link_libraries(unicode.utf
    unicode.core)
