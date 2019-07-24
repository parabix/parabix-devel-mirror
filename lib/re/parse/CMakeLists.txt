# Copyright (c) 2019 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# module: re.parse

parabix_add_module(
NAME
    re.parse
SRC
    BRE_parser.cpp
    ERE_parser.cpp
    fixed_string_parser.cpp
    GLOB_parser.cpp
    parser.cpp
    PCRE_parser.cpp
    Prosite_parser.cpp
DEPS
    re.adt
    ${Boost_LIBRARIES}
)
