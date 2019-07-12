# Copyright (c) 2019 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# module: re.parse

add_library(re.parse
    BRE_parser.cpp
    ERE_parser.cpp
    fixed_string_parser.cpp
    GLOB_parser.cpp
    parser.cpp
    PCRE_parser.cpp
    Prosite_parser.cpp)

parabix_set_library_name(re.parse)

target_link_libraries(re.parse
    re.adt)
