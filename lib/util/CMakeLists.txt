# Copyright (c) 2019 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# module: util

# TODO: move file_select.cpp to its own library

add_library(util
    file_select.cpp)

parabix_set_library_name(util)

target_link_libraries(util
    grep
    re.adt
    re.compile
    re.parse
    toolchain)
