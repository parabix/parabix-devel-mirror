# Copyright (c) 2019 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# module: re.adt

add_library(re.adt
    printer_re.cpp
    re_assertion.cpp
    re_cc.cpp
    re_diff.cpp
    re_empty_set.cpp
    re_intersect.cpp
    re_range.cpp
    re_re.cpp
    re_rep.cpp
    re_utility.cpp)

parabix_set_library_name(re.adt)

target_link_libraries(re.adt
    re.cc
    re.compile
    ucd.core)
