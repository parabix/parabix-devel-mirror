# Copyright (c) 2019 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# module: re.compile

add_library(re.compile
    casing.cpp
    collect_ccs.cpp
    decomposition.cpp
    equivalence.cpp
    exclude_CC.cpp
    grapheme_clusters.cpp
    re_analysis.cpp
    re_compiler.cpp
    re_contextual_simplification.cpp
    re_local.cpp
    re_minimizer.cpp
    re_multiplex.cpp
    re_name_gather.cpp
    re_name_resolve.cpp
    re_nullable.cpp
    re_reverse.cpp
    re_simplifier.cpp
    re_star_normal.cpp
    re_toolchain.cpp
    replaceCC.cpp
    to_utf8.cpp
    validation.cpp)

parabix_set_library_name(re.compile)

target_link_libraries(re.compile
    pablo
    re.adt
    re.cc
    ucd.core
    ucd.compile
    ucd.data)
