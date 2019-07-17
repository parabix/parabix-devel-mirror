# Copyright (c) 2019 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# module: re.compile

parabix_add_module(re.compile
    assertion_transformations.cpp
    collect_ccs.cpp
    exclude_CC.cpp
    memoization.cpp
    re_analysis.cpp
    re_compiler.cpp
    re_contextual_simplification.cpp
    re_inspector.cpp
    re_local.cpp
    re_minimizer.cpp
    re_multiplex.cpp
    re_name_gather.cpp
    re_reverse.cpp
    re_simplifier.cpp
    re_star_normal.cpp
    re_transformer.cpp
    remove_nullable.cpp
    replaceCC.cpp
    resolve_diffs.cpp
    to_utf8.cpp)

target_link_libraries(re.compile
    re.cc
    re.ucd)
