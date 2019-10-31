# Copyright (c) 2019 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# module: re.compile

parabix_add_module(
NAME
    re.transforms
SRC
    assertion_transformations.cpp
    exclude_CC.cpp
    name_lookaheads.cpp
    re_contextual_simplification.cpp
    re_minimizer.cpp
    re_multiplex.cpp
    re_reverse.cpp
    re_simplifier.cpp
    re_star_normal.cpp
    re_memoizing_transformer.cpp
    re_transformer.cpp
    remove_nullable.cpp
    replaceCC.cpp
    resolve_diffs.cpp
    to_utf8.cpp
DEPS
    re.analysis
    re.ucd
)
