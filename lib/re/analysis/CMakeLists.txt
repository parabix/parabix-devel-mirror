# Copyright (c) 2019 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# module: re.analysis

parabix_add_module(
NAME
    re.analysis
SRC
    cc_sequence_search.cpp
    collect_ccs.cpp
    nullable.cpp
    re_analysis.cpp
    re_inspector.cpp
    re_name_gather.cpp
    re_local.cpp
    validation.cpp
DEPS
    re.adt
)
