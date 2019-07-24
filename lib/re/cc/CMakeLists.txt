# Copyright (c) 2019 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# module: re.cc

parabix_add_module(
NAME
    re.cc
SRC
    cc_compiler_target.cpp
    cc_compiler.cpp
    cc_kernel.cpp
    GB_18030_data.cpp
    multiplex_CCs.cpp
DEPS
    pablo
    re.adt
    unicode.core
)
