# Copyright (c) 2019 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# module: re.cc

parabix_add_module(re.cc
    cc_compiler_target.cpp
    cc_compiler.cpp
    cc_kernel.cpp
    GB_18030_data.cpp
    multiplex_CCs.cpp)

target_link_libraries(re.cc
    pablo
    re.adt
    unicode.core)
