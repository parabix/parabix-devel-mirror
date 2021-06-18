# Copyright (c) 2019 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# module: kernel.basis

parabix_add_module(
NAME
    kernel.basis
SRC
    p2s_kernel.cpp
    s2p_kernel.cpp
DEPS
    pablo
)
