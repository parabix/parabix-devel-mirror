# Copyright (c) 2019 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# module: icgrep

parabix_add_executable(
NAME
    icgrep
SRC
    grep_interface.cpp
    icgrep.cpp
DEPS
    fileselect
    grep
)
