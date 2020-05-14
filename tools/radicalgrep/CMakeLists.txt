# This software is licensed to the public under the Open Software License 3.0.

# module: radicalgrep

parabix_add_executable(
NAME
    radicalgrep
SRC
    radicalgrep.cpp
    radical_interface.cpp
DEPS
    fileselect
    grep
    toolchain
    kernel.basis
    kernel.io
    re.cc
    re.parse
)
