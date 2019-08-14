# This software is licensed to the public under the Open Software License 3.0.

# module: ztf8

parabix_add_executable(
NAME
    ztf8-study
SRC
    ztf8-study.cpp
DEPS
    grep
)

parabix_add_executable(
NAME
    matchprior
SRC
    matchprior.cpp
DEPS
    fileselect
    kernel.basis
    kernel.io
)
