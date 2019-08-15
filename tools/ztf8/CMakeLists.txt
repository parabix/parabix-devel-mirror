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

add_custom_target(ztf1.pablosrc 
    COMMAND cp ${CMAKE_CURRENT_SOURCE_DIR}/ztf1.pablo $ENV{HOME}/.cache/parabix
    DEPENDS ztf1.pablo)

parabix_add_executable(
NAME
    ztf1
SRC
    ztf1.cpp
DEPS
    pablo.parse
    kernel.basis
    kernel.io
    kernel.pipeline
    kernel.streamutils
    kernel.util
    toolchain
)

add_dependencies(ztf1 ztf1.pablosrc)

