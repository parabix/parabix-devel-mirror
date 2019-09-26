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

parabix_add_pablo_src(ztf1.pablosrc ztf1.pablo)

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


parabix_add_executable(
NAME
    ztf-runs
SRC
    ztf-kernel.cpp
    ztf-runs.cpp
DEPS
    grep
    pablo
    kernel.basis
    kernel.io
    kernel.pipeline
    kernel.streamutils
    kernel.util
    toolchain
)

parabix_add_executable(
NAME
    ztf-hash
SRC
    ztf-kernel.cpp
    ztf-scan.cpp
    ztf-hash.cpp
DEPS
    grep
    pablo
    kernel.basis
    kernel.io
    kernel.pipeline
    kernel.streamutils
    kernel.util
    toolchain
)



