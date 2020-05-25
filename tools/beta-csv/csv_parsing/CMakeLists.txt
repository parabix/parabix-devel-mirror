





parabix_add_executable(
NAME
    temp
SRC
    temp.cpp
DEPS
    pablo
    kernel.basis
    kernel.io
    kernel.pipeline
    kernel.streamutils
    kernel.util
    toolchain
    fileselect
    re.cc
    re.parse
)


