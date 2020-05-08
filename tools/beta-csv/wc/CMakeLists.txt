

parabix_add_executable(
NAME
    csv_count_records
SRC
    csv_count_records.cpp
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




