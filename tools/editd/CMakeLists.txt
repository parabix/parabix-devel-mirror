# Copyright (c) 2019 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# module: editd

add_executable(editd
    editd_cpu_kernel.cpp
    editd_gpu_kernel.cpp
    editd.cpp
    editdscan_kernel.cpp
    pattern_compiler.cpp)

target_link_libraries(editd
    pablo
    kernel.util
    re.adt
    re.cc)
