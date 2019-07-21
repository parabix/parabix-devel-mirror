# Copyright (c) 2019 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# module: fileselect

parabix_add_module(fileselect
    file_select.cpp)

target_link_libraries(fileselect
    grep)
