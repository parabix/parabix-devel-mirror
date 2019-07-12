# Copyright (c) 2019 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# module: gb18030

add_executable(gb18030
    gb18030.cpp)

target_link_libraries(gb18030
    pablo
    re.adt
    re.cc)
