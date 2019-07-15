# Copyright (c) 2019 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# module: idisa_test

add_executable(idisa_test
    idisa_test.cpp)

target_link_libraries(idisa_test
    kernel.core
    kernel.pipeline
    kernel.util
    pablo)
