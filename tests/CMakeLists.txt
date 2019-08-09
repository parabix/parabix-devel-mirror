# Copyright (c) 2019 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# Place test executables in build/tests/bin
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin)

function(parabix_add_kernel_test NAME)
    parabix_add_executable(NAME ${NAME} SRC ${NAME}.cpp DEPS testing ${ARGN})
endfunction(parabix_add_kernel_test)

parabix_add_kernel_test(temp_test kernel.util)
parabix_add_kernel_test(test_scan_index_gen kernel.scan)
