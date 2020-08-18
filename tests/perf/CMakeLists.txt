# Copyright (c) 2020 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# Place test executables in build/tests/perf/bin
set(BIN_DIR ${PROJECT_BINARY_DIR}/bin)
set(TEST_BIN_DIR ${CMAKE_CURRENT_BINARY_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${TEST_BIN_DIR})

set(BEST_FLAGS "--regalloc=basic --optimize-regalloc--fast-isel=true --disable-x86-domain-reassignment=true --disable-machine-licm --disable-machine-cse --disable-lsr --disable-phi-elim-edge-splitting")

# Add a new perf-test target.
function(parabix_add_perf_test NAME REGEX FILE)
    set(PERF_TEST_TARGET_LIST ${PERF_TEST_TARGET_LIST} ${NAME} PARENT_SCOPE)
    add_test(NAME "${NAME}"
        WORKING_DIRECTORY ${BIN_DIR}
        COMMAND icgrep ${REGEX} ${FILE} -c ${BEST_FLAGS}
        CONFIGURATIONS perfonly
        DEPENDS icgrep)
endfunction(parabix_add_perf_test)

###   Test Targets   ###
parabix_add_perf_test(ex1 "abc" ${PROJECT_SOURCE_DIR}/CMakeLists.txt)

# `make perf-tests` to run all perf tests
add_custom_target (perf-tests
    COMMAND ${CMAKE_CTEST_COMMAND} -C perfonly --output-on-failure
    DEPENDS icgrep)
