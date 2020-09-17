# Copyright (c) 2020 International Characters.
# This software is licensed to the public under the Open Software License 3.0.

# Place test executables in build/tests/perf/bin
set(BIN_DIR ${PROJECT_BINARY_DIR}/bin)
set(TEST_BIN_DIR ${CMAKE_CURRENT_BINARY_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${TEST_BIN_DIR})
set(SCRIPT_DIR ${PROJECT_SOURCE_DIR}/scripts/perf_single_llvm)
set(ARTIFACTS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/artifacts)

set(BEST_FLAGS "--regalloc=basic --optimize-regalloc--fast-isel=true --disable-x86-domain-reassignment=true --disable-machine-licm --disable-machine-cse --disable-lsr --disable-phi-elim-edge-splitting")

# Add a new perf-test target.
function(parabix_add_perf_test NAME REGEX TARGET_FILE)
    set(PERF_TEST_TARGET_LIST ${PERF_TEST_TARGET_LIST} ${NAME} PARENT_SCOPE)
    add_test(NAME "${NAME}"
        WORKING_DIRECTORY ${BIN_DIR}
        COMMAND python3 ${SCRIPT_DIR}/script.py -b ${PROJECT_BINARY_DIR} -x ${REGEX} -t ${TARGET_FILE} -f ${ARTIFACTS_DIR}/results.csv -n ${NAME} ${BEST_FLAGS}
        CONFIGURATIONS perfonly
        DEPENDS icgrep)
endfunction(parabix_add_perf_test)

###   Test Targets   ###

# Pattern: Sherlock Holmes on huge file
parabix_add_perf_test(sherlock "Sherlock Holmes" ${PROJECT_SOURCE_DIR}/CMakeLists.txt)

# Pattern: \wAh
parabix_add_perf_test(unicode_word "\\wAh" ${PROJECT_SOURCE_DIR}/CMakeLists.txt)

# Pattern: \p{Greek}
parabix_add_perf_test(unicode_greek "\\p{Greek}" ${PROJECT_SOURCE_DIR}/CMakeLists.txt)

# Pattern: \w{5}\s+\w{5}\s+\w{5}\s+\w{5}\s+\w{5}
parabix_add_perf_test(no_literal "\\w{5}\\s+\\w{5}\\s+\\w{5}\\s+\\w{5}\\s+\\w{5}" ${PROJECT_SOURCE_DIR}/CMakeLists.txt)

# Pattern: Sherlock Holmes|John Watson|Irene Adler|Inspector Lestrade|Professor Moriarty
parabix_add_perf_test(subtitles_alternate_en "Sherlock Holmes|John Watson|Irene Adler|Inspector Lestrade|Professor Moriarty" ${PROJECT_SOURCE_DIR}/CMakeLists.txt)

# Russian pattern: Шерлок Холмс|Джон Уотсон|Ирен Адлер|инспектор Лестрейд|профессор Мориарти
parabix_add_perf_test(subtitles_alternate_rs "Шерлок Холмс|Джон Уотсон|Ирен Адлер|инспектор Лестрейд|профессор Мориарти" ${PROJECT_SOURCE_DIR}/CMakeLists.txt)

# Pattern: \w+\s+Holmes\s+\w+
parabix_add_perf_test(subtitles_surrounding_en "\\w+\\s+Holmes\\s+\\w+" ${PROJECT_SOURCE_DIR}/CMakeLists.txt)

# Russian pattern: \w+\s+Холмс\s+\w+
parabix_add_perf_test(subtitles_surrounding_rs "\\w+\\s+Холмс\\s+\\w+" ${PROJECT_SOURCE_DIR}/CMakeLists.txt)

# `make perf-tests` to run all perf tests
add_custom_target (perf-tests
    COMMAND ${CMAKE_CTEST_COMMAND} -C perfonly --output-on-failure
    DEPENDS icgrep)
