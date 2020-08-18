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

# Pattern: .* (with the -v or --invert-match flag set)
parabix_add_perf_test(nothing ".*" ${PROJECT_SOURCE_DIR}/CMakeLists.txt)

# Pattern: Sherlock Holmes (with --context 2)
parabix_add_perf_test(context "Sherlock Holmes" ${PROJECT_SOURCE_DIR}/CMakeLists.txt)

# Pattern: Sherlock Holmes on huge file
parabix_add_perf_test(huge "Sherlock Holmes" ${PROJECT_SOURCE_DIR}/CMakeLists.txt)

# Pattern: PM_RESUME
parabix_add_perf_test(linux_literal_default "PM_RESUME" ${PROJECT_SOURCE_DIR}/CMakeLists.txt)

# Pattern: \wAh
parabix_add_perf_test(linux_unicode_word "\\wAh" ${PROJECT_SOURCE_DIR}/CMakeLists.txt)

# Pattern: [A-Z]+_RESUME
parabix_add_perf_test(linux_re_literal_suffix "[A-Z]+_RESUME" ${PROJECT_SOURCE_DIR}/CMakeLists.txt)

# Pattern: ERR_SYS|PME_TURN_OFF|LINK_REQ_RST|CFG_BME_EVT
parabix_add_perf_test(linux_alternates "ERR_SYS|PME_TURN_OFF|LINK_REQ_RST|CFG_BME_EVT" ${PROJECT_SOURCE_DIR}/CMakeLists.txt)

# Pattern: ERR_SYS|PME_TURN_OFF|LINK_REQ_RST|CFG_BME_EVT  (with the -i flag set)
parabix_add_perf_test(linux_alternates_casei "ERR_SYS|PME_TURN_OFF|LINK_REQ_RST|CFG_BME_EVT" ${PROJECT_SOURCE_DIR}/CMakeLists.txt)

# Pattern: \p{Greek}
parabix_add_perf_test(linux_unicode_greek "\\p{Greek}" ${PROJECT_SOURCE_DIR}/CMakeLists.txt)

# Pattern: \p{Greek} (with the -i flag set, matches any Greek symbol)
parabix_add_perf_test(linux_unicode_greek_casei "\\p{Greek}" ${PROJECT_SOURCE_DIR}/CMakeLists.txt)

# Pattern: \w{5}\s+\w{5}\s+\w{5}\s+\w{5}\s+\w{5}
parabix_add_perf_test(linux_no_literal "\\w{5}\\s+\\w{5}\\s+\\w{5}\\s+\\w{5}\\s+\\w{5}" ${PROJECT_SOURCE_DIR}/CMakeLists.txt)

# Pattern: Sherlock Holmes|John Watson|Irene Adler|Inspector Lestrade|Professor Moriarty
parabix_add_perf_test(subtitles_alternate_en "Sherlock Holmes|John Watson|Irene Adler|Inspector Lestrade|Professor Moriarty" ${PROJECT_SOURCE_DIR}/CMakeLists.txt)

# Russian pattern: Шерлок Холмс|Джон Уотсон|Ирен Адлер|инспектор Лестрейд|профессор Мориарти
parabix_add_perf_test(subtitles_alternate_rs "Шерлок Холмс|Джон Уотсон|Ирен Адлер|инспектор Лестрейд|профессор Мориарти" ${PROJECT_SOURCE_DIR}/CMakeLists.txt)

# Pattern: Sherlock Holmes|John Watson|Irene Adler|Inspector Lestrade|Professor Moriarty (with the -i flag set)
parabix_add_perf_test(subtitles_alternate_casei "Sherlock Holmes|John Watson|Irene Adler|Inspector Lestrade|Professor Moriarty" ${PROJECT_SOURCE_DIR}/CMakeLists.txt)

# Pattern: \w+\s+Holmes\s+\w+
parabix_add_perf_test(subtitles_surrounding_words_en "\\w+\\s+Holmes\\s+\\w+" ${PROJECT_SOURCE_DIR}/CMakeLists.txt)

# Russian pattern: \w+\s+Холмс\s+\w+
parabix_add_perf_test(subtitles_surrounding_words_rs "\\w+\\s+Холмс\\s+\\w+" ${PROJECT_SOURCE_DIR}/CMakeLists.txt)

# `make perf-tests` to run all perf tests
add_custom_target (perf-tests
    COMMAND ${CMAKE_CTEST_COMMAND} -C perfonly --output-on-failure
    DEPENDS icgrep)
