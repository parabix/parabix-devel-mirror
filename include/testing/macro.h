/*
 * Copyright (c) 2019 International Characters.
 * This software is licensed to the public under the Open Software License 3.0.
 */

#pragma once

#include <tuple>
#include <testing/test_case.hpp>
#include <testing/runtime.h>
#include <testing/stream_gen.hpp>
#include <toolchain/toolchain.h>
#include <toolchain/pablo_toolchain.h>

#define TEST_CASE(NAME, ...)                                                                                    \
template<typename... Ps>                                                                                        \
struct test_case_##NAME {                                                                                       \
    using engine_t = testing::tc::test_engine<Ps...>;                                                           \
    using pipeline_t = const std::unique_ptr<kernel::ProgramBuilder> &;                                         \
    template<size_t Index>                                                                                      \
    using input_t = typename std::tuple_element<Index, typename engine_t::pipeline_input_pack_t>::type;         \
                                                                                                                \
    template<size_t Index>                                                                                      \
    static input_t<Index> Input(engine_t & T) { return T.template input<Index>(); }                             \
                                                                                                                \
    static void run(engine_t & T, pipeline_t P);                                                                \
};                                                                                                              \
                                                                                                                \
int32_t invoke_##NAME() {                                                                                       \
    auto test = testing::tc::make_test_case(__VA_ARGS__);                                                       \
    using tc_t = meta::swap_container_t<decltype(test), test_case_##NAME>;                                      \
    test.set_body(tc_t::run);                                                                                   \
    test.invoke();                                                                                              \
    return test.get_result();                                                                                   \
}                                                                                                               \
                                                                                                                \
template<typename... Ps>                                                                                        \
void test_case_##NAME<Ps...>::run(engine_t & T, pipeline_t P)


#define RUN_TESTS(...)                                                                                          \
int main(int argc, char ** argv) {                                                                              \
    codegen::ParseCommandLineOptions(argc, argv, {                                                              \
        codegen::codegen_flags(),                                                                               \
        pablo::pablo_toolchain_flags(),                                                                         \
        testing::cli::testFlags()                                                                               \
    });                                                                                                         \
    return testing::RunTestSuite({__VA_ARGS__});                                                                \
}


#define CASE(X) (std::tuple<std::string, TestCaseInvocationType>(std::string(#X), invoke_##X))
