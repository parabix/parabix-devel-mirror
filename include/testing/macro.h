/*
 * Copyright (c) 2019 International Characters.
 * This software is licensed to the public under the Open Software License 3.0.
 */

#pragma once

#include <tuple>
#include <testing/common.h>
#include <testing/runtime.h>
#include <testing/stream_gen.hpp>
#include <toolchain/toolchain.h>
#include <toolchain/pablo_toolchain.h>

#define TEST_CASE(NAME, INPUT, EXPECTED)                                                                    \
void __test_body_##NAME(testing::TestEngine &, testing::StreamSet *, testing::StreamSet *);                 \
                                                                                                            \
testing::UnitTestFunc __gen_##NAME(testing::TestEngine & T) {                                               \
    auto const & __b_ = T.driver().getBuilder();                                                            \
    T.makePipeline({                                                                                        \
        {testing::BufferTypeOf(T, INPUT), #INPUT "_ptr"}, {__b_->getSizeTy(), #INPUT "_size"},              \
        {testing::BufferTypeOf(T, EXPECTED), #EXPECTED "_ptr"}, {__b_->getSizeTy(), #EXPECTED "_size"}      \
    });                                                                                                     \
    auto Input = testing::ToStreamSet(T, INPUT.getFieldWidth(), INPUT.getNumElements(),                     \
        T->getInputScalar(#INPUT "_ptr"), T->getInputScalar(#INPUT "_size"));                               \
    auto Expected = testing::ToStreamSet(T, EXPECTED.getFieldWidth(), EXPECTED.getNumElements(),            \
        T->getInputScalar(#EXPECTED "_ptr"), T->getInputScalar(#EXPECTED "_size"));                         \
    __test_body_##NAME(T, Input, Expected);                                                                 \
    return T.compile();                                                                                     \
}                                                                                                           \
                                                                                                            \
int32_t NAME() {                                                                                            \
    assert((void *) &INPUT != (void *) &EXPECTED && "input and expected streams cannot be the same");       \
    auto result = new int32_t(0);                                                                           \
    testing::TestEngine T{};                                                                                \
    auto fn = __gen_##NAME(T);                                                                              \
    fn(                                                                                                     \
        (const void *) INPUT.getBuffer().begin().base(),                                                    \
        INPUT.getSize(),                                                                                    \
        (const void *) EXPECTED.getBuffer().begin().base(),                                                 \
        EXPECTED.getSize(),                                                                                 \
        result                                                                                              \
    );                                                                                                      \
    auto rt = *result;                                                                                      \
    delete result;                                                                                          \
    return rt;                                                                                              \
}                                                                                                           \
                                                                                                            \
void __test_body_##NAME(testing::TestEngine & T, testing::StreamSet * Input, testing::StreamSet * Expected)


#define RUN_TESTS(...)                                                                                      \
int main(int argc, char ** argv) {                                                                          \
    codegen::ParseCommandLineOptions(argc, argv, {codegen::codegen_flags(), pablo::pablo_toolchain_flags(), testing::cli::testFlags()});    \
    return testing::RunTestSuite({__VA_ARGS__});                                                            \
}

#define CASE(X) (std::tuple<std::string, TestCaseInvocationType>(std::string(#X), X))
