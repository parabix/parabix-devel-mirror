/*
 * Copyright (c) 2019 International Characters.
 * This software is licensed to the public under the Open Software License 3.0.
 */

#include <testing/testing.h>
#include <pablo/parse/pablo_source_kernel.h>
#include <pablo/parse/source_file.h>
#include <pablo/builder.hpp>

using namespace testing;

auto add1_i = BinaryStream("1 0{100} 1");
auto add1_e = BinaryStream("1 0{100} 11");

TEST_CASE(add1, add1_i, add1_e) {
    
}
