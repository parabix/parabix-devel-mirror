/*
 * Copyright (c) 2019 International Characters.
 * This software is licensed to the public under the Open Software License 3.0.
 */

#pragma once

#include <string>
#include <tuple>
#include <vector>
#include <llvm/Support/CommandLine.h>

namespace testing {

namespace cli {

llvm::cl::OptionCategory * testFlags();

extern bool SuppressCLIOutput;

}

using TestCaseInvocationType = int32_t(*)();

using InvocationList = std::vector<std::tuple<std::string, TestCaseInvocationType>>;

int32_t RunTestSuite(InvocationList list);

}
