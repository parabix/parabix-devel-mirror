/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "grep_kernel.h"
#include <boost/uuid/sha1.hpp>
#include <re/printer_re.h>
#include <re/re_toolchain.h>
#include <pablo/pablo_toolchain.h>

using namespace kernel;
using namespace pablo;
using namespace re;
using namespace llvm;

inline static std::string sha1sum(const std::string & str) {
    char buffer[41];    // 40 hex-digits and the terminating null
    uint32_t digest[5]; // 160 bits in total
    boost::uuids::detail::sha1 sha1;
    sha1.process_bytes(str.c_str(), str.size());
    sha1.get_digest(digest);
    snprintf(buffer, sizeof(buffer), "%.8x%.8x%.8x%.8x%.8x",
             digest[0], digest[1], digest[2], digest[3], digest[4]);
    return std::string(buffer);
}

inline std::string makeSignature(RE * const re_ast, const bool CountOnly) {
    std::string signature = Printer_RE::PrintRE(re_ast);
    if (CountOnly) {
        signature += "-c";
    }
    if (AlgorithmOptionIsSet(InvertMatches)) {
        signature += "-v";
    }
    return signature;
}

ICgrepKernelBuilder::ICgrepKernelBuilder (IDISA::IDISA_Builder * const iBuilder, RE * const re_ast, const bool CountOnly)
: PabloKernel(iBuilder, "",
              {Binding{iBuilder->getStreamSetTy(8), "basis"}, Binding{iBuilder->getStreamSetTy(1, 1), "linebreak"}},
              CountOnly ? std::vector<Binding>{} : std::vector<Binding>{Binding{iBuilder->getStreamSetTy(1, 1), "matches"}},
              {},
              CountOnly ? std::vector<Binding>{Binding{iBuilder->getSizeTy(), "matchedLineCount"}} : std::vector<Binding>{})
, mCountOnly(CountOnly)
, mRE(re_ast)
, mSignature(makeSignature(re_ast, CountOnly)) {
    setName("Parabix:" + sha1sum(mSignature));
}

std::string ICgrepKernelBuilder::generateKernelSignature(std::string moduleId) {
    return mSignature;
}

void ICgrepKernelBuilder::prepareKernel() {
    re2pablo_compiler(this, regular_expression_passes(mRE), mCountOnly);
    pablo_function_passes(this);
    PabloKernel::prepareKernel();
}
