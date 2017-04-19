/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "grep_kernel.h"
#include <boost/uuid/sha1.hpp>
#include <re/printer_re.h>
#include <re/re_toolchain.h>
#include <pablo/pablo_toolchain.h>
#include <IR_Gen/idisa_builder.h>  // for IDISA_Builder
#include <pablo/builder.hpp>  // for PabloBuilder
#include <pablo/pe_count.h>

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

inline std::string makeSignature(RE * const re_ast) {
    std::string signature = Printer_RE::PrintRE(re_ast);
    return signature;
}

ICgrepKernelBuilder::ICgrepKernelBuilder (IDISA::IDISA_Builder * const iBuilder, RE * const re_ast)
: PabloKernel(iBuilder, "",
              {Binding{iBuilder->getStreamSetTy(8), "basis"}, Binding{iBuilder->getStreamSetTy(1, 1), "linebreak"}},
              {Binding{iBuilder->getStreamSetTy(1, 1), "matches"}},
              {},
              {})
, mRE(re_ast)
, mSignature(makeSignature(re_ast)) {
    setName("Parabix:" + sha1sum(mSignature));
}

std::string ICgrepKernelBuilder::generateKernelSignature(std::string moduleId) {
    return mSignature;
}

void ICgrepKernelBuilder::prepareKernel() {
    re2pablo_compiler(this, regular_expression_passes(mRE));
    pablo_function_passes(this);
    PabloKernel::prepareKernel();
}

void InvertMatchesKernel::generateDoBlockMethod() {
    Value * input = loadInputStreamBlock("matchedLines", iBuilder->getInt32(0));
    Value * lbs = loadInputStreamBlock("lineBreaks", iBuilder->getInt32(0));
    Value * inverted = iBuilder->CreateXor(input, lbs);
    storeOutputStreamBlock("nonMatches", iBuilder->getInt32(0), inverted);
}

InvertMatchesKernel::InvertMatchesKernel(IDISA::IDISA_Builder * builder)
: BlockOrientedKernel(builder, "Invert", {Binding{builder->getStreamSetTy(1, 1), "matchedLines"}, Binding{builder->getStreamSetTy(1, 1), "lineBreaks"}}, {Binding{builder->getStreamSetTy(1, 1), "nonMatches"}}, {}, {}, {}) {
    setNoTerminateAttribute(true);
    
}


PopcountKernel::PopcountKernel (IDISA::IDISA_Builder * const iBuilder)
: PabloKernel(iBuilder, "Popcount",
              {Binding{iBuilder->getStreamSetTy(1), "toCount"}},
              {},
              {},
              {Binding{iBuilder->getSizeTy(), "countResult"}}) {
    
    auto pb = this->getEntryBlock();
    const auto toCount = pb->createExtract(getInputStreamVar("toCount"), pb->getInteger(0));
    pablo::Var * countResult = getOutputScalarVar("countResult");
    pb->createAssign(countResult, pb->createCount(toCount));
}
