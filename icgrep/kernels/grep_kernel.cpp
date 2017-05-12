/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "grep_kernel.h"
#include <boost/uuid/sha1.hpp>
#include <re/printer_re.h>
#include <re/re_toolchain.h>
#include <pablo/pablo_toolchain.h>
#include <kernels/kernel_builder.h>
#include <pablo/builder.hpp>
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

ICgrepKernelBuilder::ICgrepKernelBuilder (const std::unique_ptr<kernel::KernelBuilder> & iBuilder, RE * const re)
: PabloKernel(iBuilder, "",
              {Binding{iBuilder->getStreamSetTy(8), "basis"}, Binding{iBuilder->getStreamSetTy(1, 1), "linebreak"}},
              {Binding{iBuilder->getStreamSetTy(1, 1), "matches"}},
              {},
              {})
, mRE(re)
, mSignature(Printer_RE::PrintRE(re)) {
    setName("ic" + sha1sum(mSignature));
}

std::string ICgrepKernelBuilder::makeSignature(const std::unique_ptr<kernel::KernelBuilder> &) {
    return mSignature;
}

void ICgrepKernelBuilder::generatePabloMethod() {
    re2pablo_compiler(this, regular_expression_passes(mRE));
}

void InvertMatchesKernel::generateDoBlockMethod(const std::unique_ptr<KernelBuilder> & iBuilder) {
    Value * input = iBuilder->loadInputStreamBlock("matchedLines", iBuilder->getInt32(0));
    Value * lbs = iBuilder->loadInputStreamBlock("lineBreaks", iBuilder->getInt32(0));
    Value * inverted = iBuilder->CreateXor(input, lbs);
    iBuilder->storeOutputStreamBlock("nonMatches", iBuilder->getInt32(0), inverted);
}

InvertMatchesKernel::InvertMatchesKernel(const std::unique_ptr<kernel::KernelBuilder> & builder)
: BlockOrientedKernel("Invert", {Binding{builder->getStreamSetTy(1, 1), "matchedLines"}, Binding{builder->getStreamSetTy(1, 1), "lineBreaks"}}, {Binding{builder->getStreamSetTy(1, 1), "nonMatches"}}, {}, {}, {}) {
    setNoTerminateAttribute(true);    
}


PopcountKernel::PopcountKernel (const std::unique_ptr<kernel::KernelBuilder> & iBuilder)
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
