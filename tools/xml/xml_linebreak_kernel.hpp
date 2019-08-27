/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#pragma once

#include <re/adt/re_cc.h>
#include <re/cc/cc_compiler.h>
#include <re/cc/cc_compiler_target.h>
#include <pablo/pabloAST.h>
#include <pablo/pe_ones.h>
#include <pablo/pablo_kernel.h>
#include <pablo/builder.hpp>
#include <kernel/core/kernel_builder.h>
#include <kernel/pipeline/pipeline_builder.h>

namespace kernel {

/**
 * Marks all unix linebreak ('\n') positions in `basis`. Always places a bit at
 * EOF regardless of whether the input file ends in a linebreak of not. 
 */
class XmlLineBreakKernel : public pablo::PabloKernel {
public:
    using BuilderRef = const std::unique_ptr<KernelBuilder> &;

    XmlLineBreakKernel(BuilderRef b, StreamSet * basis, StreamSet * out)
    : PabloKernel(b, "XmlLineBreakKernel", {{"basis", basis}}, {{"out", out, FixedRate(), Add1()}})
    {
        assert(basis->getFieldWidth() == 8 && basis->getNumElements() == 1);
        assert(out->getFieldWidth() == 1 && out->getNumElements() == 1);
    }

    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }

    void generatePabloMethod() override {
        using namespace pablo;
        using namespace cc;
        PabloBuilder pb(getEntryScope());
        std::unique_ptr<CC_Compiler> ccc;
        ccc = llvm::make_unique<cc::Direct_CC_Compiler>(getEntryScope(), pb.createExtract(getInputStreamVar("basis"), pb.getInteger(0)));
        PabloAST * breaks = ccc->compileCC(re::makeByte('\n'));;
        PabloAST * const eofBit = pb.createAtEOF(llvm::cast<PabloAST>(pb.createOnes()));
        Var * const output = pb.createExtract(getOutputStreamVar("out"), 0);
        pb.createAssign(output, pb.createOr(breaks, eofBit));
    }
};

}

inline kernel::StreamSet * XmlLineBreaks(const std::unique_ptr<kernel::ProgramBuilder> & P, kernel::StreamSet * basis) {
    assert(basis->getFieldWidth() == 8 && basis->getNumElements() == 1);
    auto out = P->CreateStreamSet(1, 1);
    P->CreateKernelCall<kernel::XmlLineBreakKernel>(basis, out);
    return out;
}
