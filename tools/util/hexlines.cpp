/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include <cstdio>
#include <vector>
#include <llvm/Support/CommandLine.h>
#include <llvm/Support/ErrorHandling.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/IR/Module.h>
#include <re/adt/re_name.h>
#include <re/adt/re_re.h>
#include <kernel/core/kernel_builder.h>
#include <kernel/pipeline/pipeline_builder.h>
#include <kernel/streamutils/deletion.h>
#include <kernel/streamutils/pdep_kernel.h>
#include <kernel/streamutils/stream_select.h>
#include <kernel/streamutils/stream_shift.h>
#include <kernel/basis/s2p_kernel.h>
#include <kernel/basis/p2s_kernel.h>
#include <kernel/io/source_kernel.h>
#include <kernel/io/stdout_kernel.h>
#include <kernel/scan/scanmatchgen.h>
#include <re/adt/re_name.h>
#include <re/cc/cc_kernel.h>
#include <re/cc/cc_compiler.h>
#include <re/cc/cc_compiler_target.h>
#include <string>
#include <toolchain/toolchain.h>
#include <toolchain/pablo_toolchain.h>
#include <pablo/builder.hpp>
#include <pablo/pe_ones.h>
#include <pablo/pe_zeroes.h>
#include <pablo/bixnum/bixnum.h>
#include <fcntl.h>
#include <iostream>
#include <kernel/pipeline/driver/cpudriver.h>

using namespace kernel;
using namespace llvm;
using namespace pablo;

static cl::OptionCategory HexLinesOptions("hexlines Options", "hexlines control options.");
static cl::opt<std::string> inputFile(cl::Positional, cl::desc("<input file>"), cl::Required, cl::cat(HexLinesOptions));

class Hexify : public PabloKernel {
public:
    Hexify(BuilderRef kb, StreamSet * insertMask, StreamSet * spreadBasis, StreamSet * hexBasis)
        : PabloKernel(kb, "Hexify",
                      {Binding{"insertMask", insertMask}, Binding{"spreadBasis", spreadBasis}},
                      {Binding{"hexBasis", hexBasis}}) {}
protected:
    void generatePabloMethod() override;
};

void Hexify::generatePabloMethod() {
    pablo::PabloBuilder pb(getEntryScope());
    BixNumCompiler bnc(pb);
    PabloAST * insertMask = getInputStreamSet("insertMask")[0];
    std::vector<PabloAST *> spreadBasis = getInputStreamSet("spreadBasis");
    cc::Parabix_CC_Compiler_Builder ccc(getEntryScope(), spreadBasis);
    PabloAST * LF = ccc.compileCC(re::makeByte(0xA));
    
    std::vector<PabloAST *> lo(4);
    lo[3] = pb.createSel(insertMask, spreadBasis[7], pb.createAdvance(spreadBasis[3], 1), "lo[3]");
    lo[2] = pb.createSel(insertMask, spreadBasis[6], pb.createAdvance(spreadBasis[2], 1), "lo[2]");
    lo[1] = pb.createSel(insertMask, spreadBasis[5], pb.createAdvance(spreadBasis[1], 1), "lo[1]");
    lo[0] = pb.createSel(insertMask, spreadBasis[4], pb.createAdvance(spreadBasis[0], 1), "lo[0]");
    PabloAST * hexA_F = bnc.UGE(lo, 10, "hexA_F");
    BixNum lo1 = bnc.SubModular(lo, 9);
    std::vector<PabloAST *> hex(8);
    hex[7] = pb.createZeroes();
    hex[6] = hexA_F;
    hex[5] = pb.createNot(hexA_F);
    hex[4] = pb.createNot(hexA_F);
    // High 4 bits are 0x40 for hex A-F, 0x30 for hex 0-9
    for (unsigned i = 0; i < 4; i++) {
        hex[i] = pb.createSel(hexA_F, lo1[i], lo[i]);
    }
    Var * hexVar = getOutputStreamVar("hexBasis");
    for (unsigned i = 0; i < 8; i++) {
        pb.createAssign(pb.createExtract(hexVar, pb.getInteger(i)), pb.createSel(LF, spreadBasis[i], hex[i]));
    }
}

typedef void (*HexLinesFunctionType)(uint32_t fd);

HexLinesFunctionType generatePipeline(CPUDriver & pxDriver) {
    auto & b = pxDriver.getBuilder();
    auto P = pxDriver.makePipeline({Binding{b->getInt32Ty(), "inputFileDecriptor"},
                                    Binding{b->getIntAddrTy(), "callbackObject"}}, {});
    Scalar * fileDescriptor = P->getInputScalar("inputFileDecriptor");
    // File data from mmap
    StreamSet * ByteStream = P->CreateStreamSet(1, 8);
    P->CreateKernelCall<MMapSourceKernel>(fileDescriptor, ByteStream);

    StreamSet * BasisBits = P->CreateStreamSet(8);
    P->CreateKernelCall<S2PKernel>(ByteStream, BasisBits);

    StreamSet * nonLF = P->CreateStreamSet(1);
    std::vector<re::CC *> nonLF_CC = {re::makeCC(re::makeByte(0,9), re::makeByte(0xB, 0xff))};
    P->CreateKernelCall<CharacterClassKernelBuilder>(nonLF_CC, BasisBits, nonLF);
    
    StreamSet * hexInsertMask = UnitInsertionSpreadMask(P, nonLF, InsertPosition::After);
    StreamSet * spreadBasis = P->CreateStreamSet(8);
    SpreadByMask(P, hexInsertMask, BasisBits, spreadBasis);
    
    StreamSet * hexBasis = P->CreateStreamSet(8);
    P->CreateKernelCall<Hexify>(hexInsertMask, spreadBasis, hexBasis);

    StreamSet * hexLines = P->CreateStreamSet(1, 8);
    P->CreateKernelCall<P2SKernel>(hexBasis, hexLines);

    P->CreateKernelCall<StdOutKernel>(hexLines);
    return reinterpret_cast<HexLinesFunctionType>(P->compile());
}

int main(int argc, char *argv[]) {
    codegen::ParseCommandLineOptions(argc, argv, {&HexLinesOptions, pablo::pablo_toolchain_flags(), codegen::codegen_flags()});
    CPUDriver driver("hexlines");
    HexLinesFunctionType fn = generatePipeline(driver);
    const int fd = open(inputFile.c_str(), O_RDONLY);
    if (LLVM_UNLIKELY(fd == -1)) {
        llvm::errs() << "Error: cannot open " << inputFile << " for processing. Skipped.\n";
    } else {
        fn(fd);
        close(fd);
    }
    return 0;
}
