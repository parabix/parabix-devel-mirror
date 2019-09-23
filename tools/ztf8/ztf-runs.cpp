/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <kernel/streamutils/deletion.h>                      // for DeletionKernel
#include <kernel/io/source_kernel.h>
#include <kernel/basis/p2s_kernel.h>
#include <kernel/basis/s2p_kernel.h>                    // for S2PKernel
#include <kernel/io/stdout_kernel.h>                 // for StdOutKernel_
#include <kernel/streamutils/pdep_kernel.h>
#include <llvm/IR/Function.h>                      // for Function, Function...
#include <llvm/IR/Module.h>                        // for Module
#include <llvm/Support/CommandLine.h>              // for ParseCommandLineOp...
#include <llvm/Support/Debug.h>                    // for dbgs
#include <pablo/pablo_kernel.h>                    // for PabloKernel
#include <toolchain/pablo_toolchain.h>
#include <pablo/parse/pablo_source_kernel.h>
#include <pablo/parse/pablo_parser.h>
#include <pablo/parse/simple_lexer.h>
#include <pablo/parse/rd_parser.h>
#include <re/cc/cc_kernel.h>
#include <re/cc/cc_compiler.h>         // for CC_Compiler
#include <re/cc/cc_compiler_target.h>
#include <kernel/core/kernel_builder.h>
#include <pablo/pe_zeroes.h>
#include <toolchain/toolchain.h>
#include <kernel/pipeline/driver/cpudriver.h>
#include <kernel/core/streamset.h>
#include <kernel/streamutils/run_index.h>
#include <kernel/streamutils/streams_merge.h>
#include <llvm/ADT/StringRef.h>
#include <llvm/Support/raw_ostream.h>
#include <pablo/builder.hpp>
#include <fcntl.h>
#include <iostream>
#include <iomanip>
#include <kernel/pipeline/pipeline_builder.h>

using namespace pablo;
using namespace parse;
using namespace kernel;
using namespace llvm;
using namespace codegen;

static cl::OptionCategory ztfRunsOptions("ztfRuns Options", "ZTF-runs options.");
static cl::opt<std::string> inputFile(cl::Positional, cl::desc("<input file>"), cl::Required, cl::cat(ztfRunsOptions));
static cl::opt<bool> Decompression("d", cl::desc("Decompress from ZTF-Runs to UTF-8."), cl::cat(ztfRunsOptions), cl::init(false));
static cl::alias DecompressionAlias("decompress", cl::desc("Alias for -d"), cl::aliasopt(Decompression));

class ByteRun final: public PabloKernel {
public:
    ByteRun(const std::unique_ptr<kernel::KernelBuilder> & b, StreamSet * const basis, StreamSet * runMask)
    : PabloKernel(b, "byteRun", {Binding{"basis", basis}}, {Binding{"runMask", runMask}}) {}
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;
};

void ByteRun::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    std::vector<PabloAST *> basis = getInputStreamSet("basis");
    
    PabloAST * mismatches = pb.createZeroes();
    for (unsigned i = 0; i < 8; i++) {
        mismatches = pb.createOr(mismatches,
                                 pb.createXor(basis[i], pb.createAdvance(basis[i], 1)),
                                 "mismatches_to_bit" + std::to_string(i));
    }
    PabloAST * matchesprior = pb.createInFile(pb.createNot(mismatches), "matchesprior");
    pb.createAssign(pb.createExtract(getOutputStreamVar("runMask"), pb.getInteger(0)), matchesprior);
}

class ZTF_CompressionMask final: public PabloKernel {
public:
    ZTF_CompressionMask(const std::unique_ptr<kernel::KernelBuilder> & b,
                  StreamSet * runMask, StreamSet * runIndex, StreamSet * compressionMask)
    : PabloKernel(b, "ZTF_CompressionMask",
                         {Binding{"runMask", runMask, FixedRate(1), LookAhead(1)},
                          Binding{"runIndex", runIndex}},
                         {Binding{"compressionMask", compressionMask}}) {}
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;
};

void ZTF_CompressionMask::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    Var * runMask = pb.createExtract(getInputStreamVar("runMask"), pb.getInteger(0));
    std::vector<PabloAST *> runIndex = getInputStreamSet("runIndex");
    PabloAST * idx7 = pb.createAnd3(runIndex[0], runIndex[1], runIndex[2]);
    PabloAST * runFinal = pb.createAnd(runMask, pb.createNot(pb.createLookahead(runMask, 1)));
    PabloAST * unchanged = pb.createInFile(pb.createNot(runMask));
    PabloAST * compressionMask = pb.createOr3(unchanged, idx7, runFinal);
    pb.createAssign(pb.createExtract(getOutputStreamVar("compressionMask"), pb.getInteger(0)), compressionMask);
}

class ZTF_Run_Replacement final: public PabloKernel {
public:
    ZTF_Run_Replacement(const std::unique_ptr<kernel::KernelBuilder> & b,
                        StreamSet * basis, StreamSet * runIndex, StreamSet * output)
    : PabloKernel(b, "ZTF_Run_Replacement",
                         {Binding{"basis", basis}, Binding{"runIndex", runIndex}},
                         {Binding{"output", output}}) {}
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;
};

void ZTF_Run_Replacement::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    std::vector<PabloAST *> runIndex = getInputStreamSet("runIndex");
    std::vector<PabloAST *> basis = getInputStreamSet("basis");
    PabloAST * replacementMask = pb.createOr3(runIndex[0], runIndex[1], runIndex[2]);
    Var * output = getOutputStreamVar("output");
    for (unsigned i = 0; i < 8; i++) {
        PabloAST * r = nullptr;
        if (i < 3) {
            r = pb.createSel(replacementMask, runIndex[i], basis[i]);
        } else {
            // Codes F9 through FF always have the high bit set.
            r = pb.createOr(replacementMask, basis[i]);
        }
        pb.createAssign(pb.createExtract(output, pb.getInteger(i)), r);
    }
}

class ZTF_Run_Length_Decoder final: public PabloKernel {
public:
    ZTF_Run_Length_Decoder(const std::unique_ptr<kernel::KernelBuilder> & b,
                        StreamSet * ztf_basis, StreamSet * runLengths)
    : PabloKernel(b, "ZTF_Run_Length_Decoder",
                         {Binding{"basis", ztf_basis}},
                         {Binding{"runLengths", runLengths}}),
                         mLengthBits(runLengths->getNumElements()) {}
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;
    unsigned mLengthBits;
};

void ZTF_Run_Length_Decoder::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    std::vector<PabloAST *> basis = getInputStreamSet("basis");
    std::unique_ptr<cc::CC_Compiler> ccc;
    ccc = make_unique<cc::Parabix_CC_Compiler_Builder>(getEntryScope(), basis);
    PabloAST * const runCodeCC = ccc->compileCC(re::makeByte(0xF9, 0xFF));
    Var * lengthVar = getOutputStreamVar("runLengths");
    for (unsigned i = 0; i < mLengthBits; i++) {
        PabloAST * lengthData = pb.createAnd(basis[i], runCodeCC);
        pb.createAssign(pb.createExtract(lengthVar, pb.getInteger(i)), lengthData);
    }
}

class ZTF_Run_Decompression final: public PabloKernel {
public:
    ZTF_Run_Decompression(const std::unique_ptr<kernel::KernelBuilder> & b,
                           StreamSet * ztfRunCodes, StreamSet * runMask, StreamSet * ztf_u8_indexed, StreamSet * u8output)
    : PabloKernel(b, "ZTF_Run_Decompression",
                  {Binding{"ztfRunCodes", ztfRunCodes, FixedRate(1), LookAhead(1)},
                   Binding{"runSpreadMask", runMask},
                   Binding{"ztf_u8_indexed", ztf_u8_indexed}},
                  {Binding{"u8output", u8output}}) {}
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;
};

void ZTF_Run_Decompression::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    PabloAST * runStart = getInputStreamSet("ztfRunCodes")[0];
    PabloAST * spreadMask = getInputStreamSet("runSpreadMask")[0];
    std::vector<PabloAST *> ztf_u8_indexed = getInputStreamSet("ztf_u8_indexed");
    PabloAST * inserted = pb.createNot(spreadMask, "inserted");
    PabloAST * runMask = pb.createOr(inserted, runStart, "runMask");
    Var * u8basis = getOutputStreamVar("u8output");
    for (unsigned i = 0; i < 8; i++) {
        PabloAST * bitMove = pb.createAnd(pb.createAdvance(ztf_u8_indexed[i], 1), runStart);
        PabloAST * bitCopy = pb.createMatchStar(bitMove, runMask);
        PabloAST * resultBit = pb.createSel(runMask, bitCopy, ztf_u8_indexed[i]);
        pb.createAssign(pb.createExtract(u8basis, pb.getInteger(i)), resultBit);
    }
}

typedef void (*ztfRunsFunctionType)(uint32_t fd);

ztfRunsFunctionType ztfRuns_compression_gen (CPUDriver & driver) {

    auto & b = driver.getBuilder();
    Type * const int32Ty = b->getInt32Ty();
    auto P = driver.makePipeline({Binding{int32Ty, "fd"}});

    Scalar * const fileDescriptor = P->getInputScalar("fd");

    // Source data
    StreamSet * const codeUnitStream = P->CreateStreamSet(1, 8);
    P->CreateKernelCall<MMapSourceKernel>(fileDescriptor, codeUnitStream);

    StreamSet * const u8basis = P->CreateStreamSet(8);
    P->CreateKernelCall<S2PKernel>(codeUnitStream, u8basis);
    
    StreamSet * const byteRuns = P->CreateStreamSet(1);
    P->CreateKernelCall<ByteRun>(u8basis, byteRuns);

    StreamSet * const runIndex = P->CreateStreamSet(3);
    P->CreateKernelCall<RunIndex>(byteRuns, runIndex);

    StreamSet * const replacedBasis = P->CreateStreamSet(8);
    P->CreateKernelCall<ZTF_Run_Replacement>(u8basis, runIndex, replacedBasis);
    
    StreamSet * const toExtract = P->CreateStreamSet(1);
    P->CreateKernelCall<ZTF_CompressionMask>(byteRuns, runIndex, toExtract);

    StreamSet * const ztf_basis = P->CreateStreamSet(8);
    FilterByMask(P, toExtract, replacedBasis, ztf_basis);
    StreamSet * const ZTF_bytes = P->CreateStreamSet(1, 8);
    P->CreateKernelCall<P2SKernel>(ztf_basis, ZTF_bytes);
    P->CreateKernelCall<StdOutKernel>(ZTF_bytes);
    return reinterpret_cast<ztfRunsFunctionType>(P->compile());
}

ztfRunsFunctionType ztfRuns_decompression_gen (CPUDriver & driver) {
    auto & b = driver.getBuilder();
    Type * const int32Ty = b->getInt32Ty();
    auto P = driver.makePipeline({Binding{int32Ty, "fd"}});
    Scalar * const fileDescriptor = P->getInputScalar("fd");

    // Source data
    StreamSet * const source = P->CreateStreamSet(1, 8);
    P->CreateKernelCall<MMapSourceKernel>(fileDescriptor, source);
    StreamSet * const ztfRunsBasis = P->CreateStreamSet(8);
    P->CreateKernelCall<S2PKernel>(source, ztfRunsBasis);
    StreamSet * const ztfRunLengths = P->CreateStreamSet(3);
    P->CreateKernelCall<ZTF_Run_Length_Decoder>(ztfRunsBasis, ztfRunLengths);
    StreamSet * const ztfRunSpreadMask = InsertionSpreadMask(P, ztfRunLengths, InsertPosition::After);
    StreamSet * const ztfRuns_u8_Basis = P->CreateStreamSet(8);
    SpreadByMask(P, ztfRunSpreadMask, ztfRunsBasis, ztfRuns_u8_Basis);
    StreamSet * const ztfRunCodes = P->CreateStreamSet(1);
    re::CC * const runCodeCC = re::makeByte(0xF9, 0xFF);
    P->CreateKernelCall<CharacterClassKernelBuilder>("runCodes", std::vector<re::CC *>{runCodeCC}, ztfRuns_u8_Basis, ztfRunCodes);
    StreamSet * const u8basis = P->CreateStreamSet(8);
    P->CreateKernelCall<ZTF_Run_Decompression>(ztfRunCodes, ztfRunSpreadMask, ztfRuns_u8_Basis, u8basis);
    StreamSet * const u8bytes = P->CreateStreamSet(1, 8);
    P->CreateKernelCall<P2SKernel>(u8basis, u8bytes);
    P->CreateKernelCall<StdOutKernel>(u8bytes);
    return reinterpret_cast<ztfRunsFunctionType>(P->compile());
}

int main(int argc, char *argv[]) {
    codegen::ParseCommandLineOptions(argc, argv, {&ztfRunsOptions, pablo_toolchain_flags(), codegen::codegen_flags()});
    
    CPUDriver pxDriver("ztfRuns");
    const int fd = open(inputFile.c_str(), O_RDONLY);
    if (LLVM_UNLIKELY(fd == -1)) {
        errs() << "Error: cannot open " << inputFile << " for processing. Skipped.\n";
    } else {
        if (Decompression) {
            auto ztfRunsDecompressionFunction = ztfRuns_decompression_gen(pxDriver);
            ztfRunsDecompressionFunction(fd);
        } else {
            auto ztfRunsCompressionFunction = ztfRuns_compression_gen(pxDriver);
            ztfRunsCompressionFunction(fd);
        }
        close(fd);
    }
    return 0;
}
