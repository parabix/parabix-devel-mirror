/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */


#include <IR_Gen/idisa_target.h>

#include <cc/alphabet.h>
#include <cc/cc_compiler.h>
#include <cc/cc_kernel.h>
#include <kernels/deletion.h>
#include <kernels/kernel_builder.h>
#include <kernels/p2s_kernel.h>
#include <kernels/pipeline_builder.h>
#include <kernels/s2p_kernel.h>
#include <kernels/source_kernel.h>
#include <kernels/stdout_kernel.h>
#include <kernels/swizzle.h>
#include <kernels/zeroextend.h>
#include <pablo/builder.hpp>
#include <pablo/pablo_kernel.h>
#include <pablo/pablo_toolchain.h>
#include <pablo/pe_zeroes.h>
#include <toolchain/cpudriver.h>
#include <toolchain/toolchain.h>

#include <sys/stat.h>
#include <fcntl.h>
#include <iostream>

using namespace pablo;
using namespace kernel;
using namespace llvm;
using namespace codegen;
using namespace re;


static cl::OptionCategory x8u16Options("x8u16 Options", "Transcoding control options.");
static cl::opt<std::string> inputFile(cl::Positional, cl::desc("<input file>"), cl::Required, cl::cat(x8u16Options));
static cl::opt<std::string> outputFile(cl::Positional, cl::desc("<output file>"), cl::cat(x8u16Options));

  
/*  transcoderClasses(sourceCharSet) determines character classes for bitwise
    transformation of streams in a source character set to Unicode equivalents.
    Given that the source character set is encoded in K bits and the maximum
    Unicode codepoint represented requires M >= K bits, two sets of classes
    are returned:
    (a)  for each bit i of the K bits of the source character set, the class
         representing the source characters such that bit i of the Unicode
         representation differs from bit i of the source code representation,
    (b)  for each bit i of the bits from K to M of the Unicode representation,
         the class of source characters which have that bit set in the Unicode 
         representation.
*/

std::pair<std::vector<CC *>, std::vector<CC *>> transcoderClasses(UnicodeMappableAlphabet * a) {
    unsigned K = a->getEncodingBits();
    unsigned M = a->getUnicodeBits();
    UnicodeSet bitXfrmSets[K];
    UnicodeSet outputBitSets[K-M];

    for (unsigned ch_code = max_common + 1; ch_code <= alphabet_max; ch_code++) {
        transcodedCh = transcodeTable[ch_code - max_common - 1];
        changedBits = transcodedCh ^ ch_code;
        for (unsigned i = 0; i < K; i++) {
            unsigned bit = 1 << i;
            if ((changedBits & bit) == bit) {
                bitXfrmSets[i].insert(ch_code);
            }
        }
        for (unsigned i = K; i < M; i++) {
            if ((transcodedCh & bit) == bit) {
                outputBitSets[i-K].insert(ch_code);
            }
        }
    }
    std::vector<CC *> bitXfrmClasses;
    bitXfrmClasses.reserve(K);
    for (unsigned i = 0; i < K; i++) {
        bitXfrmClasses[i] = makeCC(bitXfrmSets[i], a);
    }
    std::vector<CC *> outputBitClasses;
    outputBitClasses.reserve(M-K);
    for (unsigned i = 0; i < M-K; i++) {
        outputBitClasses[i] = makeCC(outputBitSets[i], a);
    }
    return std::make_pair<std::vector<CC *>, std::vector<CC *>>(bitXfrmClasses, outputBitClasses);
}

class TranscoderKernelBuilder : public PabloKernel {
public:
    TranscoderKernelBuilder(const std::unique_ptr<kernel::KernelBuilder> & b, cc::Alphabet & a);
};

TranscoderKernelBuilder::TranscoderKernelBuilder(
        const std::unique_ptr<kernel::KernelBuilder> & b, cc::Alphabet & a, StreamSet * sourceBasis, StreamSet * UnicodeBasis)
: PabloKernel(b, a->getName() + "ToUnicode"),
{Binding{"sourceBasis", b->CreateStreamSet(a->getEncodingBits())}},
{Binding{"UnicodeBasis", b->CreateStreamSet(16)}},
{}, {}), mAlphabet(a) {}


void TranscoderKernelBuilder::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    std::vector<PabloAST *> basis = getInputStreamSet("sourceBasis");
    Var * outputVar = getOutputStreamVar("UnicodeBasis")
    Parabix_CC_Compiler ccc(getEntryScope(), basis);
    std::vector<CC *> bitXfrmClasses;
    std::vector<CC *> outputBitClasses;
    auto xClasses = transcoderClasses(mAlphabet);
    bitXfrmClasses = xClasses.first;
    outputBitClasses = xClasses.second;
    unsigned K = bitXfrmClasses.size();
    for (unsigned i = 0; K; i++) {
        PabloAST * xfrmStrm = ccc.compile(bitXfrmClasses[i]);
        PabloAST * outStrm = pablo.createXor(xfrmStrm, basis[i]);
        pb.createAssign(pb.createExtract(outputVar, i), outStrm);
    }
    for (unsigned i = 0; i < outputBitClasses.size(); i++) {
        PabloAST * outStrm = ccc.compile(outputBitClasses[i]);
        PabloAST * pb.createAssign(pb.createExtract(outputVar, K + i), outStrm);
    }
    for (unsigned i = outputBitClasses.size(); i < 16; i++) {
        PabloAST * pb.createAssign(pb.createExtract(outputVar, K + i), pb.createZeroes());
    }
    
}

typedef void (*x8u16FunctionType)(uint32_t fd, const char *);

x8u16FunctionType generatePipeline(CPUDriver & pxDriver) {

    auto & b = pxDriver.getBuilder();
    auto P = pxDriver.makePipeline({Binding{b->getInt32Ty(), "inputFileDecriptor"}, Binding{b->getInt8PtrTy(), "outputFileName"}}, {});
    Scalar * fileDescriptor = P->getInputScalar("inputFileDecriptor");
    // File data from mmap
    StreamSet * const ByteStream = P->CreateStreamSet(1, 8);
    P->CreateKernelCall<MMapSourceKernel>(fileDescriptor, ByteStream);

    // Transposed bits from s2p
    StreamSet * BasisBits = P->CreateStreamSet(8);
    P->CreateKernelCall<S2PKernel>(ByteStream, BasisBits);

    StreamSet * u16bits = P->CreateStreamSet(16);
    
    P->CreateKernelCall<TranscoderKernelBuilder>(BasisBits, u16bits);
    
    StreamSet * UTF16_out = P->CreateStreamSet(1, 16);
    
    P->CreateKernelCall<P2S16Kernel>(u16bits, UTF16_out);
    
    Scalar * outputFileName = P->getInputScalar("outputFileName");
    P->CreateKernelCall<FileSink>(outputFileName, u16bytes);

    return reinterpret_cast<x8u16FunctionType>(P->compile());
}

size_t file_size(const int fd) {
    struct stat st;
    if (LLVM_UNLIKELY(fstat(fd, &st) != 0)) {
        st.st_size = 0;
    }
    return st.st_size;
}

int main(int argc, char *argv[]) {
    codegen::ParseCommandLineOptions(argc, argv, {&x8u16Options, pablo::pablo_toolchain_flags(), codegen::codegen_flags()});
    CPUDriver pxDriver("x8u16");
    x8u16FunctionType x8u16Function = generatePipeline(pxDriver);
    const int fd = open(inputFile.c_str(), O_RDONLY);
    if (LLVM_UNLIKELY(fd == -1)) {
        std::cerr << "Error: cannot open " << inputFile << " for processing. Skipped.\n";
    } else {
        x8u16Function(fd, outputFile.c_str());
        close(fd);
    }
    return 0;
}
