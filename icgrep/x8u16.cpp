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

std::pair<std::vector<CC *>, std::vector<CC *>> transcoderClasses(cc::UnicodeMappableAlphabet & a) {
    unsigned common = a.getCommon();
    unsigned total = a.getSize();
    if (total != 256) llvm::report_fatal_error("Not single byte extended ASCII");
    unsigned K = std::log2(common);
    unsigned M = 16;
    std::vector<CC *> bitXfrmClasses;
    bitXfrmClasses.reserve(K);
    for (unsigned i = 0; i < K; i++) {
        bitXfrmClasses.push_back(makeCC(&a));
    }
    std::vector<CC *> outputBitClasses;
    outputBitClasses.reserve(M-K);
    for (unsigned i = K; i < M; i++) {
        outputBitClasses.push_back(makeCC(&a));
    }

    for (unsigned ch_code = common; ch_code < total; ch_code++) {
        codepoint_t transcodedCh = a.toUnicode(ch_code);
        if (transcodedCh >= 1<<16) llvm::report_fatal_error("Above BMP!");
        codepoint_t changedBits = transcodedCh ^ ch_code;
        for (unsigned i = 0; i < K; i++) {
            unsigned bit = 1 << i;
            if ((changedBits & bit) == bit) {
                bitXfrmClasses[i]->insert(ch_code);
            }
        }
        for (unsigned i = K; i < M; i++) {
            unsigned bit = 1 << i;
            if ((transcodedCh & bit) == bit) {
                outputBitClasses[i-K]->insert(ch_code);
            }
        }
    }
    return std::make_pair(bitXfrmClasses, outputBitClasses);
}


//
// iso_8859_5.cpp - Legacy character set as UnicodeMappableAlphabet
//
namespace iso_8859_5 {
    std::vector<codepoint_t> codepoints =
    {0x0080, 0x0081, 0x0082, 0x0083, 0x0084, 0x0085, 0x0086, 0x0087,
        0x0088, 0x0089, 0x008a, 0x008b, 0x008c, 0x008d, 0x008e, 0x008f,
        0x0090, 0x0091, 0x0092, 0x0093, 0x0094, 0x0095, 0x0096, 0x0097,
        0x0098, 0x0099, 0x009a, 0x009b, 0x009c, 0x009d, 0x009e, 0x009f,
        0x00a0, 0x0401, 0x0402, 0x0403, 0x0404, 0x0405, 0x0406, 0x0407,
        0x0408, 0x0409, 0x040a, 0x040b, 0x040c, 0x00ad, 0x040e, 0x040f,
        0x0410, 0x0411, 0x0412, 0x0413, 0x0414, 0x0415, 0x0416, 0x0417,
        0x0418, 0x0419, 0x041a, 0x041b, 0x041c, 0x041d, 0x041e, 0x041f,
        0x0420, 0x0421, 0x0422, 0x0423, 0x0424, 0x0425, 0x0426, 0x0427,
        0x0428, 0x0429, 0x042a, 0x042b, 0x042c, 0x042d, 0x042e, 0x042f,
        0x0430, 0x0431, 0x0432, 0x0433, 0x0434, 0x0435, 0x0436, 0x0437,
        0x0438, 0x0439, 0x043a, 0x043b, 0x043c, 0x043d, 0x043e, 0x043f,
        0x0440, 0x0441, 0x0442, 0x0443, 0x0444, 0x0445, 0x0446, 0x0447,
        0x0448, 0x0449, 0x044a, 0x044b, 0x044c, 0x044d, 0x044e, 0x044f,
        0x2116, 0x0451, 0x0452, 0x0453, 0x0454, 0x0455, 0x0456, 0x0457,
        0x0458, 0x0459, 0x045a, 0x045b, 0x045c, 0x00a7, 0x045e, 0x045f};
    
    cc::UnicodeMappableAlphabet alphabet("iso-8859-5", 128, codepoints);
    
}


class TranscoderKernelBuilder : public PabloKernel {
public:
    TranscoderKernelBuilder(const std::unique_ptr<kernel::KernelBuilder> & b, cc::UnicodeMappableAlphabet & a, StreamSet * sourceBasis, StreamSet * UnicodeBasis);
    void generatePabloMethod() override;
private:
    cc::UnicodeMappableAlphabet & mAlphabet;
};

TranscoderKernelBuilder::TranscoderKernelBuilder(
        const std::unique_ptr<kernel::KernelBuilder> & b, cc::UnicodeMappableAlphabet & a, StreamSet * sourceBasis, StreamSet * UnicodeBasis)
: PabloKernel(b, a.getName() + "ToUnicode",
{Binding{"sourceBasis", sourceBasis}},
{Binding{"UnicodeBasis", UnicodeBasis}},
{}, {}), mAlphabet(a) {}


void TranscoderKernelBuilder::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    std::vector<PabloAST *> basis = getInputStreamSet("sourceBasis");
    Var * outputVar = getOutputStreamVar("UnicodeBasis");
    cc::Parabix_CC_Compiler ccc(getEntryScope(), basis);
    std::vector<CC *> bitXfrmClasses;
    std::vector<CC *> outputBitClasses;
    auto xClasses = transcoderClasses(mAlphabet);
    bitXfrmClasses = xClasses.first;
    outputBitClasses = xClasses.second;
    unsigned K = bitXfrmClasses.size();
    for (unsigned i = 0; i < K; i++) {
        PabloAST * xfrmStrm = ccc.compileCC(bitXfrmClasses[i]);
        PabloAST * outStrm = pb.createXor(xfrmStrm, basis[i]);
        pb.createAssign(pb.createExtract(outputVar, i), outStrm);
    }
    for (unsigned i = 0; i < outputBitClasses.size(); i++) {
        PabloAST * outStrm = ccc.compileCC(outputBitClasses[i]);
        pb.createAssign(pb.createExtract(outputVar, K + i), outStrm);
    }
    for (unsigned i = outputBitClasses.size(); i < 16; i++) {
        pb.createAssign(pb.createExtract(outputVar, K + i), pb.createZeroes());
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
    
    P->CreateKernelCall<TranscoderKernelBuilder>(iso_8859_5::alphabet, BasisBits, u16bits);
    
    StreamSet * UTF16_out = P->CreateStreamSet(1, 16);
    
    P->CreateKernelCall<P2S16Kernel>(u16bits, UTF16_out);
    
    Scalar * outputFileName = P->getInputScalar("outputFileName");
    P->CreateKernelCall<FileSink>(outputFileName, UTF16_out);

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
