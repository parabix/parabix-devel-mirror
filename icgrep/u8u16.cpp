/*
 *  Copyright (c) 2016 International Characters.
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

static cl::OptionCategory u8u16Options("u8u16 Options", "Transcoding control options.");
static cl::opt<std::string> inputFile(cl::Positional, cl::desc("<input file>"), cl::Required, cl::cat(u8u16Options));
static cl::opt<std::string> outputFile(cl::Positional, cl::desc("<output file>"), cl::cat(u8u16Options));
static cl::opt<bool> enableAVXdel("enable-AVX-deletion", cl::desc("Enable AVX2 deletion algorithms."), cl::cat(u8u16Options));

static cl::opt<bool> BranchingMode("branch", cl::desc("Use Experimental branching pipeline mode"), cl::cat(u8u16Options));

inline bool useAVX2() {
    return enableAVXdel && AVX2_available() && codegen::BlockSize == 256;
}

class U8U16Kernel final: public pablo::PabloKernel {
public:
    U8U16Kernel(const std::unique_ptr<kernel::KernelBuilder> & b, StreamSet * BasisBits, StreamSet * u8bits, StreamSet * DelMask);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
    void generatePabloMethod() override;
};

U8U16Kernel::U8U16Kernel(const std::unique_ptr<kernel::KernelBuilder> & b, StreamSet * BasisBits, StreamSet * u8bits, StreamSet * selectors)
: PabloKernel(b, "u8u16",
// input
{Binding{"u8bit", BasisBits}},
// outputs
{Binding{"u16bit", u8bits},
 Binding{"selectors", selectors}}) {

}

void U8U16Kernel::generatePabloMethod() {
    PabloBuilder main(getEntryScope());
    Zeroes * zeroes = main.createZeroes();

    //  input: 8 basis bit streams
    std::vector<PabloAST *> u8_bits = getInputStreamSet("u8bit");

    //  output: 16 u8-indexed streams, + delmask stream + error stream
    Var * u16_hi[8];
    for (int i = 0; i < 8; ++i) {
        u16_hi[i] = main.createVar("u16_hi" + std::to_string(i), zeroes);
    }
    Var * u16_lo[8];
    for (int i = 0; i < 8; ++i) {
        u16_lo[i] = main.createVar("u16_lo" + std::to_string(i), zeroes);
    }

    Var * delmask = main.createVar("delmask", zeroes);
    Var * error_mask = main.createVar("error_mask", zeroes);

    cc::Parabix_CC_Compiler ccc(getEntryScope(), u8_bits);

    // The logic for processing non-ASCII bytes will be embedded within an if-hierarchy.
    PabloAST * nonASCII = ccc.compileCC(makeByte(0x80, 0xFF));

    // Builder for the if statement handling all non-ASCII logic
    auto nAb = main.createScope();
    // Bits 3 through LE0 of a 2-byte prefix are data bits, needed to
    // produce the UTF-16 code unit data ...,
    PabloAST * bit4a1 = nAb.createAdvance(u8_bits[4], 1);
    PabloAST * bit3a1 = nAb.createAdvance(u8_bits[3], 1);
    PabloAST * bit2a1 = nAb.createAdvance(u8_bits[2], 1);
    PabloAST * bit1a1 = nAb.createAdvance(u8_bits[1], 1);
    PabloAST * bit0a1 = nAb.createAdvance(u8_bits[0], 1);

    // Entry condition for 3 or 4 byte sequences: we have a prefix byte in the range 0xE0-0xFF.
    PabloAST * pfx34 = ccc.compileCC(makeByte(0xE0, 0xFF), nAb);
    // Builder for the if statement handling all logic for 3- and 4-byte sequences.
    auto p34b = nAb.createScope();
    // Bits LE3 through LE0 of a 3-byte prefix are data bits.  They must be moved
    // to the final position of the 3-byte sequence.
    PabloAST * bit5a1 = p34b.createAdvance(u8_bits[5], 1);
    PabloAST * bit3a2 = p34b.createAdvance(bit3a1, 1);
    PabloAST * bit2a2 = p34b.createAdvance(bit2a1, 1);
    PabloAST * bit1a2 = p34b.createAdvance(bit1a1, 1);
    PabloAST * bit0a2 = p34b.createAdvance(bit0a1, 1);

    Var * const u8scope32 = nAb.createVar("u8scope32", zeroes);
    Var * const u8scope33 = nAb.createVar("u8scope33", zeroes);
    Var * const u8scope44 = nAb.createVar("u8scope44", zeroes);

    //
    // Logic for 4-byte UTF-8 sequences
    //
    // Entry condition  or 4 byte sequences: we have a prefix byte in the range 0xF0-0xFF.
    PabloAST * pfx4 = ccc.compileCC(makeByte(0xF0, 0xFF), p34b);
    // Builder for the if statement handling all logic for 4-byte sequences only.
    auto p4b = p34b.createScope();
    // Illegal 4-byte sequences
    PabloAST * F0 = ccc.compileCC(makeByte(0xF0), p4b);
    PabloAST * F4 = ccc.compileCC(makeByte(0xF4), p4b);
    PabloAST * F0_err = p4b.createAnd(p4b.createAdvance(F0, 1), ccc.compileCC(makeByte(0x80, 0x8F), p4b));
    PabloAST * F4_err = p4b.createAnd(p4b.createAdvance(F4, 1), ccc.compileCC(makeByte(0x90, 0xBF), p4b));
    PabloAST * F5_FF = ccc.compileCC(makeByte(0xF5, 0xFF), p4b);

    Var * FX_err = p34b.createVar("FX_err", zeroes);
    p4b.createAssign(FX_err, p4b.createOr(F5_FF, p4b.createOr(F0_err, F4_err)));
    //
    // 4-byte prefixes have a scope that extends over the next 3 bytes.

    Var * u8scope42 = p34b.createVar("u8scope42", zeroes);
    Var * u8scope43 = p34b.createVar("u8scope43", zeroes);

    p4b.createAssign(u8scope42, p4b.createAdvance(pfx4, 1));
    p4b.createAssign(u8scope43, p4b.createAdvance(u8scope42, 1));
    p4b.createAssign(u8scope44, p4b.createAdvance(u8scope43, 1));
    //

    //  From the 4-byte sequence 11110abc 10defghi 10jklmno 10pqrstu,
    //  we must calculate the value abcde - 1 to produce the bit values
    //  for u16_hi1, hi0, lo7, lo6 at the scope43 position.
    Var * s43_lo7 = nAb.createVar("scope43_lo7", zeroes);
    Var * s43_lo6 = nAb.createVar("scope43_lo6", zeroes);
    Var * s43_hi1 = nAb.createVar("scope43_hi1", zeroes);
    Var * s43_hi0 = nAb.createVar("scope43_hi0", zeroes);

    Var * s43_lo5 = main.createVar("scope43_lo5", zeroes);
    Var * s43_lo4 = main.createVar("scope43_lo4", zeroes);
    Var * s43_lo3 = main.createVar("scope43_lo3", zeroes);
    Var * s43_lo2 = main.createVar("scope43_lo2", zeroes);
    Var * s43_lo1 = main.createVar("scope43_lo1", zeroes);
    Var * s43_lo0 = main.createVar("scope43_lo0", zeroes);

    p4b.createAssign(s43_lo6, p4b.createAnd(u8scope43, p4b.createNot(bit4a1)));           // e - 1
    p4b.createAssign(s43_lo7, p4b.createAnd(u8scope43, p4b.createXor(bit5a1, s43_lo6)));  // d - borrow
    PabloAST * brw1 = p4b.createAnd(s43_lo6, p4b.createNot(bit5a1));
    p4b.createAssign(s43_hi0, p4b.createAnd(u8scope43, p4b.createXor(bit0a2, brw1)));     // c - borrow
    PabloAST * brw2 = p4b.createAnd(brw1, p4b.createNot(bit0a2));
    p4b.createAssign(s43_hi1, p4b.createAnd(u8scope43, p4b.createXor(bit1a2, brw2)));     // b - borrow
    //
    p4b.createAssign(s43_lo5, p4b.createAnd(u8scope43, bit3a1));
    p4b.createAssign(s43_lo4, p4b.createAnd(u8scope43, bit2a1));
    p4b.createAssign(s43_lo3, p4b.createAnd(u8scope43, bit1a1));
    p4b.createAssign(s43_lo2, p4b.createAnd(u8scope43, bit0a1));
    p4b.createAssign(s43_lo1, p4b.createAnd(u8scope43, u8_bits[5]));
    p4b.createAssign(s43_lo0, p4b.createAnd(u8scope43, u8_bits[4]));
    //
    //
    p34b.createIf(pfx4, p4b);
    //
    // Combined logic for 3 and 4 byte sequences
    //
    PabloAST * pfx3 = ccc.compileCC(makeByte(0xE0, 0xEF), p34b);

    p34b.createAssign(u8scope32, p34b.createAdvance(pfx3, 1));
    p34b.createAssign(u8scope33, p34b.createAdvance(u8scope32, 1));

    // Illegal 3-byte sequences
    PabloAST * E0 = ccc.compileCC(makeByte(0xE0), p34b);
    PabloAST * ED = ccc.compileCC(makeByte(0xED), p34b);
    PabloAST * E0_err = p34b.createAnd(p34b.createAdvance(E0, 1), ccc.compileCC(makeByte(0x80, 0x9F), p34b));
    PabloAST * ED_err = p34b.createAnd(p34b.createAdvance(ED, 1), ccc.compileCC(makeByte(0xA0, 0xBF), p34b));
    Var * EX_FX_err = nAb.createVar("EX_FX_err", zeroes);

    p34b.createAssign(EX_FX_err, p34b.createOr(p34b.createOr(E0_err, ED_err), FX_err));
    // Two surrogate UTF-16 units are computed at the 3rd and 4th positions of 4-byte sequences.
    PabloAST * surrogate = p34b.createOr(u8scope43, u8scope44);

    Var * p34del = nAb.createVar("p34del", zeroes);
    p34b.createAssign(p34del, p34b.createOr(u8scope32, u8scope42));


    // The high 5 bits of the UTF-16 code unit are only nonzero for 3 and 4-byte
    // UTF-8 sequences.
    p34b.createAssign(u16_hi[7], p34b.createOr(p34b.createAnd(u8scope33, bit3a2), surrogate));
    p34b.createAssign(u16_hi[6], p34b.createOr(p34b.createAnd(u8scope33, bit2a2), surrogate));
    p34b.createAssign(u16_hi[5], p34b.createAnd(u8scope33, bit1a2));
    p34b.createAssign(u16_hi[4], p34b.createOr(p34b.createAnd(u8scope33, bit0a2), surrogate));
    p34b.createAssign(u16_hi[3], p34b.createOr(p34b.createAnd(u8scope33, bit5a1), surrogate));

    //
    nAb.createIf(pfx34, p34b);
    //
    // Combined logic for 2, 3 and 4 byte sequences
    //

    Var * u8lastscope = main.createVar("u8lastscope", zeroes);

    PabloAST * pfx2 = ccc.compileCC(makeByte(0xC0, 0xDF), nAb);
    PabloAST * u8scope22 = nAb.createAdvance(pfx2, 1);
    nAb.createAssign(u8lastscope, nAb.createOr(u8scope22, nAb.createOr(u8scope33, u8scope44)));
    PabloAST * u8anyscope = nAb.createOr(u8lastscope, p34del);

    PabloAST * C0_C1_err = ccc.compileCC(makeByte(0xC0, 0xC1), nAb);
    PabloAST * scope_suffix_mismatch = nAb.createXor(u8anyscope, ccc.compileCC(makeByte(0x80, 0xBF), nAb));
    nAb.createAssign(error_mask, nAb.createOr(scope_suffix_mismatch, nAb.createOr(C0_C1_err, EX_FX_err)));
    nAb.createAssign(delmask, nAb.createOr(p34del, ccc.compileCC(makeByte(0xC0, 0xFF), nAb)));

    // The low 3 bits of the high byte of the UTF-16 code unit as well as the high bit of the
    // low byte are only nonzero for 2, 3 and 4 byte sequences.
    nAb.createAssign(u16_hi[2], nAb.createOr(nAb.createAnd(u8lastscope, bit4a1), u8scope44));
    nAb.createAssign(u16_hi[1], nAb.createOr(nAb.createAnd(u8lastscope, bit3a1), s43_hi1));
    nAb.createAssign(u16_hi[0], nAb.createOr(nAb.createAnd(u8lastscope, bit2a1), s43_hi0));
    nAb.createAssign(u16_lo[7], nAb.createOr(nAb.createAnd(u8lastscope, bit1a1), s43_lo7));

    Var * p234_lo6 = main.createVar("p234_lo6", zeroes);

    nAb.createAssign(p234_lo6, nAb.createOr(nAb.createAnd(u8lastscope, bit0a1), s43_lo6));

    main.createIf(nonASCII, nAb);
    //
    //
    PabloAST * ASCII = ccc.compileCC(makeByte(0x0, 0x7F));
    PabloAST * last_byte = main.createOr(ASCII, u8lastscope);
    main.createAssign(u16_lo[6], main.createOr(main.createAnd(ASCII, u8_bits[6]), p234_lo6));
    main.createAssign(u16_lo[5], main.createOr(main.createAnd(last_byte, u8_bits[5]), s43_lo5));
    main.createAssign(u16_lo[4], main.createOr(main.createAnd(last_byte, u8_bits[4]), s43_lo4));
    main.createAssign(u16_lo[3], main.createOr(main.createAnd(last_byte, u8_bits[3]), s43_lo3));
    main.createAssign(u16_lo[2], main.createOr(main.createAnd(last_byte, u8_bits[2]), s43_lo2));
    main.createAssign(u16_lo[1], main.createOr(main.createAnd(last_byte, u8_bits[1]), s43_lo1));
    main.createAssign(u16_lo[0], main.createOr(main.createAnd(last_byte, u8_bits[0]), s43_lo0));

    Var * output = getOutputStreamVar("u16bit");
    for (unsigned i = 0; i < 8; i++) {
        main.createAssign(main.createExtract(output, i + 8), u16_hi[i]);
    }
    for (unsigned i = 0; i < 8; i++) {
        main.createAssign(main.createExtract(output, i), u16_lo[i]);
    }
    PabloAST * selectors = main.createInFile(main.createNot(delmask));
    main.createAssign(main.createExtract(getOutputStreamVar("selectors"), main.getInteger(0)), selectors);
}

typedef void (*u8u16FunctionType)(uint32_t fd, const char *);

// ------------------------------------------------------

u8u16FunctionType generatePipeline(CPUDriver & pxDriver) {

    auto & b = pxDriver.getBuilder();
    auto P = pxDriver.makePipeline({Binding{b->getInt32Ty(), "inputFileDecriptor"}, Binding{b->getInt8PtrTy(), "outputFileName"}}, {});
    Scalar * fileDescriptor = P->getInputScalar("inputFileDecriptor");
    // File data from mmap
    StreamSet * const ByteStream = P->CreateStreamSet(1, 8);
    P->CreateKernelCall<MMapSourceKernel>(fileDescriptor, ByteStream);

    // Transposed bits from s2p
    StreamSet * BasisBits = P->CreateStreamSet(8);
    P->CreateKernelCall<S2PKernel>(ByteStream, BasisBits);

    // Calculate UTF-16 data bits through bitwise logic on u8-indexed streams.
    StreamSet * u8bits = P->CreateStreamSet(16);
    StreamSet * selectors = P->CreateStreamSet();
    P->CreateKernelCall<U8U16Kernel>(BasisBits, u8bits, selectors);

    StreamSet * u16bits = P->CreateStreamSet(16);
    StreamSet * u16bytes = P->CreateStreamSet(1, 16);
    if (useAVX2()) {
        // Allocate space for fully compressed swizzled UTF-16 bit streams
        std::vector<StreamSet *> u16Swizzles(4);
        u16Swizzles[0] = P->CreateStreamSet(4);
        u16Swizzles[1] = P->CreateStreamSet(4);
        u16Swizzles[2] = P->CreateStreamSet(4);
        u16Swizzles[3] = P->CreateStreamSet(4);

        // Apply a deletion algorithm to discard all but the final position of the UTF-8
        // sequences (bit streams) for each UTF-16 code unit. Also compresses and swizzles the result.
        P->CreateKernelCall<SwizzledDeleteByPEXTkernel>(selectors, u8bits, u16Swizzles);
        // Produce unswizzled UTF-16 bit streams
        P->CreateKernelCall<SwizzleGenerator>(u16Swizzles, std::vector<StreamSet *>{u16bits});
        P->CreateKernelCall<P2S16Kernel>(u16bits, u16bytes);
    } else {
        P->CreateKernelCall<FieldCompressKernel>(b->getBitBlockWidth()/16, u8bits, selectors, u16bits);
        P->CreateKernelCall<P2S16KernelWithCompressedOutput>(u16bits, selectors, u16bytes);
    }

    Scalar * outputFileName = P->getInputScalar("outputFileName");
    P->CreateKernelCall<FileSink>(outputFileName, u16bytes);

    return reinterpret_cast<u8u16FunctionType>(P->compile());
}

// ------------------------------------------------------

void makeNonAsciiBranch(const std::unique_ptr<PipelineBuilder> & P, const unsigned FieldWidth, StreamSet * const ByteStream, StreamSet * const u16bytes) {

    // Transposed bits from s2p
    StreamSet * BasisBits = P->CreateStreamSet(8);
    P->CreateKernelCall<S2PKernel>(ByteStream, BasisBits);

    // Calculate UTF-16 data bits through bitwise logic on u8-indexed streams.
    StreamSet * u8bits = P->CreateStreamSet(16);
    StreamSet * selectors = P->CreateStreamSet();
    P->CreateKernelCall<U8U16Kernel>(BasisBits, u8bits, selectors);

    StreamSet * u16bits = P->CreateStreamSet(16);
    if (useAVX2()) {
        // Allocate space for fully compressed swizzled UTF-16 bit streams
        std::vector<StreamSet *> u16Swizzles(4);
        u16Swizzles[0] = P->CreateStreamSet(4);
        u16Swizzles[1] = P->CreateStreamSet(4);
        u16Swizzles[2] = P->CreateStreamSet(4);
        u16Swizzles[3] = P->CreateStreamSet(4);
        // Apply a deletion algorithm to discard all but the final position of the UTF-8
        // sequences (bit streams) for each UTF-16 code unit. Also compresses and swizzles the result.
        P->CreateKernelCall<SwizzledDeleteByPEXTkernel>(selectors, u8bits, u16Swizzles);
        // Produce unswizzled UTF-16 bit streams
        P->CreateKernelCall<SwizzleGenerator>(u16Swizzles, std::vector<StreamSet *>{u16bits});
        P->CreateKernelCall<P2S16Kernel>(u16bits, u16bytes);
    } else {
        P->CreateKernelCall<FieldCompressKernel>(FieldWidth, u8bits, selectors, u16bits);
        P->CreateKernelCall<P2S16KernelWithCompressedOutput>(u16bits, selectors, u16bytes);
    }
}

void makeAllAsciiBranch(const std::unique_ptr<PipelineBuilder> & P, StreamSet * const ByteStream, StreamSet * const u16bytes) {
    P->CreateKernelCall<ZeroExtend>(ByteStream, u16bytes);
}

u8u16FunctionType generatePipeline2(CPUDriver & pxDriver) {

    auto & b = pxDriver.getBuilder();
    auto P = pxDriver.makePipeline({Binding{b->getInt32Ty(), "inputFileDecriptor"}, Binding{b->getInt8PtrTy(), "outputFileName"}}, {});
    Scalar * fileDescriptor = P->getInputScalar("inputFileDecriptor");
    // File data from mmap
    StreamSet * const ByteStream = P->CreateStreamSet(1, 8);
    StreamSet * const u16bytes = P->CreateStreamSet(1, 16);
    P->CreateKernelCall<MMapSourceKernel>(fileDescriptor, ByteStream);

    StreamSet * const nonAscii =  P->CreateStreamSet();

    CC * const nonAsciiCC = makeByte(0x80, 0xFF);
    P->CreateKernelCall<CharacterClassKernelBuilder>(
        "nonASCII", std::vector<CC *>{nonAsciiCC}, ByteStream, nonAscii);

    auto B = P->CreateOptimizationBranch(nonAscii,
        {Binding{"ByteStream", ByteStream}}, {Binding{"u16bytes", u16bytes, BoundedRate(0, 1)}});

    makeNonAsciiBranch(B->getNonZeroBranch(), b->getBitBlockWidth() / 16, ByteStream, u16bytes);

    makeAllAsciiBranch(B->getAllZeroBranch(), ByteStream, u16bytes);

    Scalar * outputFileName = P->getInputScalar("outputFileName");
    P->CreateKernelCall<FileSink>(outputFileName, u16bytes);

    return reinterpret_cast<u8u16FunctionType>(P->compile());
}

size_t file_size(const int fd) {
    struct stat st;
    if (LLVM_UNLIKELY(fstat(fd, &st) != 0)) {
        st.st_size = 0;
    }
    return st.st_size;
}

int main(int argc, char *argv[]) {
    codegen::ParseCommandLineOptions(argc, argv, {&u8u16Options, pablo::pablo_toolchain_flags(), codegen::codegen_flags()});
    CPUDriver pxDriver("u8u16");
    u8u16FunctionType u8u16Function = nullptr;
    if (BranchingMode) {
        u8u16Function = generatePipeline2(pxDriver);
    } else {
        u8u16Function = generatePipeline(pxDriver);
    }
    const int fd = open(inputFile.c_str(), O_RDONLY);
    if (LLVM_UNLIKELY(fd == -1)) {
        std::cerr << "Error: cannot open " << inputFile << " for processing. Skipped.\n";
    } else {
        u8u16Function(fd, outputFile.c_str());
        close(fd);
    }
    return 0;
}



