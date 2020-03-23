/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

//  hexlines - A program to print hexadecimal representations of lines from a file.
//  The purpose of the program is to produce lines of output corresponding to the
//  hex representation of the source characters (but leaving \n characters unmodified).
//  If the input file consists of the single line "Wolf!\n", the output is the
//  single line "576f6c6621\n".
//

//  This program uses the concept of parallel bit streams as a transposed
//  representation of byte data.
//
//  If a source data stream consists of the single line "Wolf!\n", then the
//  hexadecimal representation of the input is 57 6f 6c 66 21 0a.
//  In this case, the parallel bit stream representation is a set of 8 "basis" bit streams
//  for the bits of each byte as follows (where bit 7 is the high bit of each
//  byte and bit 0 is the low bit of each byte, and "." means a 0 bit):
//    BasisBits[7]    ......
//    BasisBits[6]    1111..
//    BasisBits[5]    .1111.
//    BasisBits[4]    1.....
//    BasisBits[3]    .11..1
//    BasisBits[2]    1111..
//    BasisBits[1]    11.1.1
//    BasisBits[0]    11..1.
//                      ^
//                      |--- Note the 3rd column is 0110 1100 in binary, or hex 6c,
//                           the 3rd character.
//
//
//  STEP 1:  The first step is to spreading out the basis bit streams, inserting one
//  extra zero bit after each data bit.  This will give:

//  spreadBasis[7]    ...........
//  spreadBasis[6]    1.1.1.1....
//  spreadBasis[5]    ..1.1.1.1..
//  spreadBasis[4]    1..........
//  spreadBasis[3]    ..1.1.....1
//  spreadBasis[2]    1.1.1.1....
//  spreadBasis[1]    1.1...1...1
//  spreadBasis[0]    1.1.....1..

//  STEP 2:  The next step is to separate the 8 bits of every character (except for
//  new lines)  into 4 bits each at the original character position and the newly
//  inserted position.  This will give:
//
//          lo[3]     ...1.1....1
//          lo[2]     11111111...
//          lo[1]     .1111.111.1
//          lo[0]     11.1.....1.
//
// Note: that these 4 streams now represent the hex values 576f6c6621a
//
// STEP 3:  The third step is to convert the values to the ASCII representations of
// the hex digits.   When the hex digit is in [0-9], the high 4 bits are 0011, because
// the ASCII representation of 3 is 0x33 for example.   When the hex digit is in [a-f],
// the high 4 bits are 0100, because upper case letters A-F are represented 0x41-0x46.
// (If lower case is desired, the high 4 bits should be 0110).   The low 4 bits must
// be modified by subtracting 9 from the numeric value.   So if the hex value is D,
// the numeric value is 13 and subtracting 9 gives the low 4 bits 0100 which wiil
// are the correct low 4 bits of the ASCII representation of D or d.
//
//    hexBasis[7]     ...........
//    hexBasis[6]     ...1.1.....
//    hexBasis[5]     111.1.1111.
//    hexBasis[4]     111.1.1111.
//    hexBasis[3]     ...1.1....1
//    hexBasis[2]     11111111...
//    hexBasis[1]     .1111.111.1
//    hexBasis[0]     11.1.....1.
//

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

//  These declarations are for command line processing.
//  See the LLVM CommandLine 2.0 Library Manual https://llvm.org/docs/CommandLine.html
static cl::OptionCategory HexLinesOptions("hexlines Options", "hexlines control options.");
static cl::opt<std::string> inputFile(cl::Positional, cl::desc("<input file>"), cl::Required, cl::cat(HexLinesOptions));
static cl::opt<bool> LowerHex("l", cl::desc("Use lower case hex output (default is upper case)."), cl::init(false), cl::cat(HexLinesOptions));

//
//  The Hexify Kernel is the logic that produces hexadecimal output
//  from a source bit stream set called spreadBasis and the insertMask
//  used to spread out the bits.
//
class Hexify : public PabloKernel {
public:
    Hexify(BuilderRef kb, StreamSet * insertMask, StreamSet * spreadBasis, StreamSet * hexBasis)
        : PabloKernel(kb, LowerHex ? "Hexify_lc" : "Hexify",
                      {Binding{"insertMask", insertMask}, Binding{"spreadBasis", spreadBasis}},
                      {Binding{"hexBasis", hexBasis}}) {}
protected:
    void generatePabloMethod() override;
};


void Hexify::generatePabloMethod() {
    //  pb is an object used for build Pablo language statements
    pablo::PabloBuilder pb(getEntryScope());
    //  bnc is an object that can perform arithmetic on sets of parallel bit streams
    BixNumCompiler bnc(pb);
    // Get the input stream sets.
    PabloAST * insertMask = getInputStreamSet("insertMask")[0];
    std::vector<PabloAST *> spreadBasis = getInputStreamSet("spreadBasis");
    // ccc is an object that can compile character classes from a set of 8 parallel bit streams.
    cc::Parabix_CC_Compiler_Builder ccc(getEntryScope(), spreadBasis);
    // Compute the stream marking LFs.
    PabloAST * LF = ccc.compileCC(re::makeByte(0xA));
    
    // Compute a set of 4 bit streams as described in STEP 2 above.
    std::vector<PabloAST *> lo(4);
    lo[3] = pb.createSel(insertMask, spreadBasis[7], pb.createAdvance(spreadBasis[3], 1), "lo[3]");
    lo[2] = pb.createSel(insertMask, spreadBasis[6], pb.createAdvance(spreadBasis[2], 1), "lo[2]");
    lo[1] = pb.createSel(insertMask, spreadBasis[5], pb.createAdvance(spreadBasis[1], 1), "lo[1]");
    lo[0] = pb.createSel(insertMask, spreadBasis[4], pb.createAdvance(spreadBasis[0], 1), "lo[0]");
    //  Given these 4 bit streams, a position is in the hex A-F range when its binary code is >= 10.
    PabloAST * hexA_F = bnc.UGE(lo, 10, "hexA_F");
    //  If a number is in the hexA_F range, the 4 bits of the hexBasis are determined by subtracting 9.
    BixNum lo1 = bnc.SubModular(lo, 9);
    std::vector<PabloAST *> hexBasis(8);
    // Now compute the high 4 bits of the hexBasis.
    hexBasis[7] = pb.createZeroes();
    hexBasis[6] = hexA_F;
    if (LowerHex) {
        hexBasis[5] = pb.createNot(LF);
    } else {
        hexBasis[5] = pb.createNot(hexA_F);
    }
    hexBasis[4] = pb.createNot(hexA_F);
    // High 4 bits are 0x40 for hex A-F, 0x30 for hex 0-9
    for (unsigned i = 0; i < 4; i++) {
        hexBasis[i] = pb.createSel(hexA_F, lo1[i], lo[i]);
    }
    Var * hexVar = getOutputStreamVar("hexBasis");
    for (unsigned i = 0; i < 8; i++) {
        pb.createAssign(pb.createExtract(hexVar, pb.getInteger(i)), pb.createSel(LF, spreadBasis[i], hexBasis[i]));
    }
}

typedef void (*HexLinesFunctionType)(uint32_t fd);

HexLinesFunctionType generatePipeline(CPUDriver & pxDriver) {
    // A Parabix program is build as a set of kernel calls called a pipeline.
    // A pipeline is construction using a Parabix driver object.
    auto & b = pxDriver.getBuilder();
    auto P = pxDriver.makePipeline({Binding{b->getInt32Ty(), "inputFileDecriptor"},
                                    Binding{b->getIntAddrTy(), "callbackObject"}}, {});
    //  The program will use a file descriptor as an input.
    Scalar * fileDescriptor = P->getInputScalar("inputFileDecriptor");
    // File data from mmap
    StreamSet * ByteStream = P->CreateStreamSet(1, 8);
    //  MMapSourceKernel is a Parabix Kernel that produces a stream of bytes
    //  from a file descriptor.
    P->CreateKernelCall<MMapSourceKernel>(fileDescriptor, ByteStream);

    //  The Parabix basis bits representation is created by the Parabix S2P kernel.
    //  S2P stands for serial-to-parallel.
    StreamSet * BasisBits = P->CreateStreamSet(8);
    P->CreateKernelCall<S2PKernel>(ByteStream, BasisBits);

    //  We need to know which input positions are LFs and which are not.
    //  The nonLF positions need to be expanded to generate two hex digits each.
    //  The Parabix CharacterClassKernelBuilder can create any desired stream for
    //  characters.   Note that the input is the set of byte values in the range
    //  [\x{00}-x{09}\x{0B}-\x{FF}] that is, all byte values except \x{0A}.
    //  For our example input "Wolf!\b", the nonLF stream is "11111."
    StreamSet * nonLF = P->CreateStreamSet(1);
    std::vector<re::CC *> nonLF_CC = {re::makeCC(re::makeByte(0,9), re::makeByte(0xB, 0xff))};
    P->CreateKernelCall<CharacterClassKernelBuilder>(nonLF_CC, BasisBits, nonLF);
    
    //  We need to spread out the basis bits to make room for two positions for
    //  each non LF in the input.   The Parabix function UnitInsertionSpreadMask
    //  takes care of this using a mask of positions for insertion of one position.
    //  We insert one position for eacn nonLF character.    Given the
    //  nonLF stream "11111", the hexInsertMask is "1.1.1.1.1.1"
    StreamSet * hexInsertMask = UnitInsertionSpreadMask(P, nonLF, InsertPosition::After);

    // The parabix SpreadByMask function copies bits from an input stream
    // set to an output stream set, to positions marked by 1s in the first
    // argument (the spread mask).   Zeroes are inserted everywhere else.
    // This function performs STEP 1 in the comments above.
    StreamSet * spreadBasis = P->CreateStreamSet(8);
    SpreadByMask(P, hexInsertMask, BasisBits, spreadBasis);
    
    // Perform the logic of the Hexify kernel.
    StreamSet * hexBasis = P->CreateStreamSet(8);
    P->CreateKernelCall<Hexify>(hexInsertMask, spreadBasis, hexBasis);

    // The computed output can be converted back to byte stream form by the
    // P2S kernel (parallel-to-serial).
    StreamSet * hexLines = P->CreateStreamSet(1, 8);
    P->CreateKernelCall<P2SKernel>(hexBasis, hexLines);

    //  The StdOut kernel writes a byte stream to standard output.
    P->CreateKernelCall<StdOutKernel>(hexLines);
    return reinterpret_cast<HexLinesFunctionType>(P->compile());
}

int main(int argc, char *argv[]) {
    //  ParseCommandLineOptions uses the LLVM CommandLine processor, but we also add
    //  standard Parabix command line options such as -help, -ShowPablo and many others.
    codegen::ParseCommandLineOptions(argc, argv, {&HexLinesOptions, pablo::pablo_toolchain_flags(), codegen::codegen_flags()});
    //  A CPU driver is capable of compiling and running Parabix programs on the CPU.
    CPUDriver driver("hexlines");
    //  Build and compile the Parabix pipeline by calling the Pipeline function above.
    HexLinesFunctionType fn = generatePipeline(driver);
    //  The compile function "fn"  can now be used.   It takes a file
    //  descriptor as an input, which is specified by the filename given by
    //  the inputFile command line option.
    const int fd = open(inputFile.c_str(), O_RDONLY);
    if (LLVM_UNLIKELY(fd == -1)) {
        llvm::errs() << "Error: cannot open " << inputFile << " for processing. Skipped.\n";
    } else {
        //  Run the pipeline.
        fn(fd);
        close(fd);
    }
    return 0;
}
