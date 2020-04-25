/*
 *  Copyright (c) 2020 International Characters.
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

//  These declarations are for command line processing.
//  See the LLVM CommandLine 2.0 Library Manual https://llvm.org/docs/CommandLine.html
static cl::OptionCategory CSV_Quote_Options("CSV Quote Translation Options", "CSV Quote Translation Options.");
static cl::opt<std::string> inputFile(cl::Positional, cl::desc("<input file>"), cl::Required, cl::cat(CSV_Quote_Options));

//
//  The Hexify Kernel is the logic that produces hexadecimal output
//  from a source bit stream set called spreadBasis and the insertMask
//  used to spread out the bits.
//
class CSV_Quote_Translate : public PabloKernel {
public:
    CSV_Quote_Translate(BuilderRef kb, StreamSet * dquote, StreamSet * basis, StreamSet * translatedBasis)
        : PabloKernel(kb, "CSV_Quote_Translate",
                      {Binding{"dquote", dquote, FixedRate(), LookAhead(1)}, Binding{"basis", basis}},
                      {Binding{"translatedBasis", translatedBasis}}) {}
protected:
    void generatePabloMethod() override;
};

void CSV_Quote_Translate::generatePabloMethod() {
    pablo::PabloBuilder pb(getEntryScope());
    std::vector<PabloAST *> basis = getInputStreamSet("basis");
    PabloAST * dquote = getInputStreamSet("dquote")[0];
    PabloAST * dquote_odd = pb.createEveryNth(dquote, pb.getInteger(2));
    PabloAST * dquote_even = pb.createXor(dquote, dquote_odd);
    PabloAST * quote_escape = pb.createAnd(dquote_even, pb.createLookahead(dquote, 1));
    // Translate "" to \"    ASCII value of " = 0x22, ASCII value of \ = 0x5C
    std::vector<PabloAST *> translated_basis(8, nullptr);
    translated_basis[0] = basis[0];                              // no changes
    translated_basis[1] = pb.createXor(basis[1], quote_escape);  // flip
    translated_basis[2] = pb.createXor(basis[2], quote_escape);  // flip
    translated_basis[3] = pb.createXor(basis[3], quote_escape);  // flip
    translated_basis[4] = pb.createXor(basis[4], quote_escape);  // flip
    translated_basis[5] = pb.createXor(basis[5], quote_escape);  // flip
    translated_basis[6] = pb.createXor(basis[6], quote_escape);  // flip
    translated_basis[7] = basis[7];                              // no changes

    Var * translatedVar = getOutputStreamVar("translatedBasis");
    for (unsigned i = 0; i < 8; i++) {
        pb.createAssign(pb.createExtract(translatedVar, pb.getInteger(i)), translated_basis[i]);
    }
}

typedef void (*CSVTranslateFunctionType)(uint32_t fd);

CSVTranslateFunctionType generatePipeline(CPUDriver & pxDriver) {
    // A Parabix program is build as a set of kernel calls called a pipeline.
    // A pipeline is construction using a Parabix driver object.
    auto & b = pxDriver.getBuilder();
    auto P = pxDriver.makePipeline({Binding{b->getInt32Ty(), "inputFileDecriptor"}}, {});
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

    //  We need to know which input positions are dquotes and which are not.
    StreamSet * Dquote = P->CreateStreamSet(1);
    P->CreateKernelCall<CharacterClassKernelBuilder>(std::vector<re::CC *>{re::makeByte(0x22)}, BasisBits, Dquote);
    
    StreamSet * translatedBasis = P->CreateStreamSet(8);
    P->CreateKernelCall<CSV_Quote_Translate>(Dquote, BasisBits, translatedBasis);

    // The computed output can be converted back to byte stream form by the
    // P2S kernel (parallel-to-serial).
    StreamSet * translated = P->CreateStreamSet(1, 8);
    P->CreateKernelCall<P2SKernel>(translatedBasis, translated);

    //  The StdOut kernel writes a byte stream to standard output.
    P->CreateKernelCall<StdOutKernel>(translated);
    return reinterpret_cast<CSVTranslateFunctionType>(P->compile());
}

int main(int argc, char *argv[]) {
    //  ParseCommandLineOptions uses the LLVM CommandLine processor, but we also add
    //  standard Parabix command line options such as -help, -ShowPablo and many others.
    codegen::ParseCommandLineOptions(argc, argv, {&CSV_Quote_Options, pablo::pablo_toolchain_flags(), codegen::codegen_flags()});
    //  A CPU driver is capable of compiling and running Parabix programs on the CPU.
    CPUDriver driver("csv_quote_xlator");
    //  Build and compile the Parabix pipeline by calling the Pipeline function above.
    CSVTranslateFunctionType fn = generatePipeline(driver);
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
