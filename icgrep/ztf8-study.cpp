/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include <cstdio>
#include <vector>
#include <llvm/Support/CommandLine.h>
#include <llvm/Support/ErrorHandling.h>
#include <llvm/Support/PrettyStackTrace.h>
#include <llvm/Support/Signals.h>
#include <llvm/Support/ManagedStatic.h>
#include <llvm/Support/raw_ostream.h>
#include <re/re_name.h>
#include <re/re_re.h>
#include <kernels/kernel_builder.h>
#include <kernels/pipeline_builder.h>
#include <kernels/deletion.h>
#include <kernels/pdep_kernel.h>
#include <kernels/stream_select.h>
#include <kernels/stream_shift.h>
#include <kernels/s2p_kernel.h>
#include <kernels/source_kernel.h>
#include <grep/grep_engine.h>
#include <grep/grep_kernel.h>
#include <kernels/scanmatchgen.h>
#include <cc/cc_compiler.h>
#include <cc/cc_compiler_target.h>
#include <UCD/resolve_properties.h>
#include <UCD/ucd_compiler.hpp>
#include <string>
#include <toolchain/toolchain.h>
#include <re/re_name_resolve.h>
#include <re/re_toolchain.h>
#include <pablo/pablo_toolchain.h>
#include <fcntl.h>
#include <iostream>
#include <toolchain/cpudriver.h>

using namespace kernel;
using namespace llvm;
using namespace pablo;

static cl::OptionCategory ztf8Options("ztf8 Options", "ZTF-8 Compression/Decompression control options.");
static cl::opt<std::string> inputFile(cl::Positional, cl::desc("<input file>"), cl::Required, cl::cat(ztf8Options));
static cl::opt<int> WordLen("length", cl::desc("Length of words."), cl::init(8));


class WordMarkKernelBuilder : public PabloKernel {
public:
    WordMarkKernelBuilder(const std::unique_ptr<KernelBuilder> & kb, StreamSet * BasisBits, StreamSet * U8index, StreamSet * WordMarks);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;
};

WordMarkKernelBuilder::WordMarkKernelBuilder(const std::unique_ptr<KernelBuilder> & kb, StreamSet * BasisBits, StreamSet * U8index, StreamSet * WordMarks)
: PabloKernel(kb, "WordMarks", {Binding{"source", BasisBits}, Binding{"U8index", U8index}}, {Binding{"WordMarks", WordMarks}}) { }

void WordMarkKernelBuilder::generatePabloMethod() {
    pablo::PabloBuilder pb(getEntryScope());
    cc::Parabix_CC_Compiler_Builder ccc(getEntryScope(), getInputStreamSet("source"));
    UCD::UCDCompiler ucdCompiler(ccc);
    re::Name * word = re::makeName("word", re::Name::Type::UnicodeProperty);
    word = cast<re::Name>(re::resolveUnicodeNames(word));
    UCD::UCDCompiler::NameMap nameMap;
    nameMap.emplace(word, nullptr);
    ucdCompiler.generateWithDefaultIfHierarchy(nameMap, pb);
    auto f = nameMap.find(word);
    if (f == nameMap.end()) llvm::report_fatal_error("Cannot find word property");
    PabloAST * wordChar = f->second;
    PabloAST * nonWord = pb.createNot(wordChar);
    PabloAST * U8index = getInputStreamSet("U8index")[0];
    PabloAST * U8nonfinal = pb.createNot(U8index);
    PabloAST * wordNext = pb.createScanThru(pb.createAdvance(wordChar, 1), U8nonfinal, "wordNext");
    PabloAST * wordFollow = pb.createAnd(wordNext, nonWord, "wordFollow");
    PabloAST * wordStart = pb.createAnd(pb.createScanThru(pb.createNot(pb.createAdvance(wordChar, 1)), U8nonfinal), wordChar, "wordStart");
    PabloAST * wordBoundary = pb.createOr(wordStart, wordFollow);
    Var * const WordMarks = getOutputStreamVar("WordMarks");
    pb.createAssign(pb.createExtract(WordMarks, pb.getInteger(0)), wordFollow);
    pb.createAssign(pb.createExtract(WordMarks, pb.getInteger(1)), wordBoundary);
}


class Splitter : public PabloKernel {
public:
    Splitter(const std::unique_ptr<KernelBuilder> & kb, StreamSet * inputs, StreamSet * output1, StreamSet * output2);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;
};

Splitter::Splitter(const std::unique_ptr<KernelBuilder> & kb, StreamSet * inputs, StreamSet * output1, StreamSet * output2)
: PabloKernel(kb, "splitter", {Binding{"inputs", inputs}}, {Binding{"output1", output1}, Binding{"output2", output2}}) { }

void Splitter::generatePabloMethod() {
    pablo::PabloBuilder pb(getEntryScope());
    std::vector<PabloAST *> inputs = getInputStreamSet("inputs");
    pb.createAssign(pb.createExtract(getOutputStreamVar("output1"), pb.getInteger(0)), inputs[0]);
    pb.createAssign(pb.createExtract(getOutputStreamVar("output2"), pb.getInteger(0)), inputs[1]);
}

class WordAccumulator : public grep::MatchAccumulator {
public:
    WordAccumulator() : grep::MatchAccumulator () {}
    ~WordAccumulator() {}
    void accumulate_match(const size_t lineNum, char * line_start, char * line_end) override;
    void dumpWords();
private:
    std::map<std::string, uint64_t> mWordBag;
};

void WordAccumulator::accumulate_match (const size_t lineNum, char * line_start, char * line_end) {
    const auto bytes = line_end - line_start + 1;
    //if (bytes != WordLen) return;
    std::string new_word(line_start, bytes);
    //llvm::errs() << "Found: " << new_word << "\n";
    mWordBag[new_word]++;
}

void WordAccumulator::dumpWords() {
    for (auto const& w : mWordBag) {
        llvm::errs() << w.first << ": " << w.second << "\n";
    }
}


typedef void (*ztf8FunctionType)(uint32_t fd, WordAccumulator *);

ztf8FunctionType generatePipeline(CPUDriver & pxDriver) {
    
    auto & b = pxDriver.getBuilder();
    auto P = pxDriver.makePipeline({Binding{b->getInt32Ty(), "inputFileDecriptor"},
                                    Binding{b->getIntAddrTy(), "callbackObject"}}, {});
    Scalar * fileDescriptor = P->getInputScalar("inputFileDecriptor");
    // File data from mmap
    StreamSet * const ByteStream = P->CreateStreamSet(1, 8);
    P->CreateKernelCall<MMapSourceKernel>(fileDescriptor, ByteStream);
    
    StreamSet * BasisBits = P->CreateStreamSet(8);
    P->CreateKernelCall<S2PKernel>(ByteStream, BasisBits);
    
    StreamSet * const U8index = P->CreateStreamSet(1);
    P->CreateKernelCall<UTF8_index>(BasisBits, U8index);
    
    StreamSet * WordMarks = P->CreateStreamSet(2);
    P->CreateKernelCall<WordMarkKernelBuilder>(BasisBits, U8index, WordMarks);
    
    /*
    StreamSet * u32WordMarks = P->CreateStreamSet(2);
    FilterByMask(P, U8index, WordMarks, u32WordMarks);
    */
    StreamSet * u8WordMarks_back = P->CreateStreamSet(2);
    P->CreateKernelCall<ShiftBack>(WordMarks, u8WordMarks_back);
    
    /*
    StreamSet * u8WordMarks_back = P->CreateStreamSet(2);
    SpreadByMask(P, U8index, u32WordMarks_back, u8WordMarks_back);
    
    StreamSet * u8WordMarks_byte1 = P->CreateStreamSet(2);
    P->CreateKernelCall<ShiftForward>(u8WordMarks_back, u8WordMarks_byte1);
     */
    Scalar * const callbackObject = P->getInputScalar("callbackObject");
    
    StreamSet * u8WordEnds = P->CreateStreamSet(1);
    StreamSet * u8WordBoundaries = P->CreateStreamSet(1);
    
    P->CreateKernelCall<Splitter>(u8WordMarks_back, u8WordEnds, u8WordBoundaries);
    Kernel * scanMatchK = P->CreateKernelCall<ScanMatchKernel>(u8WordEnds, u8WordBoundaries, ByteStream, callbackObject);
    pxDriver.LinkFunction(scanMatchK, "accumulate_match_wrapper", grep::accumulate_match_wrapper);
    pxDriver.LinkFunction(scanMatchK, "finalize_match_wrapper", grep::finalize_match_wrapper);

    return reinterpret_cast<ztf8FunctionType>(P->compile());
}

int main(int argc, char *argv[]) {

    codegen::ParseCommandLineOptions(argc, argv, {&ztf8Options, pablo::pablo_toolchain_flags(), codegen::codegen_flags()});
    CPUDriver driver("ztf8");
    WordAccumulator accum;
    ztf8FunctionType fn = generatePipeline(driver);
    const int fd = open(inputFile.c_str(), O_RDONLY);
    if (LLVM_UNLIKELY(fd == -1)) {
        llvm::errs() << "Error: cannot open " << inputFile << " for processing. Skipped.\n";
    } else {
        fn(fd, &accum);
        close(fd);
    }
    accum.dumpWords();
    return 0;
}
