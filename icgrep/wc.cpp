/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <iostream>
#include <iomanip>
#include <sstream>
#include <toolchain/toolchain.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/Module.h>
#include <llvm/ExecutionEngine/ExecutionEngine.h>
#include "llvm/Linker/Linker.h"
#include <llvm/Support/CommandLine.h>
#include <llvm/Support/raw_ostream.h>
#include <cc/cc_compiler.h>
#include <pablo/pablo_kernel.h>
#include <IR_Gen/idisa_builder.h>
#include <IR_Gen/idisa_target.h>
#include <kernels/streamset.h>
#include <kernels/mmap_kernel.h>
#include <kernels/s2p_kernel.h>
#include <pablo/pablo_compiler.h>
#include <pablo/pablo_toolchain.h>
#include <fcntl.h>

using namespace llvm;

static cl::OptionCategory wcFlags("Command Flags", "wc options");

static cl::list<std::string> inputFiles(cl::Positional, cl::desc("<input file ...>"), cl::OneOrMore, cl::cat(wcFlags));

enum CountOptions {
    LineOption, WordOption, CharOption, ByteOption
};

static cl::list<CountOptions> wcOptions(
  cl::values(clEnumValN(LineOption, "l", "Report the number of lines in each input file."),
             clEnumValN(WordOption, "w", "Report the number of words in each input file."),
             clEnumValN(CharOption, "m", "Report the number of characters in each input file (override -c)."),
             clEnumValN(ByteOption, "c", "Report the number of bytes in each input file (override -m)."),
             clEnumValEnd), cl::cat(wcFlags), cl::Grouping);
                                                 


static int defaultFieldWidth = 7;  // default field width


bool CountLines = false;
bool CountWords = false;
bool CountChars = false;
bool CountBytes = false;

std::vector<uint64_t> lineCount;
std::vector<uint64_t> wordCount;
std::vector<uint64_t> charCount;
std::vector<uint64_t> byteCount;

uint64_t TotalLines = 0;
uint64_t TotalWords = 0;
uint64_t TotalChars = 0;
uint64_t TotalBytes = 0;

using namespace pablo;
using namespace kernel;
using namespace parabix;

//  The callback routine that records counts in progress.
//
extern "C" {
    void record_counts(uint64_t lines, uint64_t words, uint64_t chars, uint64_t bytes, uint64_t fileIdx) {
        lineCount[fileIdx] = lines;
        wordCount[fileIdx] = words;
        charCount[fileIdx] = chars;
        byteCount[fileIdx] = bytes;
        TotalLines += lines;
        TotalWords += words;
        TotalChars += chars;
        TotalBytes += bytes;
    }
}

//
//

std::unique_ptr<PabloKernel> wc_gen(IDISA::IDISA_Builder * iBuilder) {
    
    auto kernel = std::unique_ptr<PabloKernel>(new PabloKernel(iBuilder, "Parabix:wc",
                    {Binding{iBuilder->getStreamSetTy(8, 1), "u8bit"}},
                    {},
                    {},
                    {Binding{iBuilder->getSizeTy(), "lineCount"}, Binding{iBuilder->getSizeTy(), "wordCount"}, Binding{iBuilder->getSizeTy(), "charCount"}}));
    
    //  input: 8 basis bit streams
    const auto u8bitSet = kernel->getInputStreamVar("u8bit");
    //  output: 3 counters
    
    cc::CC_Compiler ccc(kernel.get(), u8bitSet);
    
    PabloBuilder & pb = ccc.getBuilder();

    Var * lc = kernel->getOutputScalarVar("lineCount");
    Var * wc = kernel->getOutputScalarVar("wordCount");
    Var * cc = kernel->getOutputScalarVar("charCount");

    if (CountLines) {
        PabloAST * LF = ccc.compileCC(re::makeCC(0x0A));
        pb.createAssign(lc, pb.createCount(LF));
    }
    if (CountWords) {
        PabloAST * WS = ccc.compileCC(re::makeCC(re::makeCC(0x09, 0x0D), re::makeCC(0x20)));
        PabloAST * wordChar = pb.createNot(WS);
        // WS_follow_or_start = 1 past WS or at start of file
        PabloAST * WS_follow_or_start = pb.createNot(pb.createAdvance(wordChar, 1));
        PabloAST * wordStart = pb.createInFile(pb.createAnd(wordChar, WS_follow_or_start));
        pb.createAssign(wc, pb.createCount(wordStart));
    }
    if (CountChars) {
        //
        // FIXME: This correctly counts characters assuming valid UTF-8 input.  But what if input is
        // not UTF-8, or is not valid?
        //
        PabloAST * u8Begin = ccc.compileCC(re::makeCC(re::makeCC(0, 0x7F), re::makeCC(0xC2, 0xF4)));        
        pb.createAssign(cc, pb.createCount(u8Begin));
    }
    pablo_function_passes(kernel.get());
    return kernel;
}




typedef void (*WordCountFunctionType)(uint32_t fd, size_t fileIdx);

void wcPipelineGen(ParabixDriver & pxDriver) {

    auto iBuilder = pxDriver.getIDISA_Builder();
    Module * m = iBuilder->getModule();
    
    Type * const int32Ty = iBuilder->getInt32Ty();
    Type * const sizeTy = iBuilder->getSizeTy();
    Type * const voidTy = iBuilder->getVoidTy();

    FunctionType * const recordCountsType = FunctionType::get(voidTy, {sizeTy, sizeTy, sizeTy, sizeTy, sizeTy}, false);
    Constant * const recordCounts = m->getOrInsertFunction("record_counts", recordCountsType);

    FunctionType * const mainType = FunctionType::get(voidTy, {int32Ty, sizeTy}, false);
    Function * const main = cast<Function>(m->getOrInsertFunction("Main", mainType));
    main->setCallingConv(CallingConv::C);
    Function::arg_iterator args = main->arg_begin();    
    Value * const fileDecriptor = &*(args++);
    fileDecriptor->setName("fileDecriptor");
    Value * const fileIdx = &*(args++);
    fileIdx->setName("fileIdx");

    iBuilder->SetInsertPoint(BasicBlock::Create(m->getContext(), "entry", main,0));

    StreamSetBuffer * const ByteStream = pxDriver.addBuffer(make_unique<SourceFileBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 8)));

    StreamSetBuffer * const BasisBits = pxDriver.addBuffer(make_unique<SingleBlockBuffer>(iBuilder, iBuilder->getStreamSetTy(8, 1)));

    KernelBuilder * mmapK = pxDriver.addKernelInstance(make_unique<MMapSourceKernel>(iBuilder));
    mmapK->setInitialArguments({fileDecriptor});
    pxDriver.makeKernelCall(mmapK, {}, {ByteStream});

    KernelBuilder * s2pk = pxDriver.addKernelInstance(make_unique<S2PKernel>(iBuilder));
    pxDriver.makeKernelCall(s2pk, {ByteStream}, {BasisBits});
    
    KernelBuilder * wck = pxDriver.addKernelInstance(wc_gen(iBuilder));
    pxDriver.makeKernelCall(wck, {BasisBits}, {});


    pxDriver.generatePipelineIR();
    
    Value * const fileSize = mmapK->getAccumulator("fileSize");
    Value * const lineCount = wck->getAccumulator("lineCount");
    Value * const wordCount = wck->getAccumulator("wordCount");
    Value * const charCount = wck->getAccumulator("charCount");

    iBuilder->CreateCall(recordCounts, {lineCount, wordCount, charCount, fileSize, fileIdx});
    
    iBuilder->CreateRetVoid();

    pxDriver.linkAndFinalize();
}


WordCountFunctionType wcCodeGen() {
    ParabixDriver pxDriver("wc");
    wcPipelineGen(pxDriver);
    return reinterpret_cast<WordCountFunctionType>(pxDriver.getPointerToMain());
}

void wc(WordCountFunctionType fn_ptr, const int64_t fileIdx) {
    std::string fileName = inputFiles[fileIdx];
    const int fd = open(fileName.c_str(), O_RDONLY);
    if (LLVM_UNLIKELY(fd == -1)) {
        std::cerr << "Error: cannot open " << fileName << " for processing. Skipped.\n";
    } else {
        fn_ptr(fd, fileIdx);
        close(fd);
    }
}

int main(int argc, char *argv[]) {
    AddParabixVersionPrinter();
    cl::HideUnrelatedOptions(ArrayRef<const cl::OptionCategory *>{&wcFlags, pablo_toolchain_flags(), codegen::codegen_flags()});
    cl::ParseCommandLineOptions(argc, argv);
    if (wcOptions.size() == 0) {
        CountLines = true;
        CountWords = true;
        CountBytes = true;
    } else {
        CountLines = false;
        CountWords = false;
        CountBytes = false;
        CountChars = false;
        for (unsigned i = 0; i < wcOptions.size(); i++) {
            switch (wcOptions[i]) {
                case WordOption: CountWords = true; break;
                case LineOption: CountLines = true; break;
                case CharOption: CountBytes = true; CountChars = false; break;
                case ByteOption: CountChars = true; CountBytes = false; break;
            }
        }
    }
    
    WordCountFunctionType wordCountFunctionPtr = wcCodeGen();

    const auto fileCount = inputFiles.size();
    lineCount.resize(fileCount);
    wordCount.resize(fileCount);
    charCount.resize(fileCount);
    byteCount.resize(fileCount);
    
    for (unsigned i = 0; i < fileCount; ++i) {
        wc(wordCountFunctionPtr, i);
    }
    
    size_t maxCount = 0;
    if (CountLines) maxCount = TotalLines;
    if (CountWords) maxCount = TotalWords;
    if (CountChars) maxCount = TotalChars;
    if (CountBytes) maxCount = TotalBytes;
    
    int fieldWidth = std::to_string(maxCount).size() + 1;
    if (fieldWidth < defaultFieldWidth) fieldWidth = defaultFieldWidth;

    for (unsigned i = 0; i < inputFiles.size(); ++i) {
        std::cout << std::setw(fieldWidth-1);
        if (CountLines) {
            std::cout << lineCount[i] << std::setw(fieldWidth);
        }
        if (CountWords) {
            std::cout << wordCount[i] << std::setw(fieldWidth);
        }
        if (CountChars) {
            std::cout << charCount[i] << std::setw(fieldWidth);
        }
        if (CountBytes) {
            std::cout << byteCount[i];
        }
        std::cout << " " << inputFiles[i] << std::endl;
    }
    if (inputFiles.size() > 1) {
        std::cout << std::setw(fieldWidth-1);
        if (CountLines) {
            std::cout << TotalLines << std::setw(fieldWidth);
        }
        if (CountWords) {
            std::cout << TotalWords << std::setw(fieldWidth);
        }
        if (CountChars) {
            std::cout << TotalChars << std::setw(fieldWidth);
        }
        if (CountBytes) {
            std::cout << TotalBytes;
        }
        std::cout << " total" << std::endl;
    }

    return 0;
}
