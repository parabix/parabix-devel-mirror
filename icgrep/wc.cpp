/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <iostream>
#include <iomanip>
#include <sstream>
#include <toolchain.h>
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
#include <kernels/pipeline.h>
#include <pablo/pablo_compiler.h>
#include <pablo/pablo_toolchain.h>
#include <boost/filesystem.hpp>
#include <boost/iostreams/device/mapped_file.hpp>


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

void wc_gen(PabloKernel * kernel) {
    //  input: 8 basis bit streams
    //  output: 3 counters
    
    cc::CC_Compiler ccc(kernel);
    
    PabloBuilder & pb = ccc.getBuilder();

    Var * lc = kernel->addOutput("lineCount", kernel->getSizeTy());
    Var * wc = kernel->addOutput("wordCount", kernel->getSizeTy());
    Var * cc = kernel->addOutput("charCount", kernel->getSizeTy());

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
}

Function * pipeline(Module * mMod, IDISA::IDISA_Builder * iBuilder) {
    Type * mBitBlockType = iBuilder->getBitBlockType();
    Constant * record_counts_routine;
    Type * const size_ty = iBuilder->getSizeTy();
    Type * const voidTy = iBuilder->getVoidTy();
    record_counts_routine = mMod->getOrInsertFunction("record_counts", voidTy, size_ty, size_ty, size_ty, size_ty, size_ty, nullptr);
    Type * const inputType = PointerType::get(ArrayType::get(ArrayType::get(mBitBlockType, 8), 1), 0);
    
    Function * const main = cast<Function>(mMod->getOrInsertFunction("Main", voidTy, inputType, size_ty, size_ty, nullptr));
    main->setCallingConv(CallingConv::C);
    Function::arg_iterator args = main->arg_begin();
    
    Value * const inputStream = &*(args++);
    inputStream->setName("input");
    Value * const fileSize = &*(args++);
    fileSize->setName("fileSize");
    Value * const fileIdx = &*(args++);
    fileIdx->setName("fileIdx");
    
    ExternalFileBuffer ByteStream(iBuilder, iBuilder->getStreamSetTy(1, 8));

    SingleBlockBuffer BasisBits(iBuilder, iBuilder->getStreamSetTy(8, 1));
    
    MMapSourceKernel mmapK(iBuilder);
    std::unique_ptr<Module> mmapM = mmapK.createKernelModule({}, {&ByteStream});
    mmapK.setInitialArguments({fileSize});
    
    S2PKernel  s2pk(iBuilder);
    std::unique_ptr<Module> s2pM = s2pk.createKernelModule({&ByteStream}, {&BasisBits});

    PabloKernel wck(iBuilder, "wc");
    wc_gen(&wck);
    pablo_function_passes(&wck);
    
    std::unique_ptr<Module> wcM = wck.createKernelModule({&BasisBits}, {});
    
    mmapK.addKernelDeclarations(mMod);
    s2pk.addKernelDeclarations(mMod);
    wck.addKernelDeclarations(mMod);
    
    iBuilder->SetInsertPoint(BasicBlock::Create(mMod->getContext(), "entry", main,0));

    ByteStream.setStreamSetBuffer(inputStream, fileSize);
    BasisBits.allocateBuffer();
    
    generatePipelineLoop(iBuilder, {&mmapK, &s2pk, &wck});
    
    Value * lineCount = wck.createGetAccumulatorCall(wck.getInstance(), "lineCount");
    Value * wordCount = wck.createGetAccumulatorCall(wck.getInstance(), "wordCount");
    Value * charCount = wck.createGetAccumulatorCall(wck.getInstance(), "charCount");

    iBuilder->CreateCall(record_counts_routine, std::vector<Value *>({lineCount, wordCount, charCount, fileSize, fileIdx}));
    
    iBuilder->CreateRetVoid();
    
    Linker L(*mMod);
    L.linkInModule(std::move(mmapM));
    L.linkInModule(std::move(s2pM));
    L.linkInModule(std::move(wcM));
    
    return main;
}


typedef void (*wcFunctionType)(char * byte_data, size_t filesize, size_t fileIdx);

static ExecutionEngine * wcEngine = nullptr;

wcFunctionType wcCodeGen(void) { 
    Module * M = new Module("wc", getGlobalContext());
    IDISA::IDISA_Builder * idb = IDISA::GetIDISA_Builder(M);

    llvm::Function * main_IR = pipeline(M, idb);

    wcEngine = JIT_to_ExecutionEngine(M);
    
    wcEngine->finalizeObject();

    delete idb;
    return reinterpret_cast<wcFunctionType>(wcEngine->getPointerToFunction(main_IR));
}

void wc(wcFunctionType fn_ptr, const int64_t fileIdx) {
    std::string fileName = inputFiles[fileIdx];
    size_t fileSize;
    char * fileBuffer;
    
    const boost::filesystem::path file(fileName);
    if (exists(file)) {
        if (is_directory(file)) {
            return;
        }
    } else {
        std::cerr << "Error: cannot open " << fileName << " for processing. Skipped.\n";
        return;
    }
    
    fileSize = file_size(file);
    boost::iostreams::mapped_file_source mappedFile;
    if (fileSize == 0) {
        fileBuffer = nullptr;
    }
    else {
        try {
            mappedFile.open(fileName);
        } catch (std::exception &e) {
            std::cerr << "Error: Boost mmap of " << fileName << ": " << e.what() << std::endl;
            return;
        }
        fileBuffer = const_cast<char *>(mappedFile.data());
    }
    fn_ptr(fileBuffer, fileSize, fileIdx);

    mappedFile.close();
    
}



int main(int argc, char *argv[]) {
    cl::HideUnrelatedOptions(ArrayRef<const cl::OptionCategory *>{&wcFlags, pablo_toolchain_flags(), codegen::codegen_flags()});
    cl::ParseCommandLineOptions(argc, argv);
    if (wcOptions.size() == 0) {
        CountLines = true;
        CountWords = true;
        CountBytes = true;
    }
    else {
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
    
    
    wcFunctionType fn_ptr = wcCodeGen();

    int fileCount = inputFiles.size();
    lineCount.resize(fileCount);
    wordCount.resize(fileCount);
    charCount.resize(fileCount);
    byteCount.resize(fileCount);
    
    for (unsigned i = 0; i < inputFiles.size(); ++i) {
        wc(fn_ptr, i);
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
