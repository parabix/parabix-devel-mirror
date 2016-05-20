/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <string>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>


#include <toolchain.h>
#include <pablo/pablo_toolchain.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/Module.h>
#include <llvm/ExecutionEngine/ExecutionEngine.h>
#include <llvm/ExecutionEngine/MCJIT.h>

#include <llvm/Support/CommandLine.h>
#include <llvm/Support/raw_ostream.h>

#include <utf_encoding.h>
#include <re/re_cc.h>
#include <cc/cc_compiler.h>
#include <pablo/function.h>
#include <IDISA/idisa_builder.h>
#include <IDISA/idisa_target.h>
#include <kernels/instance.h>
#include <kernels/kernel.h>
#include <kernels/s2p_kernel.h>

#include <pablo/pablo_compiler.h>
#include <pablo/pablo_toolchain.h>


#include <utf_encoding.h>

// mmap system
#include <boost/filesystem.hpp>
#include <boost/iostreams/device/mapped_file.hpp>
using namespace boost::iostreams;
using namespace boost::filesystem;

#include <fcntl.h>
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

pablo::PabloFunction * wc_gen(Encoding encoding) {
    //  input: 8 basis bit streams
    //  output: 3 count streams
    
    pablo::PabloFunction * function = pablo::PabloFunction::Create("wc", 8, 3);
    cc::CC_Compiler ccc(*function, encoding);
    
    pablo::PabloBuilder pBuilder(ccc.getBuilder().getPabloBlock(), ccc.getBuilder());
    const std::vector<pablo::Var *> u8_bits = ccc.getBasisBits();

    if (CountLines) {
        pablo::PabloAST * LF = ccc.compileCC(re::makeCC(0x0A));
        function->setResult(0, pBuilder.createAssign("lineCount", pBuilder.createCount(LF)));
    }
    else function->setResult(0, pBuilder.createAssign("lineCount", pBuilder.createZeroes()));
    if (CountWords) {
        pablo::PabloAST * WS = ccc.compileCC(re::makeCC(re::makeCC(0x09, 0x0D), re::makeCC(0x20)));
        
        pablo::PabloAST * wordChar = pBuilder.createNot(WS);
        // WS_follow_or_start = 1 past WS or at start of file
        pablo::PabloAST * WS_follow_or_start = pBuilder.createNot(pBuilder.createAdvance(wordChar, 1));
        //
        pablo::PabloAST * wordStart = pBuilder.createInFile(pBuilder.createAnd(wordChar, WS_follow_or_start));
        function->setResult(1, pBuilder.createAssign("wordCount", pBuilder.createCount(wordStart)));
    }
    else function->setResult(1, pBuilder.createAssign("wordCount", pBuilder.createZeroes()));
    if (CountChars) {
        //
        // FIXME: This correctly counts characters assuming valid UTF-8 input.  But what if input is
        // not UTF-8, or is not valid?
        //
        pablo::PabloAST * u8Begin = ccc.compileCC(re::makeCC(re::makeCC(0, 0x7F), re::makeCC(0xC2, 0xF4)));
        function->setResult(2, pBuilder.createAssign("charCount", pBuilder.createCount(u8Begin)));
    }
    else function->setResult(2, pBuilder.createAssign("charCount", pBuilder.createZeroes()));
    return function;
}

using namespace kernel;


class wcPipelineBuilder {
public:
    wcPipelineBuilder(llvm::Module * m, IDISA::IDISA_Builder * b);
    
    ~wcPipelineBuilder();
    
    void CreateKernels(pablo::PabloFunction * function);
    llvm::Function * ExecuteKernels();
    
private:
    llvm::Module *                      mMod;
    IDISA::IDISA_Builder *              iBuilder;
    KernelBuilder *                     mS2PKernel;
    KernelBuilder *                     mWC_Kernel;
    llvm::Type *                        mBitBlockType;
    int                                 mBlockSize;
};


using namespace pablo;
using namespace kernel;

wcPipelineBuilder::wcPipelineBuilder(Module * m, IDISA::IDISA_Builder * b)
: mMod(m)
, iBuilder(b)
, mBitBlockType(b->getBitBlockType())
, mBlockSize(b->getBitBlockWidth()){
    
}

wcPipelineBuilder::~wcPipelineBuilder(){
    delete mS2PKernel;
    delete mWC_Kernel;
}

void wcPipelineBuilder::CreateKernels(PabloFunction * function){
    mS2PKernel = new KernelBuilder(iBuilder, "s2p", codegen::SegmentSize);
    mWC_Kernel = new KernelBuilder(iBuilder, "wc", codegen::SegmentSize);
    
    generateS2PKernel(mMod, iBuilder, mS2PKernel);
    
    pablo_function_passes(function);
    
    PabloCompiler pablo_compiler(mMod, iBuilder);
    try {
        pablo_compiler.setKernel(mWC_Kernel);
        pablo_compiler.compile(function);
        delete function;
        releaseSlabAllocatorMemory();
    } catch (std::runtime_error e) {
        delete function;
        releaseSlabAllocatorMemory();
        std::cerr << "Runtime error: " << e.what() << std::endl;
        exit(1);
    }
    
}




Function * wcPipelineBuilder::ExecuteKernels() {
    Constant * record_counts_routine;
    Type * const int64ty = iBuilder->getInt64Ty();
    Type * const voidTy = Type::getVoidTy(mMod->getContext());
    record_counts_routine = mMod->getOrInsertFunction("record_counts", voidTy, int64ty, int64ty, int64ty, int64ty, int64ty, nullptr);
    Type * const inputType = PointerType::get(ArrayType::get(StructType::get(mMod->getContext(), std::vector<Type *>({ArrayType::get(mBitBlockType, 8)})), 1), 0);
    
    Function * const main = cast<Function>(mMod->getOrInsertFunction("Main", voidTy, inputType, int64ty, int64ty, nullptr));
    main->setCallingConv(CallingConv::C);
    Function::arg_iterator args = main->arg_begin();
    
    Value * const inputStream = &*(args++);
    inputStream->setName("input");
    Value * const bufferSize = &*(args++);
    bufferSize->setName("bufferSize");
    Value * const fileIdx = &*(args++);
    fileIdx->setName("fileIdx");
    
    iBuilder->SetInsertPoint(BasicBlock::Create(mMod->getContext(), "entry", main,0));
    
    BasicBlock * entryBlock = iBuilder->GetInsertBlock();

    BasicBlock * segmentCondBlock = nullptr;
    BasicBlock * segmentBodyBlock = nullptr;
    const unsigned segmentSize = codegen::SegmentSize;
    if (segmentSize > 1) {
        segmentCondBlock = BasicBlock::Create(mMod->getContext(), "segmentCond", main, 0);
        segmentBodyBlock = BasicBlock::Create(mMod->getContext(), "segmentBody", main, 0);
    }
    BasicBlock * fullCondBlock = BasicBlock::Create(mMod->getContext(), "fullCond", main, 0);
    BasicBlock * fullBodyBlock = BasicBlock::Create(mMod->getContext(), "fullBody", main, 0);
    BasicBlock * finalBlock = BasicBlock::Create(mMod->getContext(), "final", main, 0);
    BasicBlock * finalPartialBlock = BasicBlock::Create(mMod->getContext(), "partial", main, 0);
    BasicBlock * finalEmptyBlock = BasicBlock::Create(mMod->getContext(), "empty", main, 0);
    BasicBlock * endBlock = BasicBlock::Create(mMod->getContext(), "end", main, 0);

    Instance * s2pInstance = mS2PKernel->instantiate(inputStream);
    Instance * wcInstance = mWC_Kernel->instantiate(s2pInstance->getOutputStreamBuffer());

    Value * initialBufferSize = nullptr;
    BasicBlock * initialBlock = nullptr;
    
    if (segmentSize > 1) {
        iBuilder->CreateBr(segmentCondBlock);
        iBuilder->SetInsertPoint(segmentCondBlock);
        PHINode * remainingBytes = iBuilder->CreatePHI(int64ty, 2, "remainingBytes");
        remainingBytes->addIncoming(bufferSize, entryBlock);
        Constant * const step = ConstantInt::get(int64ty, mBlockSize * segmentSize);
        Value * segmentCondTest = iBuilder->CreateICmpULT(remainingBytes, step);
        iBuilder->CreateCondBr(segmentCondTest, fullCondBlock, segmentBodyBlock);
        iBuilder->SetInsertPoint(segmentBodyBlock);
        for (unsigned i = 0; i < segmentSize; ++i) {
            s2pInstance->CreateDoBlockCall();
        }
        for (unsigned i = 0; i < segmentSize; ++i) {
            wcInstance->CreateDoBlockCall();
        }
        remainingBytes->addIncoming(iBuilder->CreateSub(remainingBytes, step), segmentBodyBlock);
        iBuilder->CreateBr(segmentCondBlock);
        initialBufferSize = remainingBytes;
        initialBlock = segmentCondBlock;
    } else {
        initialBufferSize = bufferSize;
        initialBlock = entryBlock;
        iBuilder->CreateBr(fullCondBlock);
    }

    iBuilder->SetInsertPoint(fullCondBlock);
    PHINode * remainingBytes = iBuilder->CreatePHI(int64ty, 2, "remainingBytes");
    remainingBytes->addIncoming(initialBufferSize, initialBlock);

    Constant * const step = ConstantInt::get(int64ty, mBlockSize);
    Value * fullCondTest = iBuilder->CreateICmpULT(remainingBytes, step);
    iBuilder->CreateCondBr(fullCondTest, finalBlock, fullBodyBlock);
    
    iBuilder->SetInsertPoint(fullBodyBlock);

    s2pInstance->CreateDoBlockCall();
    wcInstance->CreateDoBlockCall();

    Value * diff = iBuilder->CreateSub(remainingBytes, step);

    remainingBytes->addIncoming(diff, fullBodyBlock);
    iBuilder->CreateBr(fullCondBlock);
    
    iBuilder->SetInsertPoint(finalBlock);
    Value * EOFmark = iBuilder->CreateShl(ConstantInt::get(iBuilder->getIntNTy(mBlockSize), 1), remainingBytes);
	wcInstance->setInternalState("EOFmark", iBuilder->CreateBitCast(EOFmark, mBitBlockType));
    
    Value * emptyBlockCond = iBuilder->CreateICmpEQ(remainingBytes, ConstantInt::get(int64ty, 0));
    iBuilder->CreateCondBr(emptyBlockCond, finalEmptyBlock, finalPartialBlock);
    
    
    iBuilder->SetInsertPoint(finalPartialBlock);
    s2pInstance->CreateDoBlockCall();

    iBuilder->CreateBr(endBlock);
    
    iBuilder->SetInsertPoint(finalEmptyBlock);
    s2pInstance->clearOutputStreamSet();
    iBuilder->CreateBr(endBlock);
    
    iBuilder->SetInsertPoint(endBlock);

    wcInstance->CreateDoBlockCall();
    
    Value * lineCount = iBuilder->CreateExtractElement(iBuilder->CreateBlockAlignedLoad(wcInstance->getOutputStream((int) 0)), iBuilder->getInt32(0));
    Value * wordCount = iBuilder->CreateExtractElement(iBuilder->CreateBlockAlignedLoad(wcInstance->getOutputStream(1)), iBuilder->getInt32(0));
    Value * charCount = iBuilder->CreateExtractElement(iBuilder->CreateBlockAlignedLoad(wcInstance->getOutputStream(2)), iBuilder->getInt32(0));
    
    iBuilder->CreateCall(record_counts_routine, std::vector<Value *>({lineCount, wordCount, charCount, bufferSize, fileIdx}));
    
    iBuilder->CreateRetVoid();
    
    return main;
}


typedef void (*wcFunctionType)(char * byte_data, size_t filesize, size_t fileIdx);

static ExecutionEngine * wcEngine = nullptr;

wcFunctionType wcCodeGen(void) {
                            
    Module * M = new Module("wc", getGlobalContext());
    IDISA::IDISA_Builder * idb = IDISA::GetIDISA_Builder(M);

    wcPipelineBuilder pipelineBuilder(M, idb);
    Encoding encoding(Encoding::Type::UTF_8, 8);
    pablo::PabloFunction * function = wc_gen(encoding);
    pipelineBuilder.CreateKernels(function);
    llvm::Function * main_IR = pipelineBuilder.ExecuteKernels();

    wcEngine = JIT_to_ExecutionEngine(M);
    
    wcEngine->finalizeObject();

    delete idb;
    return reinterpret_cast<wcFunctionType>(wcEngine->getPointerToFunction(main_IR));
}

void wc(wcFunctionType fn_ptr, const int64_t fileIdx) {
    std::string fileName = inputFiles[fileIdx];
    size_t fileSize;
    char * fileBuffer;
    
    const path file(fileName);
    if (exists(file)) {
        if (is_directory(file)) {
            return;
        }
    } else {
        std::cerr << "Error: cannot open " << fileName << " for processing. Skipped.\n";
        return;
    }
    
    fileSize = file_size(file);
    mapped_file_source mappedFile;
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
    cl::HideUnrelatedOptions(ArrayRef<const cl::OptionCategory *>{&wcFlags, pablo::pablo_toolchain_flags(), codegen::codegen_flags()});
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
    
    delete wcEngine;
    
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

                       
