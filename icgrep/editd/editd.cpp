/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <string>
#include <iostream>
#include <fstream>
#include <toolchain/toolchain.h>
#include <pablo/pablo_toolchain.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/Module.h>
#include <llvm/Support/CommandLine.h>
#include <cc/cc_compiler.h>
#include <pablo/pablo_compiler.h>
#include <pablo/pablo_kernel.h>
#include <kernels/kernel_builder.h>
#include <IR_Gen/idisa_target.h>
#include <kernels/streamset.h>
#include <kernels/source_kernel.h>
#include <kernels/s2p_kernel.h>
#include <editd/editdscan_kernel.h>
#include <kernels/streams_merge.h>
#include <editd/pattern_compiler.h>
#include <toolchain/cpudriver.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <mutex>

#include <toolchain/NVPTXDriver.h>
#include <editd/editd_gpu_kernel.h>
#ifdef CUDA_ENABLED
#include <editd/EditdCudaDriver.h>
#endif

using namespace llvm;

static cl::list<std::string> inputFiles(cl::Positional, cl::desc("<regex> <input file ...>"), cl::OneOrMore);

static cl::list<std::string> pattVector("e", cl::desc("pattern"), cl::ZeroOrMore);
static cl::opt<std::string> PatternFilename("f", cl::desc("Take patterns (one per line) from a file"), cl::value_desc("regex file"), cl::init(""));

static cl::opt<bool> CaseInsensitive("i", cl::desc("Ignore case distinctions in the pattern and the file."));

static cl::opt<int> editDistance("edit-dist", cl::desc("Edit Distance Value"), cl::init(2));
static cl::opt<int> optPosition("opt-pos", cl::desc("Optimize position"), cl::init(8));
static cl::opt<int> stepSize("step-size", cl::desc("Step Size"), cl::init(3));
static cl::opt<int> prefixLen("prefix", cl::desc("Prefix length"), cl::init(3));
static cl::opt<bool> ShowPositions("display", cl::desc("Display the match positions."), cl::init(false));

static cl::opt<int> Threads("threads", cl::desc("Total number of threads."), cl::init(1));

static cl::opt<bool> MultiEditdKernels("enable-multieditd-kernels", cl::desc("Construct multiple editd kernels in one pipeline."));

using namespace kernel;
using namespace pablo;
using namespace parabix;

const static std::string PTXFilename = "editd.ptx";

struct matchPosition
{
    size_t pos;
    size_t dist;
};

std::vector<struct matchPosition> matchList;
std::vector<std::vector<std::string>> pattGroups;

void run_second_filter(int total_len, int pattern_segs, float errRate){

    if(matchList.empty()) return;

    //remove the duplicates
    bool cleared = true;
    while(cleared){
        cleared = false;
        for (unsigned i=0; i<matchList.size()-1; i++){
            if(matchList[i].pos == matchList[i+1].pos && matchList[i].dist == matchList[i+1].dist){
                matchList.erase(matchList.begin() + i);
                cleared = true;
            }
        }
    }

    //Sort match position
    bool exchanged = true;
    while(exchanged){
        exchanged = false;
        for (unsigned i=0; i<matchList.size()-1; i++){
            if(matchList[i].pos > matchList[i+1].pos){
                size_t tmp_pos = matchList[i].pos;
                size_t tmp_dist = matchList[i].dist;
                matchList[i].pos = matchList[i+1].pos;
                matchList[i].dist = matchList[i+1].dist;
                matchList[i+1].pos = tmp_pos;
                matchList[i+1].dist = tmp_dist;
                exchanged = true;
            }
        }
    }

    std::cout << "pattern_segs = " << pattern_segs << ", total_len = " << total_len << std::endl;

    int v = pattern_segs * (editDistance+1) - total_len * errRate;

    int startPos = matchList[0].pos;
    int sum = matchList[0].dist;
    int curIdx = 0;
    unsigned i = 0;
    int count = 0;
    while (i < matchList.size()){
        if(matchList[i].pos - startPos < total_len * (errRate+1)){
            sum += matchList[i].dist;
            i++;
        }
        else{
            if(sum > v) count++;
            sum -= matchList[curIdx].dist;
            curIdx++;
            startPos = matchList[curIdx].pos;
        }
    }

    std::cout << "total candidate from the first filter is " << matchList.size() << std::endl;
    std::cout << "total candidate from the second filter is " << count << std::endl;
}

void get_editd_pattern(int & pattern_segs, int & total_len) {

    if (PatternFilename != "") {
        std::ifstream pattFile(PatternFilename.c_str());
        std::string r;
        if (pattFile.is_open()) {
            while (std::getline(pattFile, r)) {
                pattVector.push_back(r);
                pattern_segs ++;
                total_len += r.size();
            }
            std::sort(pattVector.begin(), pattVector.end());
            unsigned i = 0;
            while(i < pattVector.size()){
                std::vector<std::string> pattGroup;
                std::string prefix = pattVector[i].substr(0, prefixLen);
                while(i < pattVector.size() && pattVector[i].substr(0, prefixLen) == prefix){
                    pattGroup.push_back(pattVector[i]);
                    i++;
                }
                pattGroups.push_back(pattGroup);
            }
            pattFile.close();
        }
        codegen::GroupNum = pattVector.size();
    }

    // if there are no regexes specified through -e or -f, the first positional argument
    // must be a regex, not an input file.

    if (pattVector.size() == 0) {
        pattVector.push_back(inputFiles[0]);
        inputFiles.erase(inputFiles.begin());
    }
}

class PatternKernel final: public pablo::PabloKernel {
public:
    PatternKernel(const std::unique_ptr<kernel::KernelBuilder> & b, const std::vector<std::string> & patterns);
    std::string makeSignature(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) override;
    bool isCachable() const override { return true;}
protected:
    void generatePabloMethod() override;
private:
    const std::vector<std::string> & mPatterns;
};

PatternKernel::PatternKernel(const std::unique_ptr<kernel::KernelBuilder> & b, const std::vector<std::string> & patterns)
: PabloKernel(b, std::string(patterns[0]), {{b->getStreamSetTy(4), "pat"}}, {{b->getStreamSetTy(editDistance + 1), "E"}})
, mPatterns(patterns) {
}

std::string PatternKernel::makeSignature(const std::unique_ptr<kernel::KernelBuilder> &) {
    return getName();
}

void PatternKernel::generatePabloMethod() {
    PabloBuilder entry(getEntryBlock());
    Var * const pat = getInputStreamVar("pat");
    PabloAST * basisBits[4];
    basisBits[0] = entry.createExtract(pat, 0, "A");
    basisBits[1] = entry.createExtract(pat, 1, "C");
    basisBits[2] = entry.createExtract(pat, 2, "T");
    basisBits[3] = entry.createExtract(pat, 3, "G");
    re::Pattern_Compiler pattern_compiler(*this);
    pattern_compiler.compile(mPatterns, entry, basisBits, editDistance, optPosition, stepSize);
}

std::mutex store_mutex;
void wrapped_report_pos(size_t match_pos, int dist) {
    struct matchPosition curMatch;
    curMatch.pos = match_pos;
    curMatch.dist = dist;

    store_mutex.lock();
    matchList.push_back(curMatch);
    if(ShowPositions)
        std::cout << "pos: " << match_pos << ", dist:" << dist << "\n";
    store_mutex.unlock();
}

void editdPipeline(ParabixDriver & pxDriver, const std::vector<std::string> & patterns) {

    auto & idb = pxDriver.getBuilder();
    Module * const m = idb->getModule();
    Type * const sizeTy = idb->getSizeTy();
    Type * const voidTy = idb->getVoidTy();
    Type * const inputType = PointerType::get(ArrayType::get(ArrayType::get(idb->getBitBlockType(), 8), 1), 0);

    idb->LinkFunction("wrapped_report_pos", &wrapped_report_pos);

    const unsigned segmentSize = codegen::SegmentSize;
    const unsigned bufferSegments = codegen::BufferSegments;

    Function * const main = cast<Function>(m->getOrInsertFunction("Main", voidTy, inputType, sizeTy, nullptr));
    main->setCallingConv(CallingConv::C);
    auto args = main->arg_begin();
    Value * const inputStream = &*(args++);
    inputStream->setName("input");
    Value * const fileSize = &*(args++);
    fileSize->setName("fileSize");
    idb->SetInsertPoint(BasicBlock::Create(m->getContext(), "entry", main,0));

    auto ChStream = pxDriver.addBuffer(make_unique<SourceBuffer>(idb, idb->getStreamSetTy(4)));
    auto mmapK = pxDriver.addKernelInstance(make_unique<MemorySourceKernel>(idb, inputType));
    mmapK->setInitialArguments({inputStream, fileSize});
    pxDriver.makeKernelCall(mmapK, {}, {ChStream});

    auto MatchResults = pxDriver.addBuffer(make_unique<CircularBuffer>(idb, idb->getStreamSetTy(editDistance + 1), segmentSize * bufferSegments));
    auto editdk = pxDriver.addKernelInstance(make_unique<PatternKernel>(idb, patterns));
    pxDriver.makeKernelCall(editdk, {ChStream}, {MatchResults});

    auto editdScanK = pxDriver.addKernelInstance(make_unique<editdScanKernel>(idb, editDistance));
    pxDriver.makeKernelCall(editdScanK, {MatchResults}, {});

    pxDriver.generatePipelineIR();

    idb->CreateRetVoid();

    pxDriver.finalizeObject();
}


void multiEditdPipeline(ParabixDriver & pxDriver) {

    auto & idb = pxDriver.getBuilder();
    Module * const m = idb->getModule();
    Type * const sizeTy = idb->getSizeTy();
    Type * const voidTy = idb->getVoidTy();
    Type * const inputType = PointerType::get(ArrayType::get(ArrayType::get(idb->getBitBlockType(), 8), 1), 0);

    idb->LinkFunction("wrapped_report_pos", &wrapped_report_pos);

    const unsigned segmentSize = codegen::SegmentSize;
    const unsigned bufferSegments = codegen::BufferSegments;

    Function * const main = cast<Function>(m->getOrInsertFunction("Main", voidTy, inputType, sizeTy, nullptr));
    main->setCallingConv(CallingConv::C);
    auto args = main->arg_begin();
    Value * const inputStream = &*(args++);
    inputStream->setName("input");
    Value * const fileSize = &*(args++);
    fileSize->setName("fileSize");
    idb->SetInsertPoint(BasicBlock::Create(m->getContext(), "entry", main,0));

    auto ChStream = pxDriver.addBuffer(make_unique<SourceBuffer>(idb, idb->getStreamSetTy(4)));
    auto mmapK = pxDriver.addKernelInstance(make_unique<MemorySourceKernel>(idb, inputType));
    mmapK->setInitialArguments({inputStream, fileSize});
    pxDriver.makeKernelCall(mmapK, {}, {ChStream});

    const auto n = pattGroups.size();
    
    std::vector<StreamSetBuffer *> MatchResultsBufs(n);
    
    for(unsigned i = 0; i < n; ++i){
        auto MatchResults = pxDriver.addBuffer(make_unique<CircularBuffer>(idb, idb->getStreamSetTy(editDistance + 1), segmentSize * bufferSegments));
        auto editdk = pxDriver.addKernelInstance(make_unique<PatternKernel>(idb, pattGroups[i]));
        pxDriver.makeKernelCall(editdk, {ChStream}, {MatchResults});
        MatchResultsBufs[i] = MatchResults;
    }
    StreamSetBuffer * MergedResults = MatchResultsBufs[0];
    if (n > 1) {
        MergedResults = pxDriver.addBuffer(make_unique<CircularBuffer>(idb, idb->getStreamSetTy(editDistance + 1), segmentSize * bufferSegments));
        kernel::Kernel * streamsMergeK = pxDriver.addKernelInstance(make_unique<kernel::StreamsMerge>(idb, editDistance + 1, n));
        pxDriver.makeKernelCall(streamsMergeK, MatchResultsBufs, {MergedResults});
    }

    auto editdScanK = pxDriver.addKernelInstance(make_unique<editdScanKernel>(idb, editDistance));
    pxDriver.makeKernelCall(editdScanK, {MergedResults}, {});

    pxDriver.generatePipelineIR();

    idb->CreateRetVoid();

    pxDriver.finalizeObject();
}

class PreprocessKernel final: public pablo::PabloKernel {
public:
    PreprocessKernel(const std::unique_ptr<kernel::KernelBuilder> & b);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;
};

PreprocessKernel::PreprocessKernel(const std::unique_ptr<kernel::KernelBuilder> & b)
: PabloKernel(b, "ccc", {{b->getStreamSetTy(8), "basis"}}, {{b->getStreamSetTy(4), "pat"}}) {

}

void PreprocessKernel::generatePabloMethod() {
    cc::CC_Compiler ccc(this, getInputStreamVar("basis"));
    PabloBuilder & pb = ccc.getBuilder();
    PabloAST * A = ccc.compileCC(re::makeCC(re::makeCC(0x41), re::makeCC(0x61)), pb);
    PabloAST * C = ccc.compileCC(re::makeCC(re::makeCC(0x43), re::makeCC(0x63)), pb);
    PabloAST * T = ccc.compileCC(re::makeCC(re::makeCC(0x54), re::makeCC(0x74)), pb);
    PabloAST * G = ccc.compileCC(re::makeCC(re::makeCC(0x47), re::makeCC(0x67)), pb);
    Var * const pat = getOutputStreamVar("pat");
    pb.createAssign(pb.createExtract(pat, 0), A);
    pb.createAssign(pb.createExtract(pat, 1), C);
    pb.createAssign(pb.createExtract(pat, 2), T);
    pb.createAssign(pb.createExtract(pat, 3), G);
}

void preprocessPipeline(ParabixDriver & pxDriver) {

    auto & iBuilder = pxDriver.getBuilder();
    Module * m = iBuilder->getModule();

    Type * const voidTy = iBuilder->getVoidTy();
    Type * const int32Ty = iBuilder->getInt32Ty();
    Type * const outputType = PointerType::get(ArrayType::get(iBuilder->getBitBlockType(), 4), 0);

    const unsigned segmentSize = codegen::SegmentSize;
    const unsigned bufferSegments = codegen::BufferSegments;

    Function * const main = cast<Function>(m->getOrInsertFunction("Main", voidTy, int32Ty, outputType, nullptr));
    main->setCallingConv(CallingConv::C);
    Function::arg_iterator args = main->arg_begin();

    Value * const fileDescriptor = &*(args++);
    fileDescriptor->setName("fileDescriptor");
    Value * const outputStream = &*(args++);
    outputStream->setName("output");

    iBuilder->SetInsertPoint(BasicBlock::Create(m->getContext(), "entry", main));

    auto ByteStream = pxDriver.addBuffer(make_unique<SourceBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 8)));

    auto mmapK = pxDriver.addKernelInstance(make_unique<MMapSourceKernel>(iBuilder, 1));
    mmapK->setInitialArguments({fileDescriptor});
    pxDriver.makeKernelCall(mmapK, {}, {ByteStream});

    auto BasisBits = pxDriver.addBuffer(make_unique<CircularBuffer>(iBuilder, iBuilder->getStreamSetTy(8), segmentSize * bufferSegments));
    auto s2pk = pxDriver.addKernelInstance(make_unique<S2PKernel>(iBuilder));
    pxDriver.makeKernelCall(s2pk, {ByteStream}, {BasisBits});

    auto CCResults = pxDriver.addExternalBuffer(make_unique<ExternalBuffer>(iBuilder, iBuilder->getStreamSetTy(4), outputStream));
    auto ccck = pxDriver.addKernelInstance(make_unique<PreprocessKernel>(iBuilder));
    pxDriver.makeKernelCall(ccck, {BasisBits}, {CCResults});

    pxDriver.generatePipelineIR();

    iBuilder->CreateRetVoid();

    pxDriver.finalizeObject();
}


typedef void (*preprocessFunctionType)(const int fd, char * output_data);

typedef void (*editdFunctionType)(char * byte_data, size_t filesize);

static char * chStream;
static size_t size;

size_t file_size(const int fd) {
    struct stat st;
    if (LLVM_UNLIKELY(fstat(fd, &st) != 0)) {
        st.st_size = 0;
    }
    return st.st_size;
}

char * preprocess(preprocessFunctionType fn_ptr) {
    std::string fileName = inputFiles[0];
    const int fd = open(inputFiles[0].c_str(), O_RDONLY);
    if (LLVM_UNLIKELY(fd == -1)) {
        std::cerr << "Error: cannot open " << fileName << " for processing. Skipped.\n";
        exit(-1);
    }
    size = file_size(fd);
    int ret = posix_memalign((void**)&chStream, 32, size);
    if (ret) {
        std::cerr << "Cannot allocate memory for output.\n";
        exit(-2);
    }
    fn_ptr(fd, chStream);
    close(fd);
    return chStream;
}

void editd(editdFunctionType fn_ptr, char * inputStream, size_t size) {
    fn_ptr(inputStream, size);
}

std::mutex count_mutex;
size_t groupCount;
void * DoEditd(void *)
{
    size_t groupIdx;
    count_mutex.lock();
    groupIdx = groupCount;
    groupCount++;
    count_mutex.unlock();

    while (groupIdx < pattGroups.size()){

        ParabixDriver pxDriver("editd");
        editdPipeline(pxDriver, pattGroups[groupIdx]);
        auto editd_ptr = reinterpret_cast<editdFunctionType>(pxDriver.getMain());
        editd(editd_ptr, chStream, size);

        count_mutex.lock();
        groupIdx = groupCount;
        groupCount++;
        count_mutex.unlock();
    }

    pthread_exit(NULL);
}

#define GROUPTHREADS 64

void editdGPUCodeGen(unsigned patternLen){
    NVPTXDriver pxDriver("editd");
    auto & iBuilder = pxDriver.getBuilder();
    Module * M = iBuilder->getModule();

    const unsigned segmentSize = codegen::SegmentSize;

    Type * const mBitBlockType = iBuilder->getBitBlockType();
    Type * const inputSizeTy = PointerType::get(iBuilder->getSizeTy(), 1);
    Type * const int32ty = iBuilder->getInt32Ty();
    Type * const voidTy = Type::getVoidTy(M->getContext());
    Type * const inputTy = PointerType::get(ArrayType::get(mBitBlockType, 4), 1);
    Type * const patternPtrTy = PointerType::get(iBuilder->getInt8Ty(), 1);
    Type * const outputTy = PointerType::get(ArrayType::get(mBitBlockType, editDistance+1), 1);
    Type * const stridesTy = PointerType::get(int32ty, 1);

    Function * const main = cast<Function>(M->getOrInsertFunction("Main", voidTy, inputTy, inputSizeTy, patternPtrTy, outputTy, stridesTy, nullptr));
    main->setCallingConv(CallingConv::C);
    Function::arg_iterator args = main->arg_begin();

    Value * const inputStream = &*(args++);
    inputStream->setName("input");
    Value * const inputSizePtr = &*(args++);
    inputSizePtr->setName("inputSizePtr");
    Value * const pattStream = &*(args++);
    pattStream->setName("pattStream");
    Value * const resultStream = &*(args++);
    resultStream->setName("resultStream");
    Value * const stridesPtr = &*(args++);
    stridesPtr->setName("stridesPtr");

    iBuilder->SetInsertPoint(BasicBlock::Create(M->getContext(), "entry", main,0));

    Function * tidFunc = M->getFunction("llvm.nvvm.read.ptx.sreg.tid.x");
    Value * tid = iBuilder->CreateCall(tidFunc);
    Function * bidFunc = cast<Function>(M->getOrInsertFunction("llvm.nvvm.read.ptx.sreg.ctaid.x", int32ty, nullptr));
    Value * bid = iBuilder->CreateCall(bidFunc);

    Value * inputThreadPtr = iBuilder->CreateGEP(inputStream, tid);
    Value * strides = iBuilder->CreateLoad(stridesPtr);
    Value * outputBlocks = iBuilder->CreateMul(strides, ConstantInt::get(int32ty, GROUPTHREADS));
    Value * resultStreamPtr = iBuilder->CreateGEP(resultStream, iBuilder->CreateAdd(iBuilder->CreateMul(bid, outputBlocks), tid));
    Value * inputSize = iBuilder->CreateLoad(inputSizePtr);

    StreamSetBuffer * CCStream = pxDriver.addBuffer(make_unique<SourceBuffer>(iBuilder, iBuilder->getStreamSetTy(4), 1));
    kernel::Kernel * sourceK = pxDriver.addKernelInstance(make_unique<kernel::MemorySourceKernel>(iBuilder, inputTy, segmentSize));
    sourceK->setInitialArguments({inputThreadPtr, inputSize});
    pxDriver.makeKernelCall(sourceK, {}, {CCStream});

    ExternalBuffer * ResultStream = pxDriver.addExternalBuffer(make_unique<ExternalBuffer>(iBuilder, iBuilder->getStreamSetTy(editDistance+1), resultStreamPtr, 1));   
    kernel::Kernel * editdk = pxDriver.addKernelInstance(make_unique<kernel::editdGPUKernel>(iBuilder, editDistance, patternLen));
      
    const unsigned numOfCarries = patternLen * (editDistance + 1) * 4;
    Type * strideCarryTy = ArrayType::get(mBitBlockType, numOfCarries);
    Value * strideCarry = iBuilder->CreateAlloca(strideCarryTy);
    iBuilder->CreateStore(Constant::getNullValue(strideCarryTy), strideCarry);

    editdk->setInitialArguments({pattStream, strideCarry});
    pxDriver.makeKernelCall(editdk, {CCStream}, {ResultStream});

    pxDriver.generatePipelineIR();

    iBuilder->CreateRetVoid();

    pxDriver.finalizeObject();

}

void mergeGPUCodeGen(){
    NVPTXDriver pxDriver("merge");
    auto & iBuilder = pxDriver.getBuilder();
    Module * M = iBuilder->getModule();

    Type * const mBitBlockType = iBuilder->getBitBlockType();
    Type * const int32ty = iBuilder->getInt32Ty();
    Type * const voidTy = Type::getVoidTy(M->getContext());
    Type * const resultTy = PointerType::get(ArrayType::get(mBitBlockType, editDistance+1), 1);
    Type * const stridesTy = PointerType::get(int32ty, 1);

    Function * const main = cast<Function>(M->getOrInsertFunction("Main", voidTy, resultTy, stridesTy, nullptr));
    main->setCallingConv(CallingConv::C);
    Function::arg_iterator args = main->arg_begin();

    Value * const resultStream = &*(args++);
    resultStream->setName("resultStream");
    Value * const stridesPtr = &*(args++);
    stridesPtr->setName("stridesPtr");

    BasicBlock * entryBlock = BasicBlock::Create(iBuilder->getContext(), "entryBlock", main, 0);
    BasicBlock * strideLoopCond = BasicBlock::Create(iBuilder->getContext(), "strideLoopCond", main, 0);
    BasicBlock * strideLoopBody = BasicBlock::Create(iBuilder->getContext(), "strideLoopBody", main, 0);
    BasicBlock * stridesDone = BasicBlock::Create(iBuilder->getContext(), "stridesDone", main, 0);

    iBuilder->SetInsertPoint(entryBlock);

    Function * tidFunc = M->getFunction("llvm.nvvm.read.ptx.sreg.tid.x");
    Value * tid = iBuilder->CreateCall(tidFunc);
    Function * bidFunc = cast<Function>(M->getOrInsertFunction("llvm.nvvm.read.ptx.sreg.ctaid.x", int32ty, nullptr));
    Value * bid = iBuilder->CreateCall(bidFunc);

    Value * strides = iBuilder->CreateLoad(stridesPtr);
    Value * strideBlocks = ConstantInt::get(int32ty, iBuilder->getStride() / iBuilder->getBitBlockWidth());
    Value * outputBlocks = iBuilder->CreateMul(strides, strideBlocks);
    Value * resultStreamPtr = iBuilder->CreateGEP(resultStream, tid);

    iBuilder->CreateBr(strideLoopCond);
    iBuilder->SetInsertPoint(strideLoopCond);
    PHINode * strideNo = iBuilder->CreatePHI(int32ty, 2, "strideNo");
    strideNo->addIncoming(ConstantInt::get(int32ty, 0), entryBlock);
    Value * notDone = iBuilder->CreateICmpULT(strideNo, strides);
    iBuilder->CreateCondBr(notDone, strideLoopBody, stridesDone);

    iBuilder->SetInsertPoint(strideLoopBody);
    Value * myResultStreamPtr = iBuilder->CreateGEP(resultStreamPtr, {iBuilder->CreateMul(strideBlocks, strideNo)});
    Value * myResultStream = iBuilder->CreateLoad(iBuilder->CreateGEP(myResultStreamPtr, {iBuilder->getInt32(0), bid}));
    for (int i=1; i<codegen::GroupNum; i++){
        Value * nextStreamPtr = iBuilder->CreateGEP(myResultStreamPtr, {iBuilder->CreateMul(outputBlocks, iBuilder->getInt32(i)), bid});
        myResultStream = iBuilder->CreateOr(myResultStream, iBuilder->CreateLoad(nextStreamPtr));
    }
    iBuilder->CreateStore(myResultStream, iBuilder->CreateGEP(myResultStreamPtr, {iBuilder->getInt32(0), bid}));
    strideNo->addIncoming(iBuilder->CreateAdd(strideNo, ConstantInt::get(int32ty, 1)), strideLoopBody);
    iBuilder->CreateBr(strideLoopCond);

    iBuilder->SetInsertPoint(stridesDone);
    iBuilder->CreateRetVoid();

    pxDriver.finalizeObject();

}

editdFunctionType editdScanCPUCodeGen(ParabixDriver & pxDriver) {
   
    auto & iBuilder = pxDriver.getBuilder();
    Module * M = iBuilder->getModule();

    const unsigned segmentSize = codegen::SegmentSize;

    Type * mBitBlockType = iBuilder->getBitBlockType();
    Type * const size_ty = iBuilder->getSizeTy();
    Type * const voidTy = Type::getVoidTy(M->getContext());
    Type * const inputType = PointerType::get(ArrayType::get(mBitBlockType, editDistance+1), 0);

    Function * const main = cast<Function>(M->getOrInsertFunction("Main", voidTy, inputType, size_ty, nullptr));
    main->setCallingConv(CallingConv::C);
    iBuilder->SetInsertPoint(BasicBlock::Create(M->getContext(), "entry", main, 0));
    Function::arg_iterator args = main->arg_begin();
    Value * const inputStream = &*(args++);
    inputStream->setName("input");
    Value * const fileSize = &*(args++);
    fileSize->setName("fileSize");


    StreamSetBuffer * MatchResults = pxDriver.addBuffer(make_unique<SourceBuffer>(iBuilder, iBuilder->getStreamSetTy(editDistance+1)));
    kernel::Kernel * sourceK = pxDriver.addKernelInstance(make_unique<kernel::MemorySourceKernel>(iBuilder, inputType, segmentSize));
    sourceK->setInitialArguments({inputStream, fileSize});
    pxDriver.makeKernelCall(sourceK, {}, {MatchResults});

    auto editdScanK = pxDriver.addKernelInstance(make_unique<editdScanKernel>(iBuilder, editDistance));
    pxDriver.makeKernelCall(editdScanK, {MatchResults}, {});
        
    pxDriver.generatePipelineIR();
    iBuilder->CreateRetVoid();

    pxDriver.LinkFunction(*editdScanK, "wrapped_report_pos", &wrapped_report_pos);
    pxDriver.finalizeObject();

    return reinterpret_cast<editdFunctionType>(pxDriver.getMain());

}

int main(int argc, char *argv[]) {
    codegen::ParseCommandLineOptions(argc, argv);
    int pattern_segs = 0;
    int total_len = 0;

    get_editd_pattern(pattern_segs, total_len);

#ifdef CUDA_ENABLED
    codegen::BlockSize = 64;
#endif

    ParabixDriver pxDriver("preprocess");
    preprocessPipeline(pxDriver);
    auto preprocess_ptr = reinterpret_cast<preprocessFunctionType>(pxDriver.getMain());
    preprocess(preprocess_ptr);

#ifdef CUDA_ENABLED
    if(codegen::NVPTX){

        std::ifstream t(PatternFilename);
        if (!t.is_open()) {
            std::cerr << "Error: cannot open " << PatternFilename << " for processing. Skipped.\n";
            exit(-1);
        }
        std::string patterns((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());

        editdGPUCodeGen(patterns.length()/codegen::GroupNum - 1);
        mergeGPUCodeGen();
        ulong * rslt = RunPTX(PTXFilename, chStream, size, patterns.c_str(), patterns.length(), editDistance);

        ParabixDriver scanDriver("scan");
        editdFunctionType editd_ptr = editdScanCPUCodeGen(scanDriver);
        editd(editd_ptr, (char*)rslt, size);

        run_second_filter(pattern_segs, total_len, 0.15);

        return 0;
    }
#endif

    if(pattVector.size() == 1){

        ParabixDriver pxDriver("editd");
        editdPipeline(pxDriver, pattVector);
        auto editd_ptr = reinterpret_cast<editdFunctionType>(pxDriver.getMain());
        editd(editd_ptr, chStream, size);
    }
    else{
        if (Threads == 1) {
            if (MultiEditdKernels) {
                ParabixDriver pxDriver("editd");
                multiEditdPipeline(pxDriver);
                auto editd_ptr = reinterpret_cast<editdFunctionType>(pxDriver.getMain());
                editd(editd_ptr, chStream, size);
            }
            else{               
                for(unsigned i=0; i<pattGroups.size(); i++){

                    ParabixDriver pxDriver("editd");
                    editdPipeline(pxDriver, pattGroups[i]);
                    auto editd_ptr = reinterpret_cast<editdFunctionType>(pxDriver.getMain());
                    editd(editd_ptr, chStream, size);
                }
            }
        }
        else{
            const unsigned numOfThreads = Threads;
            pthread_t threads[numOfThreads];
            groupCount = 0;

            for(unsigned long i = 0; i < numOfThreads; ++i){
                const int rc = pthread_create(&threads[i], NULL, DoEditd, (void *)i);
                if (rc) {
                    llvm::report_fatal_error("Failed to create thread: code " + std::to_string(rc));
                }
            }

            for(unsigned i = 0; i < numOfThreads; ++i) {
                void * status = nullptr;
                const int rc = pthread_join(threads[i], &status);
                if (rc) {
                    llvm::report_fatal_error("Failed to join thread: code " + std::to_string(rc));
                }
            }

        }
        run_second_filter(pattern_segs, total_len, 0.15);
    }

    return 0;
}
