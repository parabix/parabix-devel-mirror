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
#include <kernels/cc_kernel.h>
#include <editd/editdscan_kernel.h>
#include <kernels/streams_merge.h>
#include <editd/pattern_compiler.h>
#include <toolchain/cpudriver.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <mutex>
#include <boost/uuid/sha1.hpp>
#include <editd/editd_cpu_kernel.h>

#ifdef CUDA_ENABLED
#include <toolchain/NVPTXDriver.h>
#include <editd/editd_gpu_kernel.h>
#include <editd/EditdCudaDriver.h>
#endif

using namespace llvm;

static cl::list<std::string> inputFiles(cl::Positional, cl::desc("<regex> <input file ...>"), cl::OneOrMore);

static cl::list<std::string> pattVector("e", cl::desc("pattern"), cl::ZeroOrMore);
static cl::opt<std::string> PatternFilename("f", cl::desc("Take patterns (one per line) from a file"), cl::value_desc("regex file"), cl::init(""));

static cl::opt<bool> CaseInsensitive("i", cl::desc("Ignore case distinctions in the pattern and the file."));

static cl::opt<int> editDistance("edit-dist", cl::desc("Edit Distance Value"), cl::init(2));
static cl::opt<int> optPosition("opt-pos", cl::desc("Optimize position"), cl::init(0));
static cl::opt<int> stepSize("step-size", cl::desc("Step Size"), cl::init(3));
static cl::opt<int> prefixLen("prefix", cl::desc("Prefix length"), cl::init(3));
static cl::opt<int> groupSize("groupPatterns", cl::desc("Number of pattern segments per group."), cl::init(1));
static cl::opt<bool> ShowPositions("display", cl::desc("Display the match positions."), cl::init(false));

static cl::opt<int> Threads("threads", cl::desc("Total number of threads."), cl::init(1));

static cl::opt<bool> MultiEditdKernels("enable-multieditd-kernels", cl::desc("Construct multiple editd kernels in one pipeline."));
static cl::opt<bool> EditdIndexPatternKernels("enable-index-kernels", cl::desc("Use pattern index method."));

using namespace kernel;
using namespace pablo;
using namespace parabix;

#ifdef CUDA_ENABLED
const static std::string PTXFilename = "editd.ptx";
#endif

struct matchPosition
{
    size_t pos;
    size_t dist;
};

std::vector<struct matchPosition> matchList;
std::vector<std::vector<std::string>> pattGroups;

void run_second_filter(int pattern_segs, int total_len, float errRate){

    if(matchList.empty()) return;

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
        codegen::GroupNum = pattVector.size()/groupSize;
    }

    // if there are no regexes specified through -e or -f, the first positional argument
    // must be a regex, not an input file.

    if (pattVector.size() == 0 && inputFiles.size() > 1) {
        pattVector.push_back(inputFiles[0]);
        inputFiles.erase(inputFiles.begin());
    }
}

inline static std::string sha1sum(const std::string & str) {
    char buffer[41];    // 40 hex-digits and the terminating null
    uint32_t digest[5]; // 160 bits in total
    boost::uuids::detail::sha1 sha1;
    sha1.process_bytes(str.c_str(), str.size());
    sha1.get_digest(digest);
    snprintf(buffer, sizeof(buffer), "%.8x%.8x%.8x%.8x%.8x",
             digest[0], digest[1], digest[2], digest[3], digest[4]);
    return std::string(buffer);
}

std::string createName(const std::vector<std::string> & patterns) {
    std::string name = "";
    for(unsigned i=0; i<patterns.size(); i++)
        name += patterns[i];
    return name + std::to_string(editDistance);
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
: PabloKernel(b, sha1sum(createName(patterns)), {{b->getStreamSetTy(4), "pat"}}, {{b->getStreamSetTy(editDistance + 1), "E"}})
, mPatterns(patterns) {  
}

std::string PatternKernel::makeSignature(const std::unique_ptr<kernel::KernelBuilder> &) {
    return getName();
}

void PatternKernel::generatePabloMethod() {
    PabloBuilder entry(getEntryScope());
    Var * const pat = getInputStreamVar("pat");
    PabloAST * basisBits[4];
    basisBits[0] = entry.createExtract(pat, 0, "A");
    basisBits[1] = entry.createExtract(pat, 1, "C");
    basisBits[2] = entry.createExtract(pat, 2, "T");
    basisBits[3] = entry.createExtract(pat, 3, "G");
    re::Pattern_Compiler pattern_compiler(*this);
    if (optPosition == 0) optPosition = editDistance + 6;
    pattern_compiler.compile(mPatterns, entry, basisBits, editDistance, optPosition, stepSize);
}

std::mutex store_mutex;
extern "C" void wrapped_report_pos(size_t match_pos, int dist) {
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
    const unsigned bufferSegments = codegen::BufferSegments * codegen::ThreadNum;

    Function * const main = cast<Function>(m->getOrInsertFunction("Main", voidTy, inputType, sizeTy, nullptr));
    main->setCallingConv(CallingConv::C);
    auto args = main->arg_begin();
    Value * const inputStream = &*(args++);
    inputStream->setName("input");
    Value * const fileSize = &*(args++);
    fileSize->setName("fileSize");
    idb->SetInsertPoint(BasicBlock::Create(m->getContext(), "entry", main,0));

    auto ChStream = pxDriver.addBuffer<SourceBuffer>(idb, idb->getStreamSetTy(4));
    auto mmapK = pxDriver.addKernelInstance<MemorySourceKernel>(idb, inputType);
    mmapK->setInitialArguments({inputStream, fileSize});
    pxDriver.makeKernelCall(mmapK, {}, {ChStream});

    auto MatchResults = pxDriver.addBuffer<CircularBuffer>(idb, idb->getStreamSetTy(editDistance + 1), segmentSize * bufferSegments);
    auto editdk = pxDriver.addKernelInstance<PatternKernel>(idb, patterns);
    pxDriver.makeKernelCall(editdk, {ChStream}, {MatchResults});

    auto editdScanK = pxDriver.addKernelInstance<editdScanKernel>(idb, editDistance);
    pxDriver.makeKernelCall(editdScanK, {MatchResults}, {});

    pxDriver.generatePipelineIR();
    pxDriver.deallocateBuffers();
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
    PabloBuilder pb(getEntryScope());
    cc::Parabix_CC_Compiler ccc(getEntryScope(), getInputStreamSet("basis"));
    PabloAST * A = ccc.compileCC(re::makeCC(re::makeCC(0x41), re::makeCC(0x61)));
    PabloAST * C = ccc.compileCC(re::makeCC(re::makeCC(0x43), re::makeCC(0x63)));
    PabloAST * T = ccc.compileCC(re::makeCC(re::makeCC(0x54), re::makeCC(0x74)));
    PabloAST * G = ccc.compileCC(re::makeCC(re::makeCC(0x47), re::makeCC(0x67)));
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
    const unsigned bufferSegments = codegen::BufferSegments * codegen::ThreadNum;

    Function * const main = cast<Function>(m->getOrInsertFunction("Main", voidTy, int32Ty, outputType, nullptr));
    main->setCallingConv(CallingConv::C);
    Function::arg_iterator args = main->arg_begin();

    Value * const fileDescriptor = &*(args++);
    fileDescriptor->setName("fileDescriptor");
    Value * const outputStream = &*(args++);
    outputStream->setName("output");

    iBuilder->SetInsertPoint(BasicBlock::Create(m->getContext(), "entry", main));

    auto ByteStream = pxDriver.addBuffer<SourceBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 8));

    auto mmapK = pxDriver.addKernelInstance<MMapSourceKernel>(iBuilder);
    mmapK->setInitialArguments({fileDescriptor});
    pxDriver.makeKernelCall(mmapK, {}, {ByteStream});

    auto BasisBits = pxDriver.addBuffer<CircularBuffer>(iBuilder, iBuilder->getStreamSetTy(8), segmentSize * bufferSegments);
    auto s2pk = pxDriver.addKernelInstance<S2PKernel>(iBuilder);
    pxDriver.makeKernelCall(s2pk, {ByteStream}, {BasisBits});

    auto CCResults = pxDriver.addBuffer<ExternalBuffer>(iBuilder, iBuilder->getStreamSetTy(4), outputStream);
    auto ccck = pxDriver.addKernelInstance<PreprocessKernel>(iBuilder);
    // NOTE: CCResults are never consumed because they are written directly into an external buffer. This may make analysis difficult.
    pxDriver.makeKernelCall(ccck, {BasisBits}, {CCResults});

    pxDriver.generatePipelineIR();
    pxDriver.deallocateBuffers();
    iBuilder->CreateRetVoid();

    pxDriver.finalizeObject();
}

void multiEditdPipeline(ParabixDriver & pxDriver) {
    auto & idb = pxDriver.getBuilder();
    Module * const m = idb->getModule();
    Type * const voidTy = idb->getVoidTy();
    Type * const int32Ty = idb->getInt32Ty();

    idb->LinkFunction("wrapped_report_pos", &wrapped_report_pos);

    const unsigned segmentSize = codegen::SegmentSize;
    const unsigned bufferSegments = codegen::BufferSegments * codegen::ThreadNum;

    Function * const main = cast<Function>(m->getOrInsertFunction("Main", voidTy, int32Ty, nullptr));
    main->setCallingConv(CallingConv::C);
    Function::arg_iterator args = main->arg_begin();

    Value * const fileDescriptor = &*(args++);
    fileDescriptor->setName("fileDescriptor");

    idb->SetInsertPoint(BasicBlock::Create(m->getContext(), "entry", main,0));

    auto ByteStream = pxDriver.addBuffer<SourceBuffer>(idb, idb->getStreamSetTy(1, 8));

    auto mmapK = pxDriver.addKernelInstance<MMapSourceKernel>(idb);
    mmapK->setInitialArguments({fileDescriptor});
    pxDriver.makeKernelCall(mmapK, {}, {ByteStream});

    auto ChStream = pxDriver.addBuffer<CircularBuffer>(idb, idb->getStreamSetTy(4), segmentSize * bufferSegments);
    auto ccck = pxDriver.addKernelInstance<kernel::DirectCharacterClassKernelBuilder>(idb, "ccc", 
        std::vector<re::CC *>{re::makeCC(re::makeCC(0x41), re::makeCC(0x61)),
                              re::makeCC(re::makeCC(0x43), re::makeCC(0x63)),
                              re::makeCC(re::makeCC(0x54), re::makeCC(0x74)),
                              re::makeCC(re::makeCC(0x47), re::makeCC(0x67))});
    pxDriver.makeKernelCall(ccck, {ByteStream}, {ChStream});

    const auto n = pattGroups.size();
    
    std::vector<StreamSetBuffer *> MatchResultsBufs(n);
    
    for(unsigned i = 0; i < n; ++i){
        auto MatchResults = pxDriver.addBuffer<CircularBuffer>(idb, idb->getStreamSetTy(editDistance + 1), segmentSize * bufferSegments);
        auto editdk = pxDriver.addKernelInstance<PatternKernel>(idb, pattGroups[i]);
        pxDriver.makeKernelCall(editdk, {ChStream}, {MatchResults});
        MatchResultsBufs[i] = MatchResults;
    }
    StreamSetBuffer * MergedResults = MatchResultsBufs[0];
    if (n > 1) {
        MergedResults = pxDriver.addBuffer<CircularBuffer>(idb, idb->getStreamSetTy(editDistance + 1), segmentSize * bufferSegments);
        kernel::Kernel * streamsMergeK = pxDriver.addKernelInstance<kernel::StreamsMerge>(idb, editDistance + 1, n);
        pxDriver.makeKernelCall(streamsMergeK, MatchResultsBufs, {MergedResults});
    }

    auto editdScanK = pxDriver.addKernelInstance<editdScanKernel>(idb, editDistance);
    pxDriver.makeKernelCall(editdScanK, {MergedResults}, {});

    pxDriver.generatePipelineIR();
    pxDriver.deallocateBuffers();
    idb->CreateRetVoid();

    pxDriver.finalizeObject();
}


void editdIndexPatternPipeline(ParabixDriver & pxDriver, unsigned patternLen) {

    auto & idb = pxDriver.getBuilder();
    Module * const m = idb->getModule();
    Type * const sizeTy = idb->getSizeTy();
    Type * const voidTy = idb->getVoidTy();
    Type * const inputType = PointerType::get(ArrayType::get(ArrayType::get(idb->getBitBlockType(), 8), 1), 0);
    Type * const patternPtrTy = PointerType::get(idb->getInt8Ty(), 0);

    idb->LinkFunction("wrapped_report_pos", &wrapped_report_pos);

    const unsigned segmentSize = codegen::SegmentSize;
    const unsigned bufferSegments = codegen::BufferSegments * codegen::ThreadNum;

    Function * const main = cast<Function>(m->getOrInsertFunction("Main", voidTy, inputType, sizeTy, patternPtrTy, nullptr));
    main->setCallingConv(CallingConv::C);
    auto args = main->arg_begin();
    Value * const inputStream = &*(args++);
    inputStream->setName("input");
    Value * const fileSize = &*(args++);
    fileSize->setName("fileSize");
    Value * const pattStream = &*(args++);
    pattStream->setName("pattStream");
    idb->SetInsertPoint(BasicBlock::Create(m->getContext(), "entry", main,0));

    auto ChStream = pxDriver.addBuffer<SourceBuffer>(idb, idb->getStreamSetTy(4));
    auto mmapK = pxDriver.addKernelInstance<MemorySourceKernel>(idb, inputType);
    mmapK->setInitialArguments({inputStream, fileSize});
    pxDriver.makeKernelCall(mmapK, {}, {ChStream});

    auto MatchResults = pxDriver.addBuffer<CircularBuffer>(idb, idb->getStreamSetTy(editDistance + 1), segmentSize * bufferSegments);
    auto editdk = pxDriver.addKernelInstance<kernel::editdCPUKernel>(idb, editDistance, patternLen, groupSize);

    const unsigned numOfCarries = patternLen * (editDistance + 1) * 4 * groupSize;
    Type * strideCarryTy = ArrayType::get(idb->getBitBlockType(), numOfCarries);
    Value * strideCarry = idb->CreateAlloca(strideCarryTy);
    idb->CreateStore(Constant::getNullValue(strideCarryTy), strideCarry);

    editdk->setInitialArguments({pattStream, strideCarry});
    pxDriver.makeKernelCall(editdk, {ChStream}, {MatchResults});

    auto editdScanK = pxDriver.addKernelInstance<editdScanKernel>(idb, editDistance);
    pxDriver.makeKernelCall(editdScanK, {MatchResults}, {});

    pxDriver.generatePipelineIR();

    idb->CreateRetVoid();

    pxDriver.finalizeObject();
}

typedef void (*preprocessFunctionType)(const int fd, char * output_data);

typedef void (*editdFunctionType)(char * byte_data, size_t filesize);

typedef void (*multiEditdFunctionType)(const int fd);

typedef void (*editdIndexFunctionType)(char * byte_data, size_t filesize, const char * pattern);

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

#ifdef CUDA_ENABLED
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
    auto args = main->arg_begin();

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
    Value * outputBlocks = iBuilder->CreateMul(strides, ConstantInt::get(int32ty, iBuilder->getStride() / iBuilder->getBitBlockWidth()));
    Value * resultStreamPtr = iBuilder->CreateGEP(resultStream, iBuilder->CreateAdd(iBuilder->CreateMul(bid, outputBlocks), tid));
    Value * inputSize = iBuilder->CreateLoad(inputSizePtr);

    auto CCStream = pxDriver.addBuffer<SourceBuffer>(iBuilder, iBuilder->getStreamSetTy(4), 1);
    auto sourceK = pxDriver.addKernelInstance<kernel::MemorySourceKernel>(iBuilder, inputTy, segmentSize);
    sourceK->setInitialArguments({inputThreadPtr, inputSize});
    pxDriver.makeKernelCall(sourceK, {}, {CCStream});

    auto ResultStream = pxDriver.addBuffer<ExternalBuffer>(iBuilder, iBuilder->getStreamSetTy(editDistance+1), resultStreamPtr, 1);
    auto editdk = pxDriver.addKernelInstance<kernel::editdGPUKernel>(iBuilder, editDistance, patternLen, groupSize);
      
    const unsigned numOfCarries = patternLen * (editDistance + 1) * 4 * groupSize;
    Type * strideCarryTy = ArrayType::get(mBitBlockType, numOfCarries);
    Value * strideCarry = iBuilder->CreateAlloca(strideCarryTy);
    iBuilder->CreateStore(Constant::getNullValue(strideCarryTy), strideCarry);

    editdk->setInitialArguments({pattStream, strideCarry});
    pxDriver.makeKernelCall(editdk, {CCStream}, {ResultStream});

    pxDriver.generatePipelineIR();
    pxDriver.deallocateBuffers();
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
#endif

editdFunctionType editdScanCPUCodeGen(ParabixDriver & pxDriver) {
   
    auto & iBuilder = pxDriver.getBuilder();
    Module * M = iBuilder->getModule();

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

    StreamSetBuffer * MatchResults = pxDriver.addBuffer<SourceBuffer>(iBuilder, iBuilder->getStreamSetTy(editDistance+1));
    kernel::Kernel * sourceK = pxDriver.addKernelInstance<kernel::MemorySourceKernel>(iBuilder, inputType);
    sourceK->setInitialArguments({inputStream, fileSize});
    pxDriver.makeKernelCall(sourceK, {}, {MatchResults});

    auto editdScanK = pxDriver.addKernelInstance<editdScanKernel>(iBuilder, editDistance);
    pxDriver.makeKernelCall(editdScanK, {MatchResults}, {});
    pxDriver.LinkFunction(*editdScanK, "wrapped_report_pos", &wrapped_report_pos);
    pxDriver.generatePipelineIR();
    pxDriver.deallocateBuffers();
    iBuilder->CreateRetVoid();

    pxDriver.finalizeObject();

    return reinterpret_cast<editdFunctionType>(pxDriver.getMain());

}

int main(int argc, char *argv[]) {
    codegen::ParseCommandLineOptions(argc, argv);
    int pattern_segs = 0;
    int total_len = 0;

    get_editd_pattern(pattern_segs, total_len);

    if (MultiEditdKernels) {
        ParabixDriver pxDriver("editd");
        multiEditdPipeline(pxDriver);
        auto editd_ptr = reinterpret_cast<multiEditdFunctionType>(pxDriver.getMain());

        std::string fileName = inputFiles[0];
        const int fd = open(inputFiles[0].c_str(), O_RDONLY);
        if (LLVM_UNLIKELY(fd == -1)) {
            std::cerr << "Error: cannot open " << fileName << " for processing. Skipped.\n";
            exit(-1);
        }
        editd_ptr(fd);
        close(fd);
        run_second_filter(pattern_segs, total_len, 0.15);
        return 0;
    }

#ifdef CUDA_ENABLED
    if (codegen::NVPTX)
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

        editdGPUCodeGen(pattVector[0].length());
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
        std::cout << "total matches is " << matchList.size() << std::endl;
    }
    else{
        if (Threads == 1) { 
            if (EditdIndexPatternKernels) {
                ParabixDriver pxDriver("editd");
                editdIndexPatternPipeline(pxDriver, pattVector[0].length());
                auto editd_ptr = reinterpret_cast<editdIndexFunctionType>(pxDriver.getMain());

                for(unsigned i=0; i<pattVector.size(); i+=groupSize){
                    std::string pattern = "";
                    for (int j=0; j<groupSize; j++){
                        pattern += pattVector[i+j];
                    }
                    editd_ptr(chStream, size, pattern.c_str());
                }
            }
            else {
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
