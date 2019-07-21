/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "editdscan_kernel.h"
#include "pattern_compiler.h"
#include "editd_cpu_kernel.h"
#include <string>
#include <iostream>
#include <fstream>
#include <toolchain/toolchain.h>
#include <pablo/pablo_toolchain.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/Module.h>
#include <llvm/Support/CommandLine.h>
#include <kernel/basis/s2p_kernel.h>
#include <kernel/core/kernel_builder.h>
#include <kernel/core/idisa_target.h>
#include <kernel/core/streamset.h>
#include <kernel/io/source_kernel.h>
#include <kernel/streamutils/streams_merge.h>
#include <pablo/pablo_compiler.h>
#include <pablo/pablo_kernel.h>
#include <re/cc/cc_compiler.h>
#include <re/cc/cc_compiler_target.h>
#include <re/cc/cc_kernel.h>
#include <kernel/pipeline/driver/cpudriver.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <mutex>
#include <kernel/pipeline/pipeline_builder.h>
#include <util/aligned_allocator.h>

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

//TODO: make a "CBuffer" class to abstract away the complexity of making these function typedefs.

typedef void (*preprocessFunctionType)(char * output_data, size_t * output_produced, size_t output_size, const uint32_t fd);

static char * chStream;
static size_t fsize;

class PreprocessKernel final: public pablo::PabloKernel {
public:
    PreprocessKernel(const std::unique_ptr<KernelBuilder> & b, StreamSet * BasisBits, StreamSet * CCResults);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;
};

PreprocessKernel::PreprocessKernel(const std::unique_ptr<KernelBuilder> & b, StreamSet * BasisBits, StreamSet * CCResults)
: PabloKernel(b, "editd_preprocess", {{"basis", BasisBits}}, {{"pat", CCResults}}) {

}

void PreprocessKernel::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    cc::Parabix_CC_Compiler_Builder ccc(getEntryScope(), getInputStreamSet("basis"));
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

preprocessFunctionType preprocessPipeline(CPUDriver & pxDriver) {
    StreamSet * const CCResults = pxDriver.CreateStreamSet(4);
    auto & b = pxDriver.getBuilder();
    Type * const int32Ty = b->getInt32Ty();
    auto P = pxDriver.makePipelineWithIO({}, {{"CCResults", CCResults}}, {{int32Ty, "fileDescriptor"}});
    Scalar * const fileDescriptor = P->getInputScalar("fileDescriptor");
    StreamSet * const ByteStream = P->CreateStreamSet(1, 8);
    P->CreateKernelCall<MMapSourceKernel>(fileDescriptor, ByteStream);
    StreamSet * const BasisBits = P->CreateStreamSet(8);
    P->CreateKernelCall<S2PKernel>(ByteStream, BasisBits);
    P->CreateKernelCall<PreprocessKernel>(BasisBits, CCResults);
    return reinterpret_cast<preprocessFunctionType>(P->compile());
}

size_t file_size(const int fd) {
    struct stat st;
    if (LLVM_UNLIKELY(fstat(fd, &st) != 0)) {
        st.st_size = 0;
    }
    return st.st_size;
}

#define ALIGNMENT (512 / 8)

inline size_t round_up_to(const size_t x, const size_t y) {
    assert(is_power_2(y));
    return (x + y - 1) & -y;
}

char * preprocess(preprocessFunctionType preprocess) {
    std::string fileName = inputFiles[0];
    const auto fd = open(inputFiles[0].c_str(), O_RDONLY);
    if (LLVM_UNLIKELY(fd == -1)) {
        std::cerr << "Error: cannot open " << fileName << " for processing.\n";
        exit(-1);
    }
    fsize = file_size(fd);

    // Given a 8-bit bytestream of length n, we need space for 4 bitstreams of length n ...
    AlignedAllocator<char, ALIGNMENT> alloc;
    const size_t n = round_up_to(fsize, 8 * ALIGNMENT);
    chStream = alloc.allocate((4 * n) / 8);
    size_t length = 0;
    preprocess(chStream, &length, n, fd);
    close(fd);
    return chStream;
}


std::string createName(const std::vector<std::string> & patterns) {
    std::string name = "";
    for(unsigned i=0; i<patterns.size(); i++)
        name += patterns[i];
    return name + std::to_string(editDistance);
}

class PatternKernel final: public pablo::PabloKernel {
public:
    PatternKernel(const std::unique_ptr<KernelBuilder> & b, const std::vector<std::string> & patterns, StreamSet * pat, StreamSet * E);
    std::string makeSignature(const std::unique_ptr<KernelBuilder> &) const override;
    bool isCachable() const override { return true;}
protected:
    void generatePabloMethod() override;
private:
    const std::vector<std::string> & mPatterns;
};

PatternKernel::PatternKernel(const std::unique_ptr<KernelBuilder> & b, const std::vector<std::string> & patterns, StreamSet * pat, StreamSet * E)
: PabloKernel(b, getStringHash(createName(patterns)),
{{"pat", pat}},
{{"E", E}})
, mPatterns(patterns) {
    addAttribute(InfrequentlyUsed());
}

std::string PatternKernel::makeSignature(const std::unique_ptr<KernelBuilder> &) const {
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

typedef void (*editdFunctionType)(char * byte_data, size_t filesize);

editdFunctionType editdPipeline(CPUDriver & pxDriver, const std::vector<std::string> & patterns) {
    auto & b = pxDriver.getBuilder();
    Type * const sizeTy = b->getSizeTy();
    Type * const inputType = b->getIntNTy(1)->getPointerTo();
    auto P = pxDriver.makePipeline({Binding{inputType, "input"}, Binding{sizeTy, "fileSize"}});
    Scalar * const inputStream = P->getInputScalar("input");
    Scalar * const fileSize = P->getInputScalar("fileSize");
    b->LinkFunction("wrapped_report_pos", wrapped_report_pos);
    StreamSet * const ChStream = P->CreateStreamSet(4);
    P->CreateKernelCall<MemorySourceKernel>(inputStream, fileSize, ChStream);
    StreamSet * const MatchResults = P->CreateStreamSet(editDistance + 1);
    P->CreateKernelCall<PatternKernel>(patterns, ChStream, MatchResults);
    P->CreateKernelCall<editdScanKernel>(MatchResults);
    return reinterpret_cast<editdFunctionType>(P->compile());
}

typedef void (*multiEditdFunctionType)(const int fd);

multiEditdFunctionType multiEditdPipeline(CPUDriver & pxDriver) {

    auto & b = pxDriver.getBuilder();
    auto P = pxDriver.makePipeline({Binding{b->getInt32Ty(), "fileDescriptor"}});
    b->LinkFunction("wrapped_report_pos", wrapped_report_pos);
    Scalar * const fileDescriptor = P->getInputScalar("fileDescriptor");

    StreamSet * const ByteStream = P->CreateStreamSet(1, 8);
    P->CreateKernelCall<MMapSourceKernel>(fileDescriptor, ByteStream);

    std::vector<re::CC *> ccs;
    ccs.emplace_back(re::makeCC(re::makeCC(0x41), re::makeCC(0x61)));
    ccs.emplace_back(re::makeCC(re::makeCC(0x43), re::makeCC(0x63)));
    ccs.emplace_back(re::makeCC(re::makeCC(0x47), re::makeCC(0x67)));
    ccs.emplace_back(re::makeCC(re::makeCC(0x54), re::makeCC(0x74)));

    StreamSet * const ChStream = P->CreateStreamSet(4);
    P->CreateKernelCall<CharacterClassKernelBuilder>("editd_cc", ccs, ByteStream, ChStream);

    const auto n = pattGroups.size();
    std::vector<StreamSet *> MatchResults(n);
    for(unsigned i = 0; i < n; ++i){
        MatchResults[i] = P->CreateStreamSet(editDistance + 1);
        P->CreateKernelCall<PatternKernel>(pattGroups[i], ChStream, MatchResults[i]);
    }

    StreamSet * MergedResults = MatchResults[0];
    if (n > 1) {
        StreamSet * const MergedResults = P->CreateStreamSet();
        P->CreateKernelCall<StreamsMerge>(MatchResults, MergedResults);
    }
    P->CreateKernelCall<editdScanKernel>(MergedResults);

    return reinterpret_cast<multiEditdFunctionType>(P->compile());
}

typedef void (*editdIndexFunctionType)(char * byte_data, size_t filesize, const char * pattern);

editdIndexFunctionType editdIndexPatternPipeline(CPUDriver & pxDriver, unsigned patternLen) {

    auto & b = pxDriver.getBuilder();

    Type * const inputType = b->getIntNTy(1)->getPointerTo();
    Type * const sizeTy = b->getSizeTy();
    Type * const patternPtrTy = PointerType::get(b->getInt8Ty(), 0);

    auto P = pxDriver.makePipeline({Binding{inputType, "input"}, Binding{sizeTy, "fileSize"}, Binding{patternPtrTy, "pattStream"}});
    Scalar * const inputStream = P->getInputScalar("input");
    Scalar * const fileSize = P->getInputScalar("fileSize");
    Scalar * const pattStream = P->getInputScalar("pattStream");

    b->LinkFunction("wrapped_report_pos", wrapped_report_pos);

    StreamSet * const ChStream = P->CreateStreamSet(4);
    P->CreateKernelCall<MemorySourceKernel>(inputStream, fileSize, ChStream);

    StreamSet * const MatchResults = P->CreateStreamSet(editDistance + 1);

    P->CreateKernelCall<editdCPUKernel>(patternLen, groupSize, pattStream, ChStream, MatchResults);

    P->CreateKernelCall<editdScanKernel>(MatchResults);

    return reinterpret_cast<editdIndexFunctionType>(P->compile());
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

        CPUDriver pxDriver("editd");
        auto editd = editdPipeline(pxDriver, pattGroups[groupIdx]);
        editd(chStream, fsize);

        count_mutex.lock();
        groupIdx = groupCount;
        groupCount++;
        count_mutex.unlock();
    }

    pthread_exit(NULL);
}

int main(int argc, char *argv[]) {
    codegen::ParseCommandLineOptions(argc, argv);
    int pattern_segs = 0;
    int total_len = 0;

    get_editd_pattern(pattern_segs, total_len);

    if (MultiEditdKernels) {
        CPUDriver pxDriver("editd");
        auto editd = multiEditdPipeline(pxDriver);

        std::string fileName = inputFiles[0];
        const int fd = open(inputFiles[0].c_str(), O_RDONLY);
        if (LLVM_UNLIKELY(fd == -1)) {
            std::cerr << "Error: cannot open " << fileName << " for processing. Skipped.\n";
            exit(-1);
        }
        editd(fd);
        close(fd);
        run_second_filter(pattern_segs, total_len, 0.15);
        return 0;
    }

    CPUDriver pxDriver("preprocess");
    auto preprocess_ptr = preprocessPipeline(pxDriver);
    preprocess(preprocess_ptr);

    if(pattVector.size() == 1){

        CPUDriver pxDriver("editd");
        auto editd = editdPipeline(pxDriver, pattVector);
        editd(chStream, fsize);
        std::cout << "total matches is " << matchList.size() << std::endl;
    }
    else{
        if (Threads == 1) {
            if (EditdIndexPatternKernels) {
                CPUDriver pxDriver("editd");
                auto editd_ptr = editdIndexPatternPipeline(pxDriver, pattVector[0].length());

                for(unsigned i=0; i<pattVector.size(); i+=groupSize){
                    std::string pattern = "";
                    for (int j=0; j<groupSize; j++){
                        pattern += pattVector[i+j];
                    }
                    editd_ptr(chStream, fsize, pattern.c_str());
                }
            }
            else {
                for(unsigned i=0; i<pattGroups.size(); i++){

                    CPUDriver pxDriver("editd");
                    auto editd = editdPipeline(pxDriver, pattGroups[i]);
                    editd(chStream, fsize);
                }
            }
        }
        else{
            const unsigned numOfThreads = Threads;
            SmallVector<pthread_t, 8> threads(numOfThreads);
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

    AlignedAllocator<char, 32> alloc;
    alloc.deallocate(chStream, 0);

    return 0;
}
