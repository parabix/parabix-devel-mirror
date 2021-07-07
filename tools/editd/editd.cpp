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
#include <toolchain/pablo_toolchain.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/Module.h>
#include <llvm/Support/CommandLine.h>
#include <llvm/Support/raw_ostream.h>
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
#ifdef ENABLE_PAPI
#include <util/papi_helper.hpp>
// #define REPORT_PAPI_TESTS
#endif

#define USE_MULTI_EDITD_MERGE_GROUPS

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
#ifdef USE_MULTI_EDITD_MERGE_GROUPS
static cl::opt<unsigned> MergeGroupSize("merge-group-size", cl::desc("Number of editd kernels executed before merging results"), cl::init(4));
#endif

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

typedef void (*preprocessFunctionType)(char * output_data, size_t output_size, const uint32_t fd);

static char * chStream;
static size_t fsize;

class PreprocessKernel final: public pablo::PabloKernel {
public:
    PreprocessKernel(BuilderRef b, StreamSet * BasisBits, StreamSet * CCResults);
protected:
    void generatePabloMethod() override;
};

PreprocessKernel::PreprocessKernel(BuilderRef b, StreamSet * BasisBits, StreamSet * CCResults)
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
    const auto m = (4 * n) / 8;
    chStream = alloc.allocate(m);
    preprocess(chStream, 0, fd);
    close(fd);
    return chStream;
}


LLVM_READNONE std::string createName(const std::vector<std::string> & patterns) {
    std::string name;
    raw_string_ostream out(name);
    for(const auto & pat : patterns) {
        out << pat;
    }
    out << std::to_string(editDistance);
    out.flush();
    return name;
}

class PatternKernel final : public pablo::PabloKernel {
public:
    PatternKernel(BuilderRef b, const std::vector<std::string> & patterns, StreamSet * pat, StreamSet * E);
    StringRef getSignature() const override {
        return mSignature;
    }
    bool hasSignature() const override { return true; }
    bool hasFamilyName() const override { return true; }

protected:
    void generatePabloMethod() override;
private:
    const std::vector<std::string> & mPatterns;
    const std::string mSignature;
};

PatternKernel::PatternKernel(BuilderRef b, const std::vector<std::string> & patterns, StreamSet * pat, StreamSet * E)
: PabloKernel(b, "Editd_pattern_" + getStringHash(createName(patterns)),
{{"pat", pat}},
{{"E", E}})
, mPatterns(patterns)
, mSignature(createName(patterns)) {
    // addAttribute(InfrequentlyUsed());
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

void wrapped_report_pos(size_t match_pos, int dist) {
    struct matchPosition curMatch;
    curMatch.pos = match_pos;
    curMatch.dist = dist;
    matchList.push_back(curMatch);
    if (ShowPositions) {
        std::cout << "pos: " << match_pos << ", dist:" << dist << "\n";
    }
}

typedef void (*editdFunctionType)(char * byte_data, size_t filesize);

editdFunctionType editdPipeline(CPUDriver & pxDriver, const std::vector<std::string> & patterns) {
    auto & b = pxDriver.getBuilder();
    Type * const sizeTy = b->getSizeTy();
    Type * const inputType = b->getIntNTy(1)->getPointerTo();
    auto P = pxDriver.makePipeline({Binding{inputType, "input"}, Binding{sizeTy, "fileSize"}});
    Scalar * const inputStream = P->getInputScalar("input");
    Scalar * const fileSize = P->getInputScalar("fileSize");
    StreamSet * const ChStream = P->CreateStreamSet(4);
    P->CreateKernelCall<MemorySourceKernel>(inputStream, fileSize, ChStream);
    StreamSet * const MatchResults = P->CreateStreamSet(editDistance + 1);
    P->CreateKernelCall<PatternKernel>(patterns, ChStream, MatchResults);
    Kernel * const scan = P->CreateKernelCall<editdScanKernel>(MatchResults);
    scan->link("wrapped_report_pos", wrapped_report_pos);
    return reinterpret_cast<editdFunctionType>(P->compile());
}

typedef void (*multiEditdFunctionType)(const int fd);

multiEditdFunctionType multiEditdPipeline(CPUDriver & pxDriver) {

    auto & b = pxDriver.getBuilder();
    auto P = pxDriver.makePipeline({Binding{b->getInt32Ty(), "fileDescriptor"}});
    Scalar * const fileDescriptor = P->getInputScalar("fileDescriptor");

    StreamSet * const ByteStream = P->CreateStreamSet(1, 8);
    P->CreateKernelCall<MMapSourceKernel>(fileDescriptor, ByteStream);

//    std::vector<re::CC *> ccs;
//    ccs.emplace_back(re::makeCC(0x41, 0x61, &cc::Byte));
//    ccs.emplace_back(re::makeCC(0x43, 0x63, &cc::Byte));
//    ccs.emplace_back(re::makeCC(0x47, 0x67, &cc::Byte));
//    ccs.emplace_back(re::makeCC(0x54, 0x74, &cc::Byte));

//    StreamSet * const ChStream = P->CreateStreamSet(4);
//    P->CreateKernelCall<CharacterClassKernelBuilder>(ccs, ByteStream, ChStream);

    StreamSet * const BasisBits = P->CreateStreamSet(8);
    P->CreateKernelCall<S2PKernel>(ByteStream, BasisBits);

    StreamSet * const ChStream = P->CreateStreamSet(4);
    P->CreateKernelCall<PreprocessKernel>(BasisBits, ChStream);

    const auto n = pattGroups.size();
    std::vector<StreamSet *> MatchResults(n);
    for(unsigned i = 0; i < n; ++i){
        MatchResults[i] = P->CreateStreamSet(editDistance + 1);
        P->CreateKernelCall<PatternKernel>(pattGroups[i], ChStream, MatchResults[i]);
    }

    StreamSet * finalResults = MatchResults[0];
    if (n > 1) {
        #ifdef USE_MULTI_EDITD_MERGE_GROUPS
        const unsigned m = MergeGroupSize.getValue();
        if (m < 2) {
            report_fatal_error("merge-group-size cannot be less than 2");
        }
        std::vector<StreamSet *> mergeGroup;
        mergeGroup.reserve(m);
        assert (MatchResults[0]);
        mergeGroup.push_back(MatchResults[0]);
        for (unsigned i = 1; i < n; ++i) {
            if (mergeGroup.size() >= m) {
                StreamSet * const result = P->CreateStreamSet(editDistance + 1);
                P->CreateKernelCall<StreamsMerge>(mergeGroup, result);
                mergeGroup.clear();
                assert (result);
                mergeGroup.push_back(result);
            }
            assert (mergeGroup.size() < m);
            assert (MatchResults[i]);
            mergeGroup.push_back(MatchResults[i]);
        }
        assert (mergeGroup.size() > 1);
        finalResults = P->CreateStreamSet(editDistance + 1);
        assert (finalResults);
        P->CreateKernelCall<StreamsMerge>(mergeGroup, finalResults);
        #else
        finalResults = P->CreateStreamSet(editDistance + 1);
        P->CreateKernelCall<StreamsMerge>(MatchResults, finalResults);
        #endif
    }
    Kernel * const scan = P->CreateKernelCall<editdScanKernel>(finalResults);
    scan->link("wrapped_report_pos", wrapped_report_pos);
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
    StreamSet * const ChStream = P->CreateStreamSet(4);
    P->CreateKernelCall<MemorySourceKernel>(inputStream, fileSize, ChStream);
    StreamSet * const MatchResults = P->CreateStreamSet(editDistance + 1);
    P->CreateKernelCall<editdCPUKernel>(editDistance, patternLen, groupSize, pattStream, ChStream, MatchResults);
    Kernel * const scan = P->CreateKernelCall<editdScanKernel>(MatchResults);
    scan->link("wrapped_report_pos", wrapped_report_pos);
    return reinterpret_cast<editdIndexFunctionType>(P->compile());
}


int main(int argc, char *argv[]) {
    codegen::ParseCommandLineOptions(argc, argv);
    int pattern_segs = 0;
    int total_len = 0;

    get_editd_pattern(pattern_segs, total_len);


    CPUDriver pxDriver("editd");
    if (MultiEditdKernels) {

        auto editd = multiEditdPipeline(pxDriver);

        const auto & fileName = inputFiles[0];
        const int fd = open(inputFiles[0].c_str(), O_RDONLY);
        if (LLVM_UNLIKELY(fd == -1)) {
            std::cerr << "Error: cannot open " << fileName << " for processing. Skipped.\n";
            exit(-1);
        }
        #ifdef REPORT_PAPI_TESTS
        papi::PapiCounter<4> jitExecution{{PAPI_L3_TCM, PAPI_L3_TCA, PAPI_TOT_INS, PAPI_TOT_CYC}};
        jitExecution.start();
        #endif
        editd(fd);
        #ifdef REPORT_PAPI_TESTS
        jitExecution.stop();
        jitExecution.write(std::cerr);
        #endif
        close(fd);
        run_second_filter(pattern_segs, total_len, 0.15);
        return 0;
    }

    auto preprocess_ptr = preprocessPipeline(pxDriver);
    preprocess(preprocess_ptr);

    if (pattVector.size() == 1) {
        auto editd = editdPipeline(pxDriver, pattVector);
        editd(chStream, fsize);
        std::cout << "total matches is " << matchList.size() << std::endl;
    } else if (EditdIndexPatternKernels) {
        auto editd_ptr = editdIndexPatternPipeline(pxDriver, pattVector[0].length());
        for(unsigned i=0; i< pattVector.size(); i+=groupSize){
            std::string pattern = "";
            for (int j=0; j<groupSize; j++){
                pattern += pattVector[i+j];
            }
            editd_ptr(chStream, fsize, pattern.c_str());
        }
    }
    else {
        for(unsigned i=0; i< pattGroups.size(); i++){
            auto editd = editdPipeline(pxDriver, pattGroups[i]);
            editd(chStream, fsize);
        }
    }

    run_second_filter(pattern_segs, total_len, 0.15);

    AlignedAllocator<char, 32> alloc;
    alloc.deallocate(chStream, 0);

    return 0;
}
