/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <kernel/core/idisa_target.h>
#include <boost/filesystem.hpp>
#include <re/cc/cc_compiler.h>
#include <re/cc/cc_compiler_target.h>
#include <kernel/core/kernel_builder.h>
#include <kernel/pipeline/pipeline_builder.h>
#include <kernel/basis/s2p_kernel.h>
#include <kernel/io/source_kernel.h>
#include <kernel/core/streamset.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/Module.h>
#include <llvm/Support/CommandLine.h>
#include <llvm/Support/raw_ostream.h>
#include <pablo/builder.hpp>
#include <pablo/pablo_compiler.h>
#include <pablo/pablo_kernel.h>
#include <pablo/pe_ones.h>
#include <toolchain/pablo_toolchain.h>
#include <kernel/pipeline/driver/cpudriver.h>
#include <toolchain/toolchain.h>
#include <fileselect/file_select.h>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <string>
#include <sys/stat.h>
#include <vector>

namespace fs = boost::filesystem;

using namespace llvm;
using namespace codegen;

static cl::OptionCategory matchpriorFlags("Command Flags", "matchprior options");

static cl::list<std::string> inputFiles(cl::Positional, cl::desc("<input file ...>"), cl::OneOrMore, cl::cat(matchpriorFlags));

static cl::opt<unsigned> priorDistance("distance", cl::desc("distance for prior match (default 2)"), cl::init(2),  cl::cat(matchpriorFlags));

static int defaultDisplayColumnWidth = 7;  // default field width

std::vector<fs::path> allFiles;

std::vector<uint64_t> priorCount;
std::vector<uint64_t> byteCount;

uint64_t TotalBytes = 0;
uint64_t TotalPrior = 0;

using namespace pablo;
using namespace kernel;

//  The callback routine that records counts in progress.
//
extern "C" {
    void record_prior_counts(uint64_t prior, uint64_t bytes, uint64_t fileIdx) {
        priorCount[fileIdx] = prior;
        byteCount[fileIdx] = bytes;
        TotalPrior += prior;
        TotalBytes += bytes;
    }
}

class MatchPriorKernel final: public pablo::PabloKernel {
public:
    MatchPriorKernel(const std::unique_ptr<kernel::KernelBuilder> & b, StreamSet * const countable);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;
};

MatchPriorKernel::MatchPriorKernel (const std::unique_ptr<kernel::KernelBuilder> & b, StreamSet * const countable)
: PabloKernel(b, "matchprior_" + std::to_string(priorDistance),
    {Binding{"countable", countable}},
    {},
    {},
    {Binding{b->getSizeTy(), "matchCount"}}) {
    addAttribute(SideEffecting());
}

void MatchPriorKernel::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    std::vector<PabloAST *> basis = getInputStreamSet("countable");

    PabloAST * matchesprior = pb.createOnes();
    for (unsigned i = 0; i < 7; i++) {
        matchesprior = pb.createAnd(matchesprior, pb.createXor(basis[i], pb.createAdvance(basis[i], priorDistance)),
                                    "matches_prior_" + std::to_string(priorDistance) + "_to_bit" + std::to_string(i));
    }
    Var * matchCount = getOutputScalarVar("matchCount");
    pb.createAssign(matchCount, pb.createCount(matchesprior));
}

typedef void (*MatchPriorFunctionType)(uint32_t fd, uint32_t fileIdx);

MatchPriorFunctionType mpPipelineGen(CPUDriver & pxDriver) {

    auto & iBuilder = pxDriver.getBuilder();

    Type * const int32Ty = iBuilder->getInt32Ty();

    auto P = pxDriver.makePipeline({Binding{int32Ty, "fd"}, Binding{int32Ty, "fileIdx"}});

    Scalar * const fileDescriptor = P->getInputScalar("fd");
    Scalar * const fileIdx = P->getInputScalar("fileIdx");

    StreamSet * const ByteStream = P->CreateStreamSet(1, 8);

    Kernel * mmapK = P->CreateKernelCall<MMapSourceKernel>(fileDescriptor, ByteStream);

    StreamSet * BasisBits = P->CreateStreamSet(8, 1);
    
    P->CreateKernelCall<S2PKernel>(ByteStream, BasisBits);

    Kernel * const mpk = P->CreateKernelCall<MatchPriorKernel>(BasisBits);

    Scalar * const matchCount = mpk->getOutputScalar("matchCount");
    Scalar * const fileSize = mmapK->getOutputScalar("fileItems");

    P->CreateCall("record_prior_counts", record_prior_counts, {matchCount, fileSize, fileIdx});

    return reinterpret_cast<MatchPriorFunctionType>(P->compile());
}

void mp(MatchPriorFunctionType fn_ptr, const uint32_t fileIdx) {
    std::string fileName = allFiles[fileIdx].string();
    struct stat sb;
    const int fd = open(fileName.c_str(), O_RDONLY);
    if (LLVM_UNLIKELY(fd == -1)) {
        if (errno == EACCES) {
            std::cerr << "matchprior: " << fileName << ": Permission denied.\n";
        }
        else if (errno == ENOENT) {
            std::cerr << "matchprior: " << fileName << ": No such file.\n";
        }
        else {
            std::cerr << "matchprior: " << fileName << ": Failed.\n";
        }
        return;
    }
    if (stat(fileName.c_str(), &sb) == 0 && S_ISDIR(sb.st_mode)) {
        std::cerr << "matchprior: " << fileName << ": Is a directory.\n";
        close(fd);
        return;
    }
    fn_ptr(fd, fileIdx);
    close(fd);
}

int main(int argc, char *argv[]) {
    codegen::ParseCommandLineOptions(argc, argv, {&matchpriorFlags, pablo_toolchain_flags(), codegen::codegen_flags()});
    if (argv::RecursiveFlag || argv::DereferenceRecursiveFlag) {
        argv::DirectoriesFlag = argv::Recurse;
    }
    CPUDriver pxDriver("mp");

    allFiles = argv::getFullFileList(pxDriver, inputFiles);

    const auto fileCount = allFiles.size();

    auto matchPriorFunctionPtr = mpPipelineGen(pxDriver);

    priorCount.resize(fileCount);
    byteCount.resize(fileCount);

    for (unsigned i = 0; i < fileCount; ++i) {
        mp(matchPriorFunctionPtr, i);
    }
    
    int displayColumnWidth = std::to_string(TotalBytes).size() + 1;
    if (displayColumnWidth < defaultDisplayColumnWidth) displayColumnWidth = defaultDisplayColumnWidth;

    for (unsigned i = 0; i < fileCount; ++i) {
        std::cout << std::setw(displayColumnWidth-1);
        std::cout << priorCount[i] << std::setw(displayColumnWidth);
        std::cout << byteCount[i];
        std::cout << " " << allFiles[i].string() << std::endl;
    }
    if (inputFiles.size() > 1) {
        std::cout << std::setw(displayColumnWidth-1);
        std::cout << TotalPrior << std::setw(displayColumnWidth);
        std::cout << TotalBytes;
        std::cout << " total" << std::endl;
    }

    return 0;
}
