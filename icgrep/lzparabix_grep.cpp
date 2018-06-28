//
// Created by wxy325 on 2018/6/19.
//


#include <llvm/IR/Module.h>
#include <llvm/Linker/Linker.h>
#include <llvm/Support/PrettyStackTrace.h>
#include <llvm/Support/Signals.h>
#include <llvm/Support/ManagedStatic.h>
#include <IR_Gen/idisa_target.h>
#include <boost/filesystem.hpp>
#include <boost/iostreams/device/mapped_file.hpp>

#include <cc/cc_compiler.h>
#include <toolchain/toolchain.h>
#include <kernels/cc_kernel.h>
#include <kernels/streamset.h>
#include <kernels/kernel_builder.h>
#include <toolchain/cpudriver.h>
#include <fstream>
#include <iostream>


#include <re/re_alt.h>
#include <re/re_start.h>
#include <re/re_end.h>
#include <re/re_utility.h>
#include <re/re_toolchain.h>
#include <pablo/pablo_toolchain.h>
#include <llvm/Support/raw_ostream.h>
#include <lzparabix/LZParabixGrepGenerator.h>


namespace re { class CC; }

using namespace llvm;
using namespace parabix;
using namespace kernel;

static cl::OptionCategory lz4GrepFlags("Command Flags", "lz4d options");
static cl::opt<std::string> regexString(cl::Positional, cl::desc("<regex>"), cl::Required, cl::cat(lz4GrepFlags));
static cl::opt<std::string> inputFile(cl::Positional, cl::desc("<input file>"), cl::Required, cl::cat(lz4GrepFlags));
static cl::opt<bool> countOnly("count-only", cl::desc("Only count the match result"), cl::init(false), cl::cat(lz4GrepFlags));
static cl::opt<bool> enableMultiplexing("enable-multiplexing", cl::desc("Enable CC multiplexing."), cl::init(false), cl::cat(lz4GrepFlags));

static cl::OptionCategory lz4GrepDebugFlags("LZ4 Grep Debug Flags", "lz4d debug options");


size_t getFileSize(std::string filename) {
    std::ifstream f(filename, std::ios::binary | std::ios::ate);
    if (f.fail()) {
        return 0;
    }
    size_t fileSize = f.tellg();
    f.close();
    return fileSize;
}


int main(int argc, char *argv[]) {
    // This boilerplate provides convenient stack traces and clean LLVM exit
    // handling. It also initializes the built in support for convenient
    // command line option handling.
    sys::PrintStackTraceOnErrorSignal(argv[0]);
    llvm::PrettyStackTraceProgram X(argc, argv);
    llvm_shutdown_obj shutdown;
    codegen::ParseCommandLineOptions(argc, argv, {&lz4GrepFlags, &lz4GrepDebugFlags, codegen::codegen_flags()});
    std::string fileName = inputFile;

    boost::iostreams::mapped_file_source mappedFile;
    // Since mmap offset has to be multiples of pages, we can't use it to skip headers.
    size_t fileSize = getFileSize(fileName);
    mappedFile.open(fileName , fileSize);
    //char *fileBuffer = const_cast<char *>(mappedFile.data()) + lz4Frame.getBlocksStart();
    char *fileBuffer = const_cast<char *>(mappedFile.data());
    re::RE * re_ast = re::RE_Parser::parse(regexString, re::MULTILINE_MODE_FLAG);

    LZParabixGrepGenerator g(enableMultiplexing);

    g.generateCountOnlyAioPipeline(re_ast);
    auto main = g.getCountOnlyGrepMainFunction();
    uint64_t countResult = main(fileBuffer, 0, fileSize, false);
    llvm::outs() << countResult << "\n";


    mappedFile.close();
    return 0;
}
