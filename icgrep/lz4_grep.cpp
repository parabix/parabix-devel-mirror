/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */


#include <llvm/IR/Module.h>
#include <llvm/Linker/Linker.h>
#include <llvm/Support/PrettyStackTrace.h>
#include <llvm/Support/Signals.h>
#include <llvm/Support/ManagedStatic.h>
#include <IR_Gen/idisa_target.h>
#include <boost/filesystem.hpp>
#include <boost/iostreams/device/mapped_file.hpp>

#include <lz4FrameDecoder.h>
#include <cc/cc_compiler.h>
#include <toolchain/toolchain.h>
#include <kernels/cc_kernel.h>
#include <kernels/streamset.h>
#include <kernels/s2p_kernel.h>
#include <kernels/kernel_builder.h>
#include <toolchain/cpudriver.h>

#include <iostream>
#include <lz4/LZ4GrepGenerator.h>


#include <re/re_alt.h>
#include <re/re_start.h>
#include <re/re_end.h>
#include <re/re_utility.h>
#include <re/re_toolchain.h>
#include <pablo/pablo_toolchain.h>
#include <llvm/Support/raw_ostream.h>

namespace re { class CC; }

using namespace llvm;
using namespace parabix;
using namespace kernel;

static cl::OptionCategory lz4GrepFlags("Command Flags", "lz4d options");
static cl::opt<std::string> regexString(cl::Positional, cl::desc("<regex>"), cl::Required, cl::cat(lz4GrepFlags));
static cl::opt<std::string> inputFile(cl::Positional, cl::desc("<input file>"), cl::Required, cl::cat(lz4GrepFlags));
static cl::opt<bool> countOnly("count-only", cl::desc("Only count the match result"), cl::init(false), cl::cat(lz4GrepFlags));
static cl::opt<bool> enableMultiplexing("enable-multiplexing", cl::desc("Enable CC multiplexing."), cl::init(false), cl::cat(lz4GrepFlags));


//static cl::opt<std::string> outputFile(cl::Positional, cl::desc("<output file>"), cl::Required, cl::cat(lz4GrepFlags));
//static cl::opt<bool> overwriteOutput("f", cl::desc("Overwrite existing output file."), cl::init(false), cl::cat(lz4GrepFlags));


static cl::OptionCategory lz4GrepDebugFlags("LZ4 Grep Debug Flags", "lz4d debug options");
//static cl::opt<bool> extractOnly("extract-only", cl::desc("Only extract literal data to output file"), cl::init(false), cl::cat(lz4GrepDebugFlags));
//static cl::opt<bool> extractAndDepositOnly("extract-and-deposit-only", cl::desc("Only extract and deposit literal data to output file"), cl::init(false), cl::cat(lz4GrepDebugFlags));


int main(int argc, char *argv[]) {
    // This boilerplate provides convenient stack traces and clean LLVM exit
    // handling. It also initializes the built in support for convenient
    // command line option handling.
    sys::PrintStackTraceOnErrorSignal(argv[0]);
    llvm::PrettyStackTraceProgram X(argc, argv);
    llvm_shutdown_obj shutdown;
    codegen::ParseCommandLineOptions(argc, argv, {&lz4GrepFlags, &lz4GrepDebugFlags, codegen::codegen_flags()});
    std::string fileName = inputFile;
    LZ4FrameDecoder lz4Frame(fileName);
    if (!lz4Frame.isValid()) {
        errs() << "Invalid LZ4 file.\n";
        return -1;
    }

    boost::iostreams::mapped_file_source mappedFile;
    // Since mmap offset has to be multiples of pages, we can't use it to skip headers.
    mappedFile.open(fileName , lz4Frame.getBlocksLength() + lz4Frame.getBlocksStart());
    //char *fileBuffer = const_cast<char *>(mappedFile.data()) + lz4Frame.getBlocksStart();
    char *fileBuffer = const_cast<char *>(mappedFile.data());
    re::RE * re_ast = re::RE_Parser::parse(regexString, re::MULTILINE_MODE_FLAG);
    LZ4GrepGenerator g(enableMultiplexing);
    if (countOnly) {
        g.generateCountOnlyGrepPipeline(re_ast);
        auto main = g.getMainFunc();
        main(fileBuffer, lz4Frame.getBlocksStart(), lz4Frame.getBlocksStart() + lz4Frame.getBlocksLength(), lz4Frame.hasBlockChecksum());
    } else {
        g.generateScanMatchGrepPipeline(re_ast);
        g.invokeScanMatchGrep(fileBuffer, lz4Frame.getBlocksStart(), lz4Frame.getBlocksStart() + lz4Frame.getBlocksLength(), lz4Frame.hasBlockChecksum());

    }


    mappedFile.close();
    return 0;
}
