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

#include <lz4/lz4_frame_decoder.h>
#include <cc/cc_compiler.h>
#include <toolchain/toolchain.h>
#include <kernels/cc_kernel.h>
#include <kernels/streamset.h>
#include <kernels/s2p_kernel.h>
#include <kernels/kernel_builder.h>
#include <toolchain/cpudriver.h>

#include <iostream>
#include <lz4/grep/lz4_grep_base_generator.h>
#include <lz4/grep/lz4_grep_bitstream_generator.h>
#include <lz4/grep/lz4_grep_bytestream_generator.h>
#include <lz4/grep/lz4_grep_swizzle_generator.h>



#include <re/re_alt.h>
#include <re/re_start.h>
#include <re/re_end.h>
#include <re/re_utility.h>
#include <re/re_toolchain.h>
#include <pablo/pablo_toolchain.h>
#include <llvm/Support/raw_ostream.h>
#include <memory>

namespace re { class CC; }

using namespace llvm;
using namespace kernel;

static cl::OptionCategory lz4GrepFlags("Command Flags", "lz4d options");
static cl::opt<std::string> regexString(cl::Positional, cl::desc("<regex>"), cl::Required, cl::cat(lz4GrepFlags));
static cl::opt<std::string> inputFile(cl::Positional, cl::desc("<input file>"), cl::Required, cl::cat(lz4GrepFlags));
static cl::opt<bool> countOnly("count-only", cl::desc("Only count the match result"), cl::init(false), cl::cat(lz4GrepFlags));
static cl::opt<bool> enableMultiplexing("enable-multiplexing", cl::desc("Enable CC multiplexing."), cl::init(false), cl::cat(lz4GrepFlags));
static cl::opt<bool> utf8CC("utf8-CC", cl::desc("Use UTF-8 Character Class."), cl::init(false), cl::cat(lz4GrepFlags));

static cl::OptionCategory lz4GrepDebugFlags("LZ4 Grep Debug Flags", "lz4d debug options");
static cl::opt<bool> swizzledDecompression("swizzled-decompression", cl::desc("Use swizzle approach for decompression"), cl::init(false), cl::cat(lz4GrepDebugFlags));
static cl::opt<bool> bitStreamDecompression("bitstream-decompression", cl::desc("Use bit stream approach for decompression"), cl::init(false), cl::cat(lz4GrepDebugFlags));


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

    const auto mode = (countOnly ? LZ4GrepBaseGenerator::CountOnly : LZ4GrepBaseGenerator::Match);

    std::unique_ptr<LZ4GrepBaseGenerator> g;
    if (swizzledDecompression) {
        g.reset(new LZ4GrepSwizzleGenerator(mode));
    } else if (bitStreamDecompression) {
        g.reset(new LZ4GrepBitStreamGenerator(mode));
    } else {
        g.reset(new LZ4GrepByteStreamGenerator(mode));
    }

    if (countOnly) {
        g->generateCountOnlyGrepPipeline(re_ast, enableMultiplexing, utf8CC);
        auto main = g->getCountOnlyGrepMainFunction();
        uint64_t countResult = main(fileBuffer, lz4Frame.getBlocksStart(), lz4Frame.getBlocksStart() + lz4Frame.getBlocksLength(), lz4Frame.hasBlockChecksum());
        llvm::outs() << countResult << "\n";
    } else {
        g->generateScanMatchGrepPipeline(re_ast, enableMultiplexing, utf8CC);
        g->invokeScanMatchGrep(fileBuffer, lz4Frame.getBlocksStart(), lz4Frame.getBlocksStart() + lz4Frame.getBlocksLength(), lz4Frame.hasBlockChecksum());
    }

    mappedFile.close();

    return 0;
}
