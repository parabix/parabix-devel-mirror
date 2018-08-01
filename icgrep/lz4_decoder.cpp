/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <llvm/Support/PrettyStackTrace.h>
#include <lz4/lz4_decompression_generator.h>

namespace re { class CC; }

using namespace llvm;

static cl::OptionCategory lz4dFlags("Command Flags", "lz4d options");
static cl::opt<std::string> inputFile(cl::Positional, cl::desc("<input file>"), cl::Required, cl::cat(lz4dFlags));
static cl::opt<std::string> outputFile(cl::Positional, cl::desc("<output file>"), cl::Required, cl::cat(lz4dFlags));
static cl::opt<bool> overwriteOutput("f", cl::desc("Overwrite existing output file."), cl::init(false), cl::cat(lz4dFlags));

int main(int argc, char *argv[]) {
    sys::PrintStackTraceOnErrorSignal(argv[0]);
    llvm::PrettyStackTraceProgram X(argc, argv);
    llvm_shutdown_obj shutdown;
    codegen::ParseCommandLineOptions(argc, argv, {&lz4dFlags, codegen::codegen_flags()});

    LZ4DecompressionGenerator g;
    return g.decompress(std::move(inputFile), std::move(outputFile), overwriteOutput);
}
