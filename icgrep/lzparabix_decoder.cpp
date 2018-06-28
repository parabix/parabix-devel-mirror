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
#include <kernels/s2p_kernel.h>
#include <kernels/kernel_builder.h>
#include <toolchain/cpudriver.h>
#include <llvm/Support/raw_ostream.h>
#include <fstream>
#include <iostream>
#include <string>
#include <lzparabix/LZParabixGenerator.h>

namespace re { class CC; }

using namespace llvm;
using namespace parabix;
using namespace kernel;



static cl::OptionCategory decoderlags("Command Flags", "lzparabix decoder options");
static cl::opt<std::string> inputFile(cl::Positional, cl::desc("<input file>"), cl::Required, cl::cat(decoderlags));
static cl::opt<std::string> outputFile(cl::Positional, cl::desc("<output file>"), cl::Required, cl::cat(decoderlags));
static cl::opt<bool> overwriteOutput("f", cl::desc("Overwrite existing output file."), cl::init(false), cl::cat(decoderlags));


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
    codegen::ParseCommandLineOptions(argc, argv, {&decoderlags, codegen::codegen_flags()});
    std::string fileName = inputFile;

    if (boost::filesystem::exists(outputFile)) {
        if (overwriteOutput) {
            boost::filesystem::remove(outputFile);
        } else {
            llvm::errs() << outputFile + " existed. Use -f argument to overwrite.\n";
            return -1;
        }
    }

    boost::iostreams::mapped_file_source mappedFile;
    // Since mmap offset has to be multiples of pages, we can't use it to skip headers.
    size_t fileSize = getFileSize(fileName);
    mappedFile.open(fileName , fileSize);
    //char *fileBuffer = const_cast<char *>(mappedFile.data()) + lz4Frame.getBlocksStart();
    char *fileBuffer = const_cast<char *>(mappedFile.data());

    LZParabixGenerator g;
    g.generatePipeline(outputFile);
    auto main = g.getMainFunc();

    main(fileBuffer, 0, fileSize, false);
    mappedFile.close();
    return 0;
}
