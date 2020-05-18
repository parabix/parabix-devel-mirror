#include <cstdio>
#include <vector>
#include <llvm/Support/CommandLine.h>
#include <llvm/Support/ErrorHandling.h>
#include <llvm/Support/PrettyStackTrace.h>
#include <llvm/Support/Signals.h>
#include <llvm/Support/ManagedStatic.h>
#include <llvm/Support/raw_ostream.h>
#include <pablo/pablo_kernel.h>
#include <pablo/builder.hpp>
#include <pablo/pe_zeroes.h>
#include <re/parse/parser.h>
#include <re/toolchain/toolchain.h>
#include <re/cc/cc_compiler.h>
#include <re/cc/cc_compiler_target.h>
#include <re/adt/adt.h>
#include <re/unicode/re_name_resolve.h>
#include <re/cc/cc_kernel.h>
#include <re/ucd/ucd_compiler.hpp>
#include <grep/grep_engine.h>
#include <grep/grep_kernel.h>
#include <fstream>
#include <string>
#include <toolchain/toolchain.h>
#include <toolchain/pablo_toolchain.h>
#include <boost/filesystem.hpp>
#include <fileselect/file_select.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <llvm/ADT/STLExtras.h> // for make_unique
#include <kernel/pipeline/driver/cpudriver.h>
#include <kernel/core/kernel_builder.h>
#include <kernel/pipeline/pipeline_builder.h>
#include <kernel/basis/s2p_kernel.h>
#include <kernel/io/source_kernel.h>
#include <kernel/core/streamset.h>
#include <kernel/unicode/UCD_property_kernel.h>
#include <kernel/core/idisa_target.h>
#include <boost/filesystem.hpp>
#include <util/aligned_allocator.h>
#include "pinyin.h"

namespace fs = boost::filesystem;

using namespace llvm;
using namespace codegen;
using namespace kernel;

static cl::OptionCategory pygrepFlags("Command Flags", "pinyingrep options");

static cl::opt<std::string> KangXiLinePattern(cl::Positional, cl::desc("<Unicode for pinyin>"), cl::Required, cl::cat(pygrepFlags));

static cl::list<std::string> inputFiles(cl::Positional, cl::desc("<input file ...>"), cl::OneOrMore, cl::cat(pygrepFlags));

std::vector<re::RE*> generateREs(std::string KangXiLinePattern){
    PinyinPattern::Pattern_Parse parser;
    parser.parse(KangXiLinePattern);
    
    PinyinPattern::Pattern_Enumerate enumerator;
    enumerator.enumerate(parser);

    return enumerator.createREs();
}


int main(int argc, char* argv[]){
    codegen::ParseCommandLineOptions(argc, argv, {&pygrepFlags, codegen::codegen_flags()});
    if (argv::RecursiveFlag || argv::DereferenceRecursiveFlag) {
        argv::DirectoriesFlag = argv::Recurse;
    }
    CPUDriver pxDriver("pinyingrep");
    allFiles = argv::getFullFileList(pxDriver, inputFiles);
    const auto fileCount = allFiles.size();

    auto KangXiLineREs = generateREs(KangXiLinePattern);
	grep::InternalSearchEngine engine(driver);
    engine.setRecordBreak(grep::GrepRecordBreakKind::LF);
    engine.grepCodeGen(KangXiLinePattern);
    engine.doGrep(UnihanBuf, BufSize, accum);
}
