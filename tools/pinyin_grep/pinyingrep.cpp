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

static cl::opt<std::string> KangXiLinePattern(cl::Positional, cl::desc("Pinyin Syllables"), cl::Required, cl::cat(pygrepFlags));

static cl::list<std::string> inputFiles(cl::Positional, cl::desc("<input file ...>"), cl::OneOrMore, cl::cat(pygrepFlags));

std::vector<fs::path> allFiles;
//step4
std::vector<re::RE*> generateREs(vector<string> KangXiLinePattern){
    std::vector<re::RE*> PinyinCC;
    re::RE* PinyinCC_final;
    for(int i= 0;i<KangXiLinePattern.size();i++)
    {
        re::RE * PinyinRe = re::RE_Parser::parse(KangXiLinePattern[i], 0U, argv::RegexpSyntax, false);
       //how to find this CC? 
        PinyinCC.push_back(re::makeCC());
    }
    return PinyinCC;
    //PinyinCC_final = re::makeSeq(PinyinCC.begin(),PinyinCC.end());
    //return PinyinCC_final;
}

int main(int argc, char* argv[]){
    PY::Buffer buf;
    AlignedAllocator <char, 32> alloc;
    char * UnihanBuf;
    
    codegen::ParseCommandLineOptions(argc, argv, {&pygrepFlags, codegen::codegen_flags()});
    if (argv::RecursiveFlag || argv::DereferenceRecursiveFlag) {
        argv::DirectoriesFlag = argv::Recurse;
    }
    
    CPUDriver pxDriver("pygrep");
    allFiles = argv::getFullFileList(pxDriver, inputFiles);
    const auto fileCount = allFiles.size();
    //step2
    vector <string> KangXiLinePattern;
    //string Search_Prefix = "kHanyuPinyin.*";
    KangXiLinePattern = PY::Before_Search(PinyinPattern);
    //here needs step3

    //step4
    auto KangXilineREs = generateREs(KangXiLinePattern);
    //step5 for each RE,use parabix internal search Engine to search
    
    // what is this driver it cannot be sepcified by the IDE
	grep::InternalSearchEngine engine(driver);
    //

    engine.setRecordBreak(grep::GrepRecordBreakKind::LF);
    //cannot using this KangXiLinePatter, must use RE int the Vector KangXiLineREs, 
    //do not kown what this function used to do
    engine.grepCodeGen(KangXiLinePattern);

    UnihanBuf = alloc.allocate(buf.R_size32(), 0);
    std::memcpy(UnihanBuf, buf.R_fstring().data(),buf.R_size());
    std::memset(UnihanBuf + buf.R_size(), 0, buf.R_diff());

    engine.doGrep(UnihanBuf, buf.R_size32(), accum);
    alloc.deallocate(UnihanBuf, 0);
}
