#include <cstdio>
#include <vector>
#include <llvm/ADT/STLExtras.h>
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
#include <map>
#include <toolchain/toolchain.h>
#include <toolchain/pablo_toolchain.h>
#include <boost/filesystem.hpp>
#include <fileselect/file_select.h>
#include <sys/stat.h>
#include <fcntl.h>
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
#include "../icgrep/grep_interface.h"
#include "pinyin.h"

namespace fs = boost::filesystem;

using namespace llvm;
using namespace codegen;
using namespace kernel;
using namespace std;
using namespace pablo;
static cl::OptionCategory pygrepFlags("Command Flags", "pinyingrep options");

static cl::opt<std::string> PinyinLinePattern(cl::Positional, cl::desc("Pinyin Syllables"), cl::Required, cl::cat(pygrepFlags));

static cl::list<std::string> inputFiles(cl::Positional, cl::desc("<input file ...>"), cl::OneOrMore, cl::cat(pygrepFlags));

//the default one is kHanyuPinyin
static cl::opt<bool> Database("data", cl::desc("Searching by kXHC1983 database instead of kHanyuPinyin database"), cl::init(false), cl::cat(pygrepFlags));

ColoringType ColorFlag;
//options for colourization; (e.g. -c auto)
static cl::opt<ColoringType, true> Color("c", cl::desc("Set the colorization of the output."),
                                         cl::values(clEnumValN(alwaysColor, "always", "Turn on colorization when outputting to a file and terminal"),
                                                    clEnumValN(autoColor,   "auto", "Turn on colorization only when outputting to terminal"),
                                                    clEnumValN(neverColor,  "never", "Turn off output colorization")
                                                    CL_ENUM_VAL_SENTINEL), cl::cat(pygrepFlags), cl::location(ColorFlag), cl::init(neverColor));
static cl::opt<bool> LineNumberOption("n", cl::desc("Show the line number with each matching line."), cl::init(false), cl::cat(pygrepFlags));
static cl::opt<bool> WithFilenameOption("h", cl::desc("Show the file name with each matching line."), cl::init(false), cl::cat(pygrepFlags));
static cl::opt<bool> IndexSearchingOption("i", cl::desc("using hexidemical unicode index for searching"), cl::init(false), cl::cat(pygrepFlags));

std::vector<fs::path> allFiles;

typedef uint64_t (*UCountFunctionType)(uint32_t fd);

/*the function to generate REs for Search Engine to use*/
std::vector <std::vector<re::RE*> >generateREs(std::vector<std::vector<std::string> >KangXiLinePattern){
    std::vector<std::vector<re::RE*> >PinyinCC;
    
    std::vector<std::vector<std::string> >:: iterator iter1;
    for(iter1 = KangXiLinePattern.begin();iter1!=KangXiLinePattern.end();iter1++)
    {
        std::vector<re::RE*> temp_Vec_RE;
        std::vector<std::string> temp_Vec_String;
        std::vector<re::RE*>::iterator iter2;
        std::vector<std::string>::iterator iter3;
        temp_Vec_String = *iter1;
        for(iter3 = temp_Vec_String.begin();iter3 != temp_Vec_String.end();iter3++)
        {
            re::RE * PinyinRe = re::RE_Parser::parse(*iter3, 0U, re::PCRE, false);
            temp_Vec_RE.push_back(PinyinRe);
        }
        PinyinCC.push_back(temp_Vec_RE);
    }
    return PinyinCC;
    
}



int main(int argc, char* argv[]){
    
    AlignedAllocator <char, 32> alloc;
    char * UnihanBuf;
    PinyinPattern::Buffer buf(false);
    codegen::ParseCommandLineOptions(argc, argv, {&pygrepFlags, codegen::codegen_flags()});
    if (argv::RecursiveFlag || argv::DereferenceRecursiveFlag) {
        argv::DirectoriesFlag = argv::Recurse;
    }
    
    CPUDriver pxDriver("pygrep");
    allFiles = argv::getFullFileList(pxDriver, inputFiles);
    
    std::vector <re::RE*> VectorRE,FinalRE;
    
    if(!IndexSearchingOption)
    {
        std::vector <std::vector<std::string> >KangXiLinePattern;
        /*parse the input syllables*/
        KangXiLinePattern = PinyinPattern::Syllable_Parse(PinyinLinePattern,Database);
        /*if there is no syllbales or no legal syllables return directly*/
        if(KangXiLinePattern.empty())
        {
            return 0;
        }
        
        /*copy the context of Unihan_reading.txt into the UnihanBuf for Search*/
        UnihanBuf = alloc.allocate(buf.R_size32(), 0);
        std::memcpy(UnihanBuf, buf.R_fstring().data(),buf.R_size());
        std::memset(UnihanBuf + buf.R_size(), 0, buf.R_diff());
        
        auto KangXilineREs = generateREs(KangXiLinePattern);
        
        std::vector <PinyinPattern::PinyinSetAccumulator> accum;
        std::vector <std::vector<re::RE*> > ::iterator RE_iter;
        std::vector <PinyinPattern::PinyinSetAccumulator>::iterator accum_iter;

        /*The search Engine first Search through the Unihan_Reading.txt and find the results store in the accum*/
        for(RE_iter = KangXilineREs.begin();RE_iter != KangXilineREs.end();RE_iter++)
        {
            
            PinyinPattern::PinyinSetAccumulator Temp_accum;
            auto Syllable = *RE_iter;
            std::vector<re::RE *>::iterator tone_iter;
            for(tone_iter = Syllable.begin();tone_iter != Syllable.end();tone_iter++)
            {
                grep::InternalSearchEngine engine(pxDriver);
                engine.setRecordBreak(grep::GrepRecordBreakKind::LF);
                engine.grepCodeGen(*tone_iter);
                engine.doGrep(UnihanBuf, buf.R_size32(), Temp_accum);
                
            }
            accum.push_back(Temp_accum);
        }
        
        alloc.deallocate(UnihanBuf, 0);
        
        /*using the unicode store in the accum to make another RE for the final search*/
        for(accum_iter = accum.begin();accum_iter!=accum.end();accum_iter++)
        {
            auto temp_accum = *accum_iter;
            re::CC * CC_ast = re::makeCC(temp_accum.getAccumulatedSet());
            VectorRE.push_back(CC_ast);
        }
        /*if the syllable is a sequence then make the RE a sequence*/
        FinalRE.push_back(re::makeSeq(VectorRE.begin(),VectorRE.end()));
        
    }
    else{
        stringstream s;
        unsigned int CodePoint;
        s << hex << PinyinLinePattern;
        s >> CodePoint;
        re::CC * CC_ast = re::makeCC(CodePoint);
        FinalRE.push_back(CC_ast);
    }
    
    /*the final search part*/
    std::unique_ptr<grep::GrepEngine> grep = make_unique<grep::EmitMatchesEngine>(pxDriver);
    if ((ColorFlag == alwaysColor) || ((ColorFlag == autoColor) && isatty(STDOUT_FILENO)))
    {
        grep->setColoring();
    }
    if (WithFilenameOption) grep->showFileNames();
    if (LineNumberOption) grep->showLineNumbers();
    
    grep->initFileResult(allFiles);
    grep->initREs(FinalRE);
    grep->grepCodeGen();
    const bool matchFound = grep->searchAllFiles();
    
    return matchFound ? argv::MatchFoundExitCode : argv::MatchNotFoundExitCode;
}
