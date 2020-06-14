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
//#include <llvm/ADT/STLExtras.h> // for make_unique
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

//static cl::opt<bool> Database2("d2", cl::desc("Searching through kXHC1983 database"), cl::init(false), cl::cat(pygrepFlags))
std::vector<fs::path> allFiles;

typedef uint64_t (*UCountFunctionType)(uint32_t fd);


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
            /*for testing */
            cout << *iter3 <<endl;
            re::RE * PinyinRe = re::RE_Parser::parse(*iter3, 0U, re::PCRE, false);
            temp_Vec_RE.push_back(PinyinRe);
        }
        PinyinCC.push_back(temp_Vec_RE);
    }
    return PinyinCC;

}

UCountFunctionType pipelineGen(CPUDriver & pxDriver, re::RE * pinyinCC) {

    auto & B = pxDriver.getBuilder();

    auto P = pxDriver.makePipeline(
                {Binding{B->getInt32Ty(), "fileDescriptor"}},
                {Binding{B->getInt64Ty(), "countResult"}});

    Scalar * const fileDescriptor = P->getInputScalar("fileDescriptor");

    //  Create a stream set consisting of a single stream of 8-bit units (bytes).
    StreamSet * const ByteStream = P->CreateStreamSet(1, 8);

    //  Read the file into the ByteStream.
    P->CreateKernelCall<ReadSourceKernel>(fileDescriptor, ByteStream);

    //  Create a set of 8 parallel streams of 1-bit units (bits).
    StreamSet * const BasisBits = P->CreateStreamSet(8, 1);

    //  Transpose the ByteSteam into parallel bit stream form.
    P->CreateKernelCall<S2PKernel>(ByteStream, BasisBits);

    //  Create a character class bit stream.
    StreamSet * CCstream = P->CreateStreamSet(1, 1);
    
    std::map<std::string, StreamSet *> propertyStreamMap;
    auto nameString = "kHanyuPinyin:" + PinyinLinePattern;
    propertyStreamMap.emplace(nameString, CCstream);
    P->CreateKernelCall<UnicodePropertyKernelBuilder>(makeName(nameString, pinyinCC), BasisBits, CCstream);

    P->CreateKernelCall<PopcountKernel>(CCstream, P->getOutputScalar("countResult"));

    return reinterpret_cast<UCountFunctionType>(P->compile());
}



int main(int argc, char* argv[]){
    PinyinPattern::Buffer buf;
    AlignedAllocator <char, 32> alloc;
    char * UnihanBuf;
    
    codegen::ParseCommandLineOptions(argc, argv, {&pygrepFlags, codegen::codegen_flags()});
    if (argv::RecursiveFlag || argv::DereferenceRecursiveFlag) {
        argv::DirectoriesFlag = argv::Recurse;
    }
    
    CPUDriver pxDriver("pygrep");
    allFiles = argv::getFullFileList(pxDriver, inputFiles);
    const auto fileCount = allFiles.size();
    /*test point1*/
    cout << "Testing point1" << endl;
    /**/
    std::vector <std::vector<std::string> >KangXiLinePattern;
    
    KangXiLinePattern = PinyinPattern::Syllable_Parse(PinyinLinePattern,Database);
    if(KangXiLinePattern.empty())
    {
        return 0;
    }
    /*test point2*/
    cout << "Parsing finished successfully" << endl;
    /*parsing test*/
    UnihanBuf = alloc.allocate(buf.R_size32(), 0);
    std::memcpy(UnihanBuf, buf.R_fstring().data(),buf.R_size());
    std::memset(UnihanBuf + buf.R_size(), 0, buf.R_diff());
    /*test point3*/
    cout <<"memcpy finished successfully" << endl;
    /**/
    auto KangXilineREs = generateREs(KangXiLinePattern);
    
    std::vector <PinyinPattern::PinyinSetAccumulator> accum;
    std::vector <PinyinPattern::PinyinSetAccumulator> ::iterator accum_iter;
    std::vector <std::vector<re::RE*> > ::iterator RE_iter;
    accum_iter = accum.begin();
    std::vector <re::RE*> VectorRE,FinalRE;
    /*  for (auto word: KangXilineREs){
     auto PinyinRE = word;
     grep::InternalSearchEngine engine(pxDriver);
     engine.setRecordBreak(grep::GrepRecordBreakKind::LF);
     engine.grepCodeGen(PinyinRE);
     engine.doGrep(UnihanBuf, buf.R_size32(), *accum_iter);
     //resolveUnicodeNames(PinyinRE);
     } */
    for(RE_iter = KangXilineREs.begin();RE_iter != KangXilineREs.end();RE_iter++)
    {
        
        PinyinPattern::PinyinSetAccumulator Temp_accum;
        auto Syllable = *RE_iter;
        std::vector<re::RE *>::iterator tone_iter;
        //syllables search times test
        int times = 0;
        
        for(tone_iter = Syllable.begin();tone_iter != Syllable.end();tone_iter++)
        {
            grep::InternalSearchEngine engine(pxDriver);
            engine.setRecordBreak(grep::GrepRecordBreakKind::LF);
            engine.grepCodeGen(*tone_iter);
            engine.doGrep(UnihanBuf, buf.R_size32(), Temp_accum);
            times++;
        }
        //syllables search times test
        cout <<times<<endl;
        
        accum.push_back(Temp_accum);
    }

    /*testing point4*/
    cout <<"unicode generated successfully"<<endl;
    /**/
    alloc.deallocate(UnihanBuf, 0);
    
    for(accum_iter = accum.begin();accum_iter!=accum.end();accum_iter++)
    {
        auto temp_accum = *accum_iter;
        re::CC * CC_ast = re::makeCC(temp_accum.getAccumulatedSet());
        VectorRE.push_back(CC_ast);
    }
    
    FinalRE.push_back(re::makeSeq(VectorRE.begin(),VectorRE.end()));
    /*testing point5*/
    cout <<"RE make successfully"<<endl;
    /**/
    //   re::CC * CC_ast = re::makeCC(accum.getAccumulatedSet());
    //   VectorRE.push_back(CC_ast);
    //VectorRE = re::makeSeq(VectorRE.begin(),VectorRE.end());
    //   FinalRE.push_back(re::makeSeq(VectorRE.begin(),VectorRE.end()));
    //   UCountFunctionType uCountFunctionPtr = pipelineGen(pxDriver, CC_ast);


    std::unique_ptr<grep::GrepEngine> grep = make_unique<grep::EmitMatchesEngine>(pxDriver);
    if ((ColorFlag == alwaysColor) || ((ColorFlag == autoColor) && isatty(STDOUT_FILENO))) {
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
