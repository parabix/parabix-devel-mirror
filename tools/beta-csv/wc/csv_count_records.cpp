#include <kernel/core/idisa_target.h>
#include <kernel/core/kernel_builder.h>
#include <kernel/core/streamset.h>
#include <kernel/pipeline/pipeline_builder.h>
#include <kernel/pipeline/driver/cpudriver.h>
#include <kernel/streamutils/deletion.h>
#include <kernel/streamutils/pdep_kernel.h>
#include <kernel/streamutils/stream_select.h>
#include <kernel/streamutils/stream_shift.h>
#include <kernel/unicode/UCD_property_kernel.h>
#include <kernel/basis/s2p_kernel.h>
#include <kernel/basis/p2s_kernel.h>
#include <kernel/io/source_kernel.h>
#include <kernel/io/stdout_kernel.h>
#include <kernel/scan/scanmatchgen.h>
#include <boost/filesystem.hpp>
#include <re/cc/cc_compiler.h>
#include <re/cc/cc_compiler_target.h>
#include <re/adt/adt.h>
#include <re/parse/parser.h>
#include <re/unicode/re_name_resolve.h>
#include <re/cc/cc_kernel.h>
#include <re/ucd/ucd_compiler.hpp>
#include <llvm/IR/Function.h>
#include <llvm/IR/Module.h>
#include <llvm/Support/CommandLine.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/Support/ErrorHandling.h>
#include <pablo/pablo_kernel.h>
#include <pablo/builder.hpp>
#include <pablo/pe_zeroes.h>
#include <pablo/pe_ones.h>
#include <pablo/bixnum/bixnum.h>
#include <toolchain/pablo_toolchain.h>
#include <grep/grep_kernel.h>
#include <toolchain/toolchain.h>
#include <fileselect/file_select.h>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <string>
#include <sys/stat.h>
#include <vector>
#include <map>
#include <cstdio>

//#include <re/adt/re_re.h>
//#include <re/adt/re_name.h>
//unused in adapted version of code




namespace fs = boost::filesystem;

using namespace llvm;
using namespace codegen;
using namespace kernel;



using namespace kernel;
using namespace llvm;
using namespace pablo;

//  Given a Unicode character class (set of Unicode characters), ucount
//  counts the number of occurrences of characters in that class within
//  given input files.

static cl::OptionCategory ucFlags("Command Flags", "ucount options");

//modified: taken out cl::opt

//  Multiple input files are allowed on the command line; counts are produced
//  for each file.
static cl::list<std::string> inputFiles(cl::Positional, cl::desc("<input file ...>"), cl::CommaSeparated, cl::cat(ucFlags));//modified cl::OneOrMore to CommaSeparated

std::vector<fs::path> allFiles;


//
//  The Hexify Kernel is the logic that produces hexadecimal output
//  from a source bit stream set called spreadBasis and the insertMask
//  used to spread out the bits.
//
class CSV_Quote_Translate : public PabloKernel {
public:

    CSV_Quote_Translate(BuilderRef kb, StreamSet * dquote, StreamSet * basis, StreamSet * translatedBasis,StreamSet * target)///modified added StreamSet * target
        : PabloKernel(kb, "CSV_Quote_Translate",
                      {Binding{"dquote", dquote, FixedRate(), LookAhead(1)}, Binding{"basis", basis},Binding{"target",target}},///modified added Binding{"target",target}
                      {Binding{"translatedBasis", translatedBasis}}) {}
protected:
    void generatePabloMethod() override;
};

void CSV_Quote_Translate::generatePabloMethod() {
    pablo::PabloBuilder pb(getEntryScope());
    std::vector<PabloAST *> basis = getInputStreamSet("basis");
    PabloAST * dquote = getInputStreamSet("dquote")[0];
    PabloAST * dquote_odd = pb.createEveryNth(dquote, pb.getInteger(2));
    PabloAST * dquote_even = pb.createXor(dquote, dquote_odd);
    PabloAST * quote_escape = pb.createAnd(dquote_even, pb.createLookahead(dquote, 1));
    ///////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////
    ////modified///////////////////////////////////////////////
    PabloAST * target = getInputStreamSet("target")[0];
    PabloAST * escaped_quote = pb.createAdvance(quote_escape, 1);
    PabloAST * start_dquote = pb.createXor(dquote_odd, escaped_quote);
    PabloAST * end_dquote = pb.createXor(dquote_even, quote_escape);
    PabloAST * literal_target = pb.createAnd(target,pb.createIntrinsicCall(pablo::Intrinsic::InclusiveSpan, {start_dquote, end_dquote}));
    // Translate \r to \n    ASCII value of \r = 0x0d, ASCII value of \n = 0x0a
    std::vector<PabloAST *> translated_basis(8, nullptr);
    translated_basis[0] = basis[0];  // no changes
    translated_basis[1] = basis[1];  // no changes
    translated_basis[2] = basis[2];  // no changes
    translated_basis[3] = basis[3];  // no changes
    translated_basis[4] = basis[4];  // no changes
    translated_basis[5] = pb.createXor(basis[5], literal_target);  // flip
    translated_basis[6] = pb.createXor(basis[6], literal_target);  // flip
    translated_basis[7] = pb.createXor(basis[7], literal_target);  // flip
     ///////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////

    Var * translatedVar = getOutputStreamVar("translatedBasis");
    for (unsigned i = 0; i < 8; i++) {
        pb.createAssign(pb.createExtract(translatedVar, pb.getInteger(i)), translated_basis[i]);
    }
}



typedef uint64_t (*CSVCountFunctionType)(uint32_t fd);

//
//  This is the function that generates and compiles a Parabix pipeline to
//  perform the character counting task on a single input file.   The program
//  takes a re::Name object whose definition includes the UnicodeSet defining
//  the character class.    The compiled pipeline program is returned.
//
//  The compiled pipeline may then be executed.   When executed, it must be given
//  an integer "file descriptor" as its input, and will produce the count of
//  the number of characters of the given character class as a result.
//
//CSVCountFunctionType pipelineGen(CPUDriver & pxDriver, re::Name * CC_name) {
CSVCountFunctionType pipelineGen(CPUDriver & pxDriver) {
    auto & B = pxDriver.getBuilder();

    auto P = pxDriver.makePipeline(
                {Binding{B->getInt32Ty(), "fileDescriptor"}},
                {Binding{B->getInt64Ty(), "countResult"}});

    Scalar *  fileDescriptor = P->getInputScalar("fileDescriptor");

    //  Create a stream set consisting of a single stream of 8-bit units (bytes).
    StreamSet *  ByteStream = P->CreateStreamSet(1, 8);

    //  Read the file into the ByteStream.
    P->CreateKernelCall<ReadSourceKernel>(fileDescriptor, ByteStream);

    //  Create a set of 8 parallel streams of 1-bit units (bits).
    StreamSet *  BasisBits = P->CreateStreamSet(8, 1);

    //  Transpose the ByteSteam into parallel bit stream form.
    P->CreateKernelCall<S2PKernel>(ByteStream, BasisBits);


    

    ///////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////
    ////modified///////////////////////////////////////////////
    //  Create a character class bit stream.
    StreamSet * CCstream = P->CreateStreamSet(1, 1);

    StreamSet * Dquote = P->CreateStreamSet(1);
    P->CreateKernelCall<CharacterClassKernelBuilder>(std::vector<re::CC *>{re::makeByte(0x22)}, BasisBits, Dquote);

    std::map<std::string, StreamSet *>::iterator it;
    StreamSet * translatedBasis = P->CreateStreamSet(8,1);

    StreamSet * newlines = P->CreateStreamSet(1);
    P->CreateKernelCall<CharacterClassKernelBuilder>(std::vector<re::CC *>{re::makeByte(0x0d)}, BasisBits, newlines);
    P->CreateKernelCall<CSV_Quote_Translate>(Dquote, BasisBits, translatedBasis,newlines);

    //StreamSet * translated = P->CreateStreamSet(1, 8);                            //<-  uncomment to see translated stream
    //P->CreateKernelCall<P2SKernel>(translatedBasis, translated);                  //<-  uncomment to see translated stream
    //P->CreateKernelCall<StdOutKernel>(translated);                                //<-  uncomment to see translated stream

    ///////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////
    P->CreateKernelCall<CharacterClassKernelBuilder>(std::vector<re::CC *>{re::makeByte(0x0d)}, translatedBasis, CCstream);     //checks for '\r'
    //P->CreateKernelCall<CharacterClassKernelBuilder>(std::vector<re::CC *>{re::makeByte(0x0a)}, translatedBasis, CCstream);   //checks for '\n'
    //for the two lines of code above, make valid one of them to check either '\r' or '\n'
    P->CreateKernelCall<PopcountKernel>(CCstream, P->getOutputScalar("countResult"));

    return reinterpret_cast<CSVCountFunctionType>(P->compile());
}
//
//  Given a compiled pipeline program for counting  the characters of a class,
//  as well as an index into the global vector of inputFiles,  open the
//  given file and execute the compiled program to produce the count result.
uint64_t ucount1(CSVCountFunctionType fn_ptr, const uint32_t fileIdx) {
    std::string fileName = allFiles[fileIdx].string();
    struct stat sb;
    const int fd = open(fileName.c_str(), O_RDONLY);
    if (LLVM_UNLIKELY(fd == -1)) {
        if (errno == EACCES) {
            std::cerr << "ucount: " << fileName << ": Permission denied.\n";
        }
        else if (errno == ENOENT) {
            std::cerr << "ucount: " << fileName << ": No such file.\n";
        }
        else {
            std::cerr << "ucount: " << fileName << ": Failed.\n";
        }
        return 0;
    }
    if (stat(fileName.c_str(), &sb) == 0 && S_ISDIR(sb.st_mode)) {
        std::cerr << "ucount: " << fileName << ": Is a directory.\n";
        close(fd);
        return 0;
    }
    uint64_t theCount = fn_ptr(fd);
    close(fd);
    return theCount;
}

int main(int argc, char *argv[]) {
    codegen::ParseCommandLineOptions(argc, argv, {&ucFlags, codegen::codegen_flags()});
    /*
    modified: taken out CC_expr
    if (argv::RecursiveFlag || argv::DereferenceRecursiveFlag) {
        argv::DirectoriesFlag = argv::Recurse;
    }*/
    CPUDriver pxDriver("wc");

    allFiles = argv::getFullFileList(pxDriver, inputFiles);
    const auto fileCount = allFiles.size();

    CSVCountFunctionType CSVCountFunctionPtr = nullptr;
    /*
    modified: taken out CC_expr
    re::RE * CC_re = re::RE_Parser::parse(CC_expr);
    resolveUnicodeNames(CC_re);
    if (re::Name * UCD_property_name = dyn_cast<re::Name>(CC_re)) {
        CSVCountFunctionPtr = pipelineGen(pxDriver, UCD_property_name);
    } else if (re::CC * CC_ast = dyn_cast<re::CC>(CC_re)) {
        CSVCountFunctionPtr = pipelineGen(pxDriver, makeName(CC_ast));
    } else {
        std::cerr << "Input expression must be a Unicode property or CC but found: " << CC_expr << " instead.\n";
        exit(1);
    }
    */
    CSVCountFunctionPtr=pipelineGen(pxDriver);
    std::vector<uint64_t> theCounts;
    
    theCounts.resize(fileCount);
    uint64_t totalCount = 0;

    for (unsigned i = 0; i < fileCount; ++i) {
        theCounts[i] = ucount1(CSVCountFunctionPtr, i);
        totalCount += theCounts[i];
    }
    
    const int defaultDisplayColumnWidth = 7;
    int displayColumnWidth = std::to_string(totalCount).size() + 1;
    if (displayColumnWidth < defaultDisplayColumnWidth) displayColumnWidth = defaultDisplayColumnWidth;

    for (unsigned i = 0; i < fileCount; ++i) {
        std::cout << std::setw(displayColumnWidth);
        std::cout << theCounts[i] << std::setw(displayColumnWidth);
        std::cout << " " << allFiles[i].string() << std::endl;
    }
    if (inputFiles.size() > 1) {
        std::cout << std::setw(displayColumnWidth-1);
        std::cout << totalCount;
        std::cout << " total" << std::endl;
    }

    return 0;
}
