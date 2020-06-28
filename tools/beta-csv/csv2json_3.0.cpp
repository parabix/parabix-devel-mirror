/*
 *  Copyright (c) 2020 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */


#include <cstdio>
#include <vector>
#include <llvm/Support/CommandLine.h>
#include <llvm/Support/ErrorHandling.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/IR/Module.h>
#include <re/adt/re_name.h>
#include <re/adt/re_re.h>
#include <kernel/core/kernel_builder.h>
#include <kernel/pipeline/pipeline_builder.h>
#include <kernel/streamutils/deletion.h>
#include <kernel/streamutils/pdep_kernel.h>
#include <kernel/streamutils/stream_select.h>
#include <kernel/streamutils/stream_shift.h>
#include <kernel/basis/s2p_kernel.h>
#include <kernel/basis/p2s_kernel.h>
#include <kernel/io/source_kernel.h>
#include <kernel/io/stdout_kernel.h>
#include <kernel/scan/scanmatchgen.h>
#include <re/adt/re_name.h>
#include <re/cc/cc_kernel.h>
#include <re/cc/cc_compiler.h>
#include <re/cc/cc_compiler_target.h>
#include <string>
#include <toolchain/toolchain.h>
#include <toolchain/pablo_toolchain.h>
#include <pablo/pablo_intrinsic.h>
#include <pablo/builder.hpp>
#include <pablo/pe_ones.h>
#include <pablo/pe_zeroes.h>
#include <pablo/bixnum/bixnum.h>
#include <fcntl.h>
#include <iostream>
#include <kernel/pipeline/driver/cpudriver.h>
#include <kernel/util/debug_display.h>
#include <fstream>
#include <sstream>
#include <math.h>
#include <kernel/streamutils/sentinel.h>
#include <kernel/streamutils/string_insert.h>
#include <kernel/streamutils/run_index.h>
#include <pablo/parse/pablo_source_kernel.h> 
#include <boost/filesystem.hpp>
#include <fileselect/file_select.h>
#include<sys/stat.h>
#include<stdlib.h>
#include<dirent.h>
#include <re/cc/cc_compiler.h>
#include <re/cc/cc_compiler_target.h>
#include <re/adt/adt.h>
#include <re/parse/parser.h>
#include <re/unicode/re_name_resolve.h>
#include <re/cc/cc_kernel.h>
#include <re/ucd/ucd_compiler.hpp>
#include <kernel/unicode/UCD_property_kernel.h>
using namespace kernel;
using namespace llvm;
using namespace pablo;
using namespace std;
namespace fs = boost::filesystem;

int Get_Field_Count(const char delimiter, const char* argv);
void Get_Header(const char delimiter, const char* dir, vector<string>& v, int n);
int getAllFiles(std::vector<fs::path> &dirs, std::vector<string> &allFiles);
void setHeader(cl::list<std::string> &userHeader, vector<string> &header);
//  The declarations below are for command line processing.
//  See the LLVM CommandLine 2.0 Library Manual https://llvm.org/docs/CommandLine.html
static cl::OptionCategory CSV_Parsing_Options("CSV Parsing Options", "CSV Parsing Options.");
static cl::opt<bool,false> displaynames("names", cl::desc("display processed file names"), cl::cat(CSV_Parsing_Options));
static cl::alias displaynamesAlias("shownames",cl::desc("display processed file names -names"),cl::aliasopt(displaynames));

static cl::opt<std::string> setDelimitor("set", cl::desc("<input delimitor>"),cl::cat(CSV_Parsing_Options) );
static cl::alias setDelimitorAlias("delimitor",cl::desc("<input delimitor ...>"),cl::aliasopt(setDelimitor));

//static cl::opt<bool,false> specifyHeader("header",cl::desc("specify header"),cl::cat(CSV_Header_Options));
//static cl::alias setHeaderAlias("setHeader",cl::desc("specify header -header"),cl::aliasopt(specifyHeader));
static cl::list<std::string> inputFiles("file",cl::Positional, cl::desc("<input file ...>]") , cl::PositionalEatsArgs ,cl::cat(CSV_Parsing_Options));
static cl::alias inputFilesAlias("files",cl::desc("input file ... -file"),cl::aliasopt(inputFiles));

static cl::list<std::string> userHeader("header",cl::Positional, cl::desc("<input header ...>"), cl::PositionalEatsArgs, cl::cat(CSV_Parsing_Options));
static cl::alias userHeaderAlias("userHeader",cl::desc("input header ... [header]"),cl::aliasopt(userHeader));

//static cl::opt<std::string> inputFile(cl::Positional, cl::desc("<input file>"), cl::Required, cl::cat(CSV_Parsing_Options));
//  Multiple input files are allowed on the command line;
//static cl::opt<bool,false> inputOptionFiles("f",cl::desc("input file ..."),cl::cat(CSV_Parsing_Options));
//static cl::alias inputOptionFilesAlias("files",cl::desc("input file ... -f"),cl::aliasopt(inputOptionFiles));


static cl::list<std::string> inputDirs("dir",cl::Positional, cl::desc("[<input dir ...> or "), cl::PositionalEatsArgs,cl::cat(CSV_Parsing_Options));
static cl::alias inputDirsAlias("dirs",cl::desc("input dir ... -dir"),cl::aliasopt(inputDirs));


std::vector<fs::path> files;
std::vector<fs::path> dirs;
std::vector<string> allFiles;


//static cl::opt<bool,false> pyColorOption("c",cl::desc("set coloring of output"),cl::cat(pygrepFlags));
//static cl::alias pyColorOptionAlias0("colour",cl::desc("alias for coloring -c"),cl::aliasopt(pyColorOption));


class CSV_Masking : public PabloKernel {
    public:
        CSV_Masking(    BuilderRef kb, StreamSet * dquote, StreamSet * delimiter,  StreamSet * newline, StreamSet * basis, 
                        StreamSet * translatedBasis, StreamSet * field_starts, StreamSet* record_starts, StreamSet * mask, StreamSet * escapes,
                        StreamSet * emptyFirst, StreamSet * emptyMid, StreamSet * emptyLast, int fileHeaders
                    ):
        PabloKernel(kb, "CSV_Masking"+to_string(fileHeaders), {
            Binding{"dquote", dquote, FixedRate(), LookAhead(1)}, 
            Binding{"basis", basis},
            Binding{"newline",newline, FixedRate(), LookAhead(1)},
            Binding{"delimiter",delimiter},
        },
        {
            Binding{"translatedBasis",translatedBasis},
            Binding{"field_starts",field_starts},
            Binding{"record_starts",record_starts},
            Binding{"mask",mask},
            Binding{"escapes",escapes},
            Binding{"emptyFirst", emptyFirst},
            Binding{"emptyMid", emptyMid},
            Binding{"emptyLast", emptyLast},
        }, {}, {}),headers_from_file(fileHeaders){}
    protected:
        void generatePabloMethod() override;
        int headers_from_file;
};

class CSV_Marks : public PabloKernel {
public:
    CSV_Marks(BuilderRef kb, StreamSet * field_starts, StreamSet * escape,StreamSet * record_starts, int n, StreamSet * marks, int fileHeaders) 
        : PabloKernel(kb, "CSV_Marks"+to_string(n)+to_string(fileHeaders),
                      {Binding{"field_starts", field_starts, FixedRate(),LookAhead(1)},
                      Binding{"record_starts", record_starts},
                      Binding{"escape", escape}},
                      {Binding{"marks", marks}},
                      {},
                      {}),num_of_field(n),headers_from_file(fileHeaders){}
protected:
    void generatePabloMethod() override;
    int num_of_field;
    int headers_from_file;
};

class LookaheadDriver : public PabloKernel {
public:
    LookaheadDriver(    BuilderRef kb, StreamSet * emptyFirst, StreamSet * emptyMid, StreamSet * skewedEmptyLast, StreamSet * maskIn, StreamSet * fieldsIn, //inputs
                        StreamSet * maskOut, StreamSet * fieldsOut, StreamSet * emptyMask   //outputs
                    )
        : PabloKernel(kb, "LookaheadDriver",
                    {   Binding{"emptyFirst", emptyFirst},
                        Binding{"emptyMid", emptyMid},
                        Binding{"skewedEmptyLast", skewedEmptyLast, FixedRate(),LookAhead(1)},
                        Binding{"maskIn", maskIn},
                        Binding{"fieldsIn", fieldsIn}
                    },
                    {   Binding{"maskOut",maskOut},
                        Binding{"fieldsOut", fieldsOut},
                        Binding{"emptyMask", emptyMask}
                    },
                    {},
                    {}){}
protected:
    void generatePabloMethod() override;
};

class createNotDriver : public PabloKernel {
public:
    createNotDriver(    BuilderRef kb, StreamSet * in, //inputs
                        StreamSet * out   //outputs
                    )
        : PabloKernel(kb, "createNotDriver",
                    {   Binding{"in", in}
                    },
                    {   Binding{"out",out}
                    },
                    {},
                    {}){}
protected:
    void generatePabloMethod() override;
};

typedef void (*CSVTranslateFunctionType)(uint32_t fd);
CSVTranslateFunctionType generatePipeline(CPUDriver & pxDriver, int n, vector<string>& Header_Vec, unsigned bixnum_size, re::Name * CC_delimitor);

int Get_Field_Count(const char delimiter, const char* argv);
void Get_Header(const char delimiter, const char* dir, vector<string>& v, int n);

int main(int argc, char *argv[]) {
    //  ParseCommandLineOptions uses the LLVM CommandLine processor, but we also add
    //  standard Parabix command line options such as -help, -ShowPablo and many others.
    codegen::ParseCommandLineOptions(argc, argv, {&CSV_Parsing_Options,pablo::pablo_toolchain_flags(), codegen::codegen_flags()});
    //  A CPU driver is capable of compiling and running Parabix programs on the CPU.
    CPUDriver driver("csv_quote_xlator");
    

    char delimiter = ',';
    re::Name * CC_deli = NULL;
    re::RE * CC_re = re::RE_Parser::parse(setDelimitor);
    resolveUnicodeNames(CC_re);
    if(re::Name * UCD_property_name = dyn_cast<re::Name>(CC_re)){
        //cout<<UCD_property_name->getName()<<endl;
        CC_deli = dyn_cast<re::Name>(CC_re);
    }else if (re::CC * CC_ast = dyn_cast<re::CC>(CC_re)) {
        //cout<<CC_ast->count()<<endl;
        //cout<<makeName(CC_ast)->getFullName()<<endl;
        if(CC_ast->count()>1){
            std::cerr << "Only one type of delimitor is allowed.\n";
            exit(1);  
        }
        else if(CC_ast->count() == 1){
            CC_deli = makeName(dyn_cast<re::CC>(CC_re));
            if(strlen(setDelimitor.c_str())==1)
            {
                delimiter = setDelimitor[0];
            }
            else{
                string de = setDelimitor.c_str();
                if(de == "\\t") delimiter = '\t';   
                else if(de == "\\r") delimiter ='\r';
                else if(de == "'\\\\") delimiter ='\\';
                else if(de=="\\?") delimiter ='\?';
                else if(de=="\\'") delimiter = '\'';
            }
        }
    }else if (setDelimitor.length() != 0){
        std::cerr << "Input expression must be a Unicode property or CC but found: " << setDelimitor << " instead.\n";
        exit(1);
    }

    
    
    //cout<<"deli["<<setDelimitor.c_str()<<"]"<<"length:"<<strlen(setDelimitor.c_str())<<endl;
    //cout<<"deli["<<delimiter<<"]"<<endl;
    if(inputFiles.size()>0){
        files = argv::getFullFileList(driver, inputFiles);
        for(unsigned i=0;i<files.size();i++){
            allFiles.push_back(files[i].string());
        }
    }
    if(inputDirs.size()>0){
        dirs = argv::getFullFileList(driver, inputDirs);
        int valid = getAllFiles(dirs, allFiles);
        if(valid == -1){
            llvm::errs() << "Error: invalid dir input, program exiting\n";
            exit(-1);
        }
    }


    int fileHeaders=0;
    vector<string> header;
    
    if(userHeader.size()>0){
        setHeader(userHeader,header);
        /*
        cout<<"begin print header\n";
        vector<string>::iterator it;
        for(it=header.begin();it!=header.end();it++){
            cout<<*it<<endl;
        }
        cout<<"finished print header\n";
        */
    }
    for(unsigned i=0;i<allFiles.size();i++){
        std::string fileName = allFiles[i];
        if(userHeader.size()==0){
            header.clear();
            fileHeaders = Get_Field_Count(delimiter, fileName.c_str());
            Get_Header(delimiter,fileName.c_str(), header, fileHeaders);
        }
        unsigned max_length = 0;
        for(unsigned i=0;i<header.size();i++)   if(header[i].size()>max_length)    max_length=header[i].size();
            unsigned bixnum_size = (int)(log2(max_length))+1;
        //  Build and compile the Parabix pipeline by calling the Pipeline function above.
        CSVTranslateFunctionType fn = generatePipeline(driver, fileHeaders, header, bixnum_size, CC_deli);
        //  The compile function "fn"  can now be used.   It takes a file
        //  descriptor as an input, which is specified by the filename given by
        //  the inputFile command line option.
        
        const int fd = open(fileName.c_str(), O_RDONLY);
        if (LLVM_UNLIKELY(fd == -1)) {
            llvm::errs() << "Error: cannot open " << fileName << " for processing. Skipped.\n";
        } else {
            //  Run the pipeline.
            if(displaynames){
                if(i!=0) cout<<endl;
                cout<<"__________________________________________________________________________________\n";
                cout<<endl<<fileName<<"->"<<fileName.replace(fileName.find_last_of("."),4,".json\n\n");
            }
            
            fn(fd);
            close(fd);
            if(displaynames)
                cout<<"\n__________________________________________________________________________________\n";
        }
    }
    

    

    

    
    return 0;
}

void CSV_Masking::  generatePabloMethod() {
    pablo::PabloBuilder pb(getEntryScope());
    std::vector<PabloAST *> basis = getInputStreamSet("basis");
    PabloAST * dquotes = getInputStreamSet("dquote")[0];
    PabloAST * newlines = getInputStreamSet("newline")[0];
    PabloAST * delimiters = getInputStreamSet("delimiter")[0];
    
    //dquotes
    PabloAST * dquote_odd = pb.createEveryNth(dquotes, pb.getInteger(2));
    PabloAST * dquote_even = pb.createXor(dquotes, dquote_odd);
    PabloAST * escaped_quote = pb.createAnd(dquote_even, pb.createLookahead(dquotes, 1));
    PabloAST * quote_escape = pb.createAdvance(escaped_quote, 1);
    PabloAST * start_dquote = pb.createXor(dquote_odd, quote_escape);
    PabloAST * end_dquote = pb.createXor(dquote_even, escaped_quote);
    PabloAST * surrandingDquotes = pb.createOr(start_dquote,end_dquote);
    
    //newline
    PabloAST * literal_newlines = pb.createAnd(newlines,pb.createIntrinsicCall(pablo::Intrinsic::InclusiveSpan, {start_dquote, end_dquote}));
    PabloAST * CSV_newlines = pb.createXor(newlines,literal_newlines);//newlines not in strings
    PabloAST * double_newline_end = pb.createAdvance(pb.createAnd(newlines,pb.createLookahead(newlines,1)),1);

    //record start
    PabloAST * first = pb.createNot(pb.createAdvance(pb.createOnes(),1));
    PabloAST * record_starts = pb.createXor3(CSV_newlines,double_newline_end, first);

    //header
    PabloAST * header = pb.createZeroes();
    if(headers_from_file!=0){
        header = pb.createScanThru(pb.createOnes(),CSV_newlines);
    }
    
    //PabloAST * headerMask = pb.createXor(header,pb.createAnd(header,surrandingDquotes));

    //delimiters
    PabloAST * literal_delimiters = pb.createAnd(delimiters,pb.createIntrinsicCall(pablo::Intrinsic::InclusiveSpan, {start_dquote, end_dquote}));
    PabloAST * CSV_delimiters = pb.createXor(delimiters,literal_delimiters);//delimiter not in strings
    
    PabloAST *  toDelete = pb.createOr(pb.createOr(CSV_delimiters,surrandingDquotes),quote_escape);// to be deleted from csv data
    
    //empty fields
    PabloAST * empty_first_fields = pb.createAnd(pb.createAdvance(CSV_newlines, 1), CSV_delimiters);

    PabloAST * empty_intermed_fields = pb.createAnd(pb.createAdvance(CSV_delimiters, 1), CSV_delimiters);

    PabloAST * empty_last_fields = pb.createAnd(pb.createAdvance(CSV_delimiters, 1), CSV_newlines);

    PabloAST * empty_both = pb.createOr(empty_first_fields, empty_intermed_fields);

    //escapes
    PabloAST * escapes = pb.createOr(escaped_quote,literal_newlines);

    //field starts
    
    
    PabloAST * syntactical = pb.createOr(delimiters, newlines);

    PabloAST * char_after_delim = pb.createOr(pb.createAdvance(CSV_delimiters, 1), pb.createAdvance(record_starts,1));

    PabloAST * firstInField = pb.createAnd(pb.createNot(syntactical), char_after_delim);

    PabloAST * char_after_start_quote = pb.createAnd(pb.createAdvance(firstInField, 1), pb.createAdvance(start_dquote,1));//pb.createAdd(pb.createOr(pb.createAdvance(CSV_delimiters,1),pb.createAdvance(record_starts,1)),start_dquote);

    PabloAST * noquote_field_starts = pb.createXor(firstInField, start_dquote);

    PabloAST * field_starts = pb.createOr3(noquote_field_starts, char_after_start_quote, empty_both);
    
    //filtermask
    PabloAST * mask = pb.createNot(toDelete);

    mask = pb.createOr(mask, empty_both);
    
    if(headers_from_file!=0)
        mask = pb.createAnd(pb.createNot(header),mask);
    
    //getVars
    Var * field_startsVar = getOutputStreamVar("field_starts");
    Var * maskVar = getOutputStreamVar("mask");
    Var * escapeVar = getOutputStreamVar("escapes");
    Var * record_startsVar = getOutputStreamVar("record_starts");
    Var * emptyFirstVar = getOutputStreamVar("emptyFirst");
    Var * emptyMidVar = getOutputStreamVar("emptyMid");
    Var * emptyLastVar = getOutputStreamVar("emptyLast");

    // Translate \n to n    ASCII value of \n = 0x0d, ASCII value of n = 0x6e   blackslashes will be added lateron to show \n
    std::vector<PabloAST *> translated_basis(8, nullptr);
    translated_basis[0] = basis[0];                                     // nochange
    translated_basis[1] = basis[1];                                     // nochange
    translated_basis[2] = pb.createXor(basis[2], literal_newlines);     // flip
    translated_basis[3] = basis[3];                                     // nochange
    translated_basis[4] = basis[4];                                     // nochange
    translated_basis[5] = pb.createXor(basis[5], literal_newlines);     // flip
    translated_basis[6] = pb.createXor(basis[6], literal_newlines);     // flip
    translated_basis[7] = basis[7];                                     // nochange

    Var * translatedVar = getOutputStreamVar("translatedBasis");
    for (unsigned i = 0; i < 8; i++) {
        pb.createAssign(pb.createExtract(translatedVar, pb.getInteger(i)), translated_basis[i]);
    }

    pb.createAssign(pb.createExtract(field_startsVar, pb.getInteger(0)), field_starts);
    pb.createAssign(pb.createExtract(maskVar, pb.getInteger(0)), mask);
    pb.createAssign(pb.createExtract(escapeVar, pb.getInteger(0)), escapes);
    pb.createAssign(pb.createExtract(record_startsVar, pb.getInteger(0)), record_starts);
    pb.createAssign(pb.createExtract(emptyFirstVar, pb.getInteger(0)), empty_first_fields);
    pb.createAssign(pb.createExtract(emptyMidVar, pb.getInteger(0)), empty_intermed_fields);
    pb.createAssign(pb.createExtract(emptyLastVar, pb.getInteger(0)), empty_last_fields);
}

void LookaheadDriver::generatePabloMethod(){
    pablo::PabloBuilder pb(getEntryScope());
    PabloAST * emptyFirst = getInputStreamSet("emptyFirst")[0];
    PabloAST * emptyMid = getInputStreamSet("emptyMid")[0];
    PabloAST * emptyFieldsIn = getInputStreamSet("skewedEmptyLast")[0];
    PabloAST * maskIn = getInputStreamSet("maskIn")[0];
    PabloAST * fieldsIn = getInputStreamSet("fieldsIn")[0];

    PabloAST * emptyLast = pb.createLookahead(emptyFieldsIn, 1);

    Var * marksOutVar = getOutputStreamVar("maskOut");
    Var * fieldsOutVar = getOutputStreamVar("fieldsOut");
    Var * emptyMaskVar = getOutputStreamVar("emptyMask");

    pb.createAssign(pb.createExtract(marksOutVar, pb.getInteger(0)), pb.createOr(maskIn, emptyLast));
    pb.createAssign(pb.createExtract(fieldsOutVar, pb.getInteger(0)), pb.createOr(fieldsIn, emptyLast));
    pb.createAssign(pb.createExtract(emptyMaskVar, pb.getInteger(0)), pb.createOr3(emptyFirst, emptyMid, emptyLast));
}

void createNotDriver::generatePabloMethod(){
    pablo::PabloBuilder pb(getEntryScope());
    PabloAST * in = getInputStreamSet("in")[0];

    Var * outVar = getOutputStreamVar("out");

    pb.createAssign(pb.createExtract(outVar, pb.getInteger(0)), pb.createNot(in));
}

void CSV_Marks::generatePabloMethod() {
    pablo::PabloBuilder pb(getEntryScope());
    PabloAST * field_starts = getInputStreamSet("field_starts")[0];
    PabloAST * escape = getInputStreamSet("escape")[0];
    PabloAST * record_starts = getInputStreamSet("record_starts")[0];
    PabloAST * first = pb.createNot(pb.createAdvance(pb.createOnes(),1));
    PabloAST * betweenRecords = pb.createAnd(pb.createLookahead(field_starts,1),record_starts);
    //field_starts=pb.createAnd(field_starts,pb.createNot(first));
    //PabloAST * starts = pb.createLookahead(pb.createOr(record_starts,field_starts),1);

    Var * marksVar = getOutputStreamVar("marks");
    if(num_of_field>0){
        //for first headers of every record 
        PabloAST ** fields = (PabloAST**)malloc(sizeof(PabloAST *)*num_of_field);
        fields[0]=pb.createEveryNth(field_starts,pb.getInteger(num_of_field));
        pb.createAssign(pb.createExtract(marksVar, pb.getInteger(0)),fields[0]);

        //nth headers of every record. seconds, thirds and so on
        for (int i = 1; i < num_of_field; i++) {
            fields[i]=pb.createIndexedAdvance(fields[i-1],field_starts,pb.getInteger(1));
            pb.createAssign(pb.createExtract(marksVar, pb.getInteger(i)), fields[i]);
        }
        if(headers_from_file == 0){
            fields[0]=pb.createAnd(pb.createNot(pb.createAdd(pb.createAdvance(first,1),first)),fields[0]);
            pb.createAssign(pb.createExtract(marksVar, pb.getInteger(0)),fields[0]);
        }
    }
    
    pb.createAssign(pb.createExtract(marksVar, pb.getInteger(num_of_field)), escape);
    //pb.createAssign(pb.createExtract(marksVar, pb.getInteger(num_of_field+1)), pb.createXor(betweenRecords,first));
    pb.createAssign(pb.createExtract(marksVar, pb.getInteger(num_of_field+1)), pb.createAnd(betweenRecords,pb.createNot(first)));
    pb.createAssign(pb.createExtract(marksVar, pb.getInteger(num_of_field+2)), first);
    pb.createAssign(pb.createExtract(marksVar, pb.getInteger(num_of_field+3)), pb.createXor(betweenRecords,record_starts));

}

CSVTranslateFunctionType generatePipeline(CPUDriver & pxDriver, int fileHeaders, vector<string>& Header_Vec, unsigned bixnum_size, re::Name *CC_delimitor) {
    // A Parabix program is build as a set of kernel calls called a pipeline.
    // A pipeline is construction using a Parabix driver object.
    auto & b = pxDriver.getBuilder();
    auto P = pxDriver.makePipeline({Binding{b->getInt32Ty(), "inputFileDecriptor"}}, {});
    //  The program will use a file descriptor as an input.
    Scalar * fileDescriptor = P->getInputScalar("inputFileDecriptor");
    // File data from mmap
    StreamSet * ByteStream = P->CreateStreamSet(1, 8);
    //  MMapSourceKernel is a Parabix Kernel that produces a stream of bytes
    //  from a file descriptor.
    P->CreateKernelCall<MMapSourceKernel>(fileDescriptor, ByteStream);

    //  The Parabix basis bits representation is created by the Parabix S2P kernel.
    //  S2P stands for serial-to-parallel.
    StreamSet * BasisBitsWithCR = P->CreateStreamSet(8);
    
    P->CreateKernelCall<S2PKernel>(ByteStream, BasisBitsWithCR);

    StreamSet * CR = P->CreateStreamSet(1);
    P->CreateKernelCall<CharacterClassKernelBuilder>(std::vector<re::CC *>{re::makeByte(0x0d)}, BasisBitsWithCR, CR);
    StreamSet * CRmask = P->CreateStreamSet(1);

    StreamSet * BasisBits = P->CreateStreamSet(8);
    P->CreateKernelCall<createNotDriver>(CR,CRmask);
    FilterByMask(P,CRmask,BasisBitsWithCR,BasisBits);	//in this kernel call we filter out CRs from the data, to avoid complication later on

    //  We need to know which input positions are dquotes and which are not.
    StreamSet * Dquote = P->CreateStreamSet(1);
    P->CreateKernelCall<CharacterClassKernelBuilder>(std::vector<re::CC *>{re::makeByte(0x22)}, BasisBits, Dquote);
    
    StreamSet * delimiter = P->CreateStreamSet(1);
    if(CC_delimitor == NULL)
        P->CreateKernelCall<CharacterClassKernelBuilder>(std::vector<re::CC *>{re::makeByte(0x2c)}, BasisBits, delimiter);
    else
        P->CreateKernelCall<UnicodePropertyKernelBuilder>(CC_delimitor,BasisBits,delimiter);
    

    StreamSet * newline = P->CreateStreamSet(1);
    P->CreateKernelCall<CharacterClassKernelBuilder>(std::vector<re::CC *>{re::makeByte(0x0a)}, BasisBits, newline);

    StreamSet * Field_starts_incomplete = P->CreateStreamSet(1);    //output streamsets from Lookahead Driver kernels
    StreamSet * CSV_data_mask_incomplete = P->CreateStreamSet(1);
    StreamSet * escapes = P->CreateStreamSet(1);
    StreamSet * Record_starts = P->CreateStreamSet(1);
    StreamSet * TranslatedBasis = P->CreateStreamSet(8,1);
    StreamSet * emptyFirst = P->CreateStreamSet(1);
    StreamSet * emptyMid = P->CreateStreamSet(1);
    StreamSet * emptyLastSkew = P->CreateStreamSet(1);

    P->CreateKernelCall<CSV_Masking>(   Dquote, delimiter, newline, BasisBits,
                                        TranslatedBasis, Field_starts_incomplete,Record_starts,CSV_data_mask_incomplete, 
                                        escapes, emptyFirst, emptyMid, emptyLastSkew, fileHeaders
                                    );

    //P->CreateKernelCall<DebugDisplayKernel>("CSV_data_mask", CSV_data_mask);
    // StreamSet * original = P->CreateStreamSet(1,8);
    // P->CreateKernelCall<P2SKernel>(BasisBits, original);
    // P->CreateKernelCall<DebugDisplayKernel>("inputfile",original);

    StreamSet * CSV_data_mask = P->CreateStreamSet(1);          //output streamsets from Lookahead Driver kernels
    StreamSet * Field_starts = P->CreateStreamSet(1);
    StreamSet * empties = P->CreateStreamSet(1);

    P->CreateKernelCall<LookaheadDriver>(   emptyFirst, emptyMid, emptyLastSkew, CSV_data_mask_incomplete, Field_starts_incomplete,
                                            CSV_data_mask, Field_starts, empties);

    //filtering streamsets to remove csv syntax
    StreamSet * FilteredBasis = P->CreateStreamSet(8,1);
    FilterByMask(P,CSV_data_mask,TranslatedBasis,FilteredBasis);

    StreamSet * Filtered_field_starts = P->CreateStreamSet(1);
    FilterByMask(P,CSV_data_mask,Field_starts,Filtered_field_starts);

    StreamSet * Filtered_escapes = P->CreateStreamSet(1);
    FilterByMask(P,CSV_data_mask,escapes,Filtered_escapes);

    StreamSet * Filtered_record_starts = P->CreateStreamSet(1);
    FilterByMask(P,CSV_data_mask,Record_starts,Filtered_record_starts);

    StreamSet * FilteredEmpties = P->CreateStreamSet(1);
    FilterByMask(P,CSV_data_mask,empties,FilteredEmpties);
    
    
    // StreamSet * Filtered_mask = P->CreateStreamSet(1);
    // FilterByMask(P,CSV_data_mask,CSV_data_mask,Filtered_mask);
    // P->CreateKernelCall<DebugDisplayKernel>("Filtered_mask", Filtered_mask);
    // P->CreateKernelCall<DebugDisplayKernel>("Record_starts", Record_starts);
    // P->CreateKernelCall<DebugDisplayKernel>("mask", CSV_data_mask);
    // P->CreateKernelCall<DebugDisplayKernel>("first", emptyFirst);
    // P->CreateKernelCall<DebugDisplayKernel>("mid", emptyMid);
    // P->CreateKernelCall<DebugDisplayKernel>("empties", FilteredEmpties);
    //  P->CreateKernelCall<DebugDisplayKernel>("Filtered_field_starts", Filtered_field_starts);
    //  P->CreateKernelCall<DebugDisplayKernel>("Filtered_escapes", Filtered_escapes);
    // P->CreateKernelCall<DebugDisplayKernel>("Field_starts", Field_starts);
    //  P->CreateKernelCall<DebugDisplayKernel>("Filtered_record_starts", Filtered_record_starts);
    
    //  StreamSet * FilteredByte = P->CreateStreamSet(1,8);           //for debug purpose
    //  P->CreateKernelCall<P2SKernel>(FilteredBasis, FilteredByte);
     //P->CreateKernelCall<DebugDisplayKernel>("FilteredBytes", FilteredByte);
    //  P->CreateKernelCall<StdOutKernel>(FilteredByte);
   
    //StreamSet* maskPlus = P->CreateStreamSet(1);
    //P->CreateKernelCall<AddSentinel>(Filtered_mask,maskPlus);
    //P->CreateKernelCall<DebugDisplayKernel>("maskPlus", maskPlus);
    //StreamSet* FilteredBasisPlus = P->CreateStreamSet(8);
    //P->CreateKernelCall<AddSentinel>(FilteredBasis,FilteredBasisPlus);

    unsigned marksize = Header_Vec.size();
    StreamSet * InsertMarks = P->CreateStreamSet(marksize);
    P->CreateKernelCall<CSV_Marks>(Filtered_field_starts,Filtered_escapes,Filtered_record_starts, Header_Vec.size()-4,InsertMarks, fileHeaders);
    //P->CreateKernelCall<CSV_Marks>(startPlus,escapePlus,,n,InsertMarks);
    // P->CreateKernelCall<DebugDisplayKernel>("InsertMarks", InsertMarks);
    
    //P->CreateKernelCall<DebugDisplayKernel>("FilteredBasis", FilteredBasis);
    unsigned insertLengthBits = bixnum_size;  //needs to be changed later, will create problems if max field name length is greater than 31
                                    //cannot be less than 5 with current program

    StreamSet * const InsertBixNum = P->CreateStreamSet(insertLengthBits,1);
    P->CreateKernelCall<StringInsertBixNum>(Header_Vec, InsertMarks, InsertBixNum);

    StreamSet * const SpreadMask = InsertionSpreadMask(P, InsertBixNum, InsertPosition::Before);
    StreamSet * ExpandedBasis = P->CreateStreamSet(8);
    StreamSet * const InsertIndex = P->CreateStreamSet(insertLengthBits);

    P->CreateKernelCall<RunIndex>(SpreadMask, InsertIndex, nullptr, true);

    //StreamSet * FilteredBasisPlus = P->CreateStreamSet(8);
    //->CreateKernelCall<AddSentinel>(FilteredBasis,FilteredBasisPlus);
   //P->CreateKernelCall<DebugDisplayKernel>("InsertIndex", InsertIndex);
    SpreadByMask(P, SpreadMask, FilteredBasis, ExpandedBasis);

    // StreamSet * ExpandedBytes = P->CreateStreamSet(1,8);
    // P->CreateKernelCall<P2SKernel>(ExpandedBasis, ExpandedBytes);
    // P->CreateKernelCall<DebugDisplayKernel>("Expanded Basis", ExpandedBytes);

    StreamSet * ExpandedMarks = P->CreateStreamSet(marksize);
    SpreadByMask(P, SpreadMask, InsertMarks, ExpandedMarks);

    StreamSet * ExpandedEmpties = P->CreateStreamSet(1);
    SpreadByMask(P, SpreadMask, FilteredEmpties, ExpandedEmpties);

    StreamSet * FilledBasis = P->CreateStreamSet(8);
    P->CreateKernelCall<StringReplaceKernel>(Header_Vec, ExpandedBasis, SpreadMask, ExpandedMarks, InsertIndex, FilledBasis);

    StreamSet * EmptiesMask = P->CreateStreamSet(1);
    P->CreateKernelCall<createNotDriver>(ExpandedEmpties, EmptiesMask);

    StreamSet * FinalBasis = P->CreateStreamSet(8);
    FilterByMask(P,EmptiesMask,FilledBasis,FinalBasis);
    
    StreamSet * FilledBytes  = P->CreateStreamSet(1, 8);
    P->CreateKernelCall<P2SKernel>(FinalBasis, FilledBytes);

    // P->CreateKernelCall<DebugDisplayKernel>("output", FilledBytes);
    
    //output to file
    P->CreateKernelCall<StdOutKernel>(FilledBytes);
    return reinterpret_cast<CSVTranslateFunctionType>(P->compile());
}

int Get_Field_Count(const char delimiter, const char* argv) {
	string ipath;
	//argv[0] is the paths of the input file.
	ipath = string(argv);
	ifstream csvInput;
	csvInput.open(ipath, ios::in);
    ostringstream  tmp;
	tmp << csvInput.rdbuf();
	string  str = tmp.str();

	int FieldNumber = 0;
    
		const char *mystart = str.c_str();    
		bool instring{ false };
		for (const char* p = mystart; *p; p++) {  // iterate through the string
			if (*p == '"')                        
				instring = !instring;
			else if (*p == delimiter && !instring) {    // if comma OUTSIDE double quote
				FieldNumber++;  // count the field
			}
			else if (*p == '\n' && !instring) {
				break;
			}
		}
	csvInput.close();
	return FieldNumber + 1;//the number of delimiter +1
}

void Get_Header(const char delimiter, const char* dir, vector<string>& v, int n){
    ifstream in(dir);
    string fieldname;
    for(int i=0;i<n-1;i++){
        getline(in,fieldname,delimiter);
        int len=fieldname.size();
        if(len==0)  fieldname="header"+to_string(i);
        else{
            if(fieldname[0]=='\"' && fieldname[len-1]=='\"'){
                if(len<3)   fieldname="header"+to_string(i);
                else    fieldname=fieldname.substr(1, len-2);
            }
        }
        if(i==0) {
            v.push_back("{\"" + fieldname + "\":\"");  //first fieldname
        }
        else v.push_back("\",\"" + fieldname + "\":\"");
    }
    string last;
    getline(in,last);
    if(last[last.size()-1]=='\r'){  //getline above returns the file contents before the first '\n' character. If the file was created in windows, this line feed will be preceded by a carriage return which must be removed.
        last=last.substr(0, last.size()-1);
    }
    int len=last.size();
    if(len==0)  last="header"+to_string(n-1);
    else{
        if(last[0]=='\"' && last[len-1]=='\"'){
            if(len<3)   last="header"+to_string(n-1);
            else    last=last.substr(1, len-2);
        }
    }
    v.push_back("\",\"" + last + "\":\"");    //last fieldname

    v.push_back("\\");                          //back slash
    v.push_back("\"},");                            //rec start
    if(n>0)                                     //first
        v.push_back("[");
    else
        v.push_back("[\n");
    v.push_back("\"}\n]");
    // for(int i=0;i<n+4;i++){
    //     cout<<v[i]<<endl;
    // }
    return;
}

void setHeader(cl::list<std::string> &userHeader, vector<string> &header)
{
    cl::list<std::string>::iterator it;
    unsigned i=0;
    for(it=userHeader.begin();it!=userHeader.end();it++,i++){
        if(i==0)
            header.push_back("{\""+*it+"\":\"");
        else if(i==userHeader.size())
            header.push_back("\",\"" + *it + "\":\"");
        else 
            header.push_back("\",\"" + *it + "\":\"");  
    }
    header.push_back("\\");                          //back slash
    header.push_back("\"},");                            //rec start
    if(userHeader.size()>0)                                     //first
        header.push_back("[\n"+header.front());
    else
        header.push_back("[\n");
    header.push_back("\"}\n]");
}

int getAllFiles(std::vector<fs::path> &dirs, std::vector<string> &allFiles)
{
    std::vector<fs::path>::iterator it;
    for(it = dirs.begin();it!= dirs.end();it++){
        string dirName = it->string();
        if(dirName.find_last_of("/")!=dirName.length())
            dirName+="/";
        DIR* folder=opendir(dirName.c_str());
        if(!folder)
        {
            return -1;
        }
        struct dirent* file = NULL;
        size_t index;
        file=readdir(folder);
        while(file)
        {
            string filename=file->d_name;
            if((index=filename.find(".csv"))!=filename.npos)
            allFiles.push_back(dirName+filename);
            file=readdir(folder);
        }
        closedir(folder);
    }

    return 1;
}
