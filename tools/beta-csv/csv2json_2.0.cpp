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

#include <kernel/streamutils/sentinel.h>
#include <kernel/streamutils/string_insert.h>
#include <kernel/streamutils/run_index.h>
#include <pablo/parse/pablo_source_kernel.h> 
using namespace kernel;
using namespace llvm;
using namespace pablo;
using namespace std;


int Get_Field_Count(const char delimiter, const char* argv);
void Get_Header(const char delimiter, const char* dir, vector<string>& v, int n);

//  The declarations below are for command line processing.
//  See the LLVM CommandLine 2.0 Library Manual https://llvm.org/docs/CommandLine.html
static cl::OptionCategory CSV_Parsing_Options("Translation Options", "Translation Options.");
static cl::opt<std::string> inputFile(cl::Positional, cl::desc("<input file>"), cl::Required, cl::cat(CSV_Parsing_Options));
static cl::opt<std::string> outputFile(cl::Positional, cl::desc("<output file>"), cl::cat(CSV_Parsing_Options));


class CSV_Masking : public PabloKernel {
    public:
        CSV_Masking(BuilderRef kb, StreamSet * dquote, StreamSet * delimiter,  StreamSet * newline, StreamSet * CR, StreamSet * basis, 
        StreamSet * translatedBasis, StreamSet * field_starts, StreamSet* record_starts, StreamSet* starts, StreamSet * mask, StreamSet * escapes, StreamSet * testOut, StreamSet * testOut1, StreamSet * testOut2):
        PabloKernel(kb, "CSV_Masking", {
            Binding{"dquote", dquote, FixedRate(), LookAhead(1)}, 
            Binding{"basis", basis},
            Binding{"newline",newline, FixedRate(), LookAhead(1)},
            Binding{"delimiter",delimiter},
            Binding{"CR",CR}
        },
        {
            Binding{"translatedBasis",translatedBasis},
            Binding{"field_starts",field_starts},
            Binding{"record_starts",record_starts},
            Binding{"starts",starts},
            Binding{"mask",mask},
            Binding{"escapes",escapes},
            Binding{"testOut", testOut},
            Binding{"testOut1", testOut1},
            Binding{"testOut2", testOut2}
        }, {}, {}){}
    protected:
        void generatePabloMethod() override;
};

class CSV_Marks : public PabloKernel {
public:
    CSV_Marks(BuilderRef kb, StreamSet * field_starts, StreamSet * escape,StreamSet * record_starts, int n, StreamSet * marks) 
        : PabloKernel(kb, "CSV_Marks"+to_string(n),
                      {Binding{"field_starts", field_starts, FixedRate(),LookAhead(1)},
                      Binding{"record_starts", record_starts},
                      Binding{"escape", escape}},
                      {Binding{"marks", marks}},
                      {},
                      {}),num_of_field(n){}
protected:
    void generatePabloMethod() override;
    int num_of_field;
};

typedef void (*CSVTranslateFunctionType)(uint32_t fd);
CSVTranslateFunctionType generatePipeline(CPUDriver & pxDriver, int n, vector<string>& Header_Vec, string s);

int Get_Field_Count(const char delimiter, const char* argv);
void Get_Header(const char delimiter, const char* dir, vector<string>& v, int n);

int main(int argc, char *argv[]) {
    //  ParseCommandLineOptions uses the LLVM CommandLine processor, but we also add
    //  standard Parabix command line options such as -help, -ShowPablo and many others.
    codegen::ParseCommandLineOptions(argc, argv, {&CSV_Parsing_Options, pablo::pablo_toolchain_flags(), codegen::codegen_flags()});
    //  A CPU driver is capable of compiling and running Parabix programs on the CPU.
    CPUDriver driver("csv_quote_xlator");
    
    const char delimiter = ',';
    int n = Get_Field_Count(delimiter, inputFile.c_str());
    vector<string> header;
    Get_Header(delimiter,inputFile.c_str(), header, n);
    
    //  Build and compile the Parabix pipeline by calling the Pipeline function above.
    CSVTranslateFunctionType fn = generatePipeline(driver,n,header,"}\n]");
    //  The compile function "fn"  can now be used.   It takes a file
    //  descriptor as an input, which is specified by the filename given by
    //  the inputFile command line option.
    const int fd = open(inputFile.c_str(), O_RDONLY);
    if (LLVM_UNLIKELY(fd == -1)) {
        llvm::errs() << "Error: cannot open " << inputFile << " for processing. Skipped.\n";
    } else {
        //  Run the pipeline.
        fn(fd);
        close(fd);
    }
    return 0;
}

void CSV_Masking::generatePabloMethod() {
    pablo::PabloBuilder pb(getEntryScope());
    std::vector<PabloAST *> basis = getInputStreamSet("basis");
    PabloAST * dquotes = getInputStreamSet("dquote")[0];
    PabloAST * newlines = getInputStreamSet("newline")[0];
    PabloAST * delimiters = getInputStreamSet("delimiter")[0];
    PabloAST * CRs = getInputStreamSet("CR")[0];
    
    //dquotes
    PabloAST * dquote_odd = pb.createEveryNth(dquotes, pb.getInteger(2));
    PabloAST * dquote_even = pb.createXor(dquotes, dquote_odd);
    PabloAST * quote_escape = pb.createAnd(dquote_even, pb.createLookahead(dquotes, 1));
    PabloAST * escaped_quote = pb.createAdvance(quote_escape, 1);
    PabloAST * start_dquote = pb.createXor(dquote_odd, escaped_quote);
    PabloAST * end_dquote = pb.createXor(dquote_even, quote_escape);
    PabloAST * surrandingDquotes = pb.createOr(start_dquote,end_dquote);
    
    //newline
    PabloAST * literal_CRs = pb.createAnd(CRs,pb.createIntrinsicCall(pablo::Intrinsic::InclusiveSpan, {start_dquote, end_dquote}));
    PabloAST * literal_newlines = pb.createAnd(pb.createXor(newlines,literal_CRs),pb.createIntrinsicCall(pablo::Intrinsic::InclusiveSpan, {start_dquote, end_dquote}));
    PabloAST * CSV_newlines = pb.createXor(newlines,literal_newlines);//newlines not in strings
    PabloAST * double_newline_end = pb.createAdvance(pb.createAnd(newlines,pb.createLookahead(newlines,1)),1);

    //record start
    PabloAST * record_starts = pb.createXor(CSV_newlines,double_newline_end);

    //header
    PabloAST * header = pb.createScanThru(pb.createOnes(),CSV_newlines);
    //PabloAST * headerMask = pb.createXor(header,pb.createAnd(header,surrandingDquotes));

    //delimiters
    PabloAST * literal_delimiters = pb.createAnd(delimiters,pb.createIntrinsicCall(pablo::Intrinsic::InclusiveSpan, {start_dquote, end_dquote}));
    PabloAST * CSV_delimiters = pb.createXor(delimiters,literal_delimiters);//delimiter not in strings
    
    PabloAST *  toDelete = pb.createOr3(pb.createOr3(CSV_delimiters,surrandingDquotes,literal_CRs),quote_escape,CRs);// to be deleted from csv data
   
    PabloAST * mask = pb.createNot(toDelete);

    PabloAST * char_after_delim = pb.createOr(pb.createAdvance(CSV_delimiters, 1), pb.createAdvance(record_starts,1));

    PabloAST * char_after_start_quote = pb.createAnd(pb.createAdvance(char_after_delim, 1), pb.createAdvance(start_dquote,1));//pb.createAdd(pb.createOr(pb.createAdvance(CSV_delimiters,1),pb.createAdvance(record_starts,1)),start_dquote);

    PabloAST * noquote_field_starts = pb.createXor(char_after_delim, start_dquote);

    PabloAST *field_starts = pb.createOr(noquote_field_starts, char_after_start_quote);
    // pb.createIntrinsicCall(pablo::Intrinsic::PrintRegister, {CSV_delimiters});

    PabloAST * escapes = pb.createOr(escaped_quote,literal_newlines);
    
    //mask= pb.createXor(pb.createAnd(pb.createNot(header),mask),last);
    mask= pb.createAnd(pb.createNot(header),mask);

    Var * field_startsVar = getOutputStreamVar("field_starts");
    Var * maskVar = getOutputStreamVar("mask");
    Var * escapeVar = getOutputStreamVar("escapes");
    Var * record_startsVar = getOutputStreamVar("record_starts");
    Var * startsVar = getOutputStreamVar("starts");
    Var * testOutVar = getOutputStreamVar("testOut");
    Var * testOut1Var = getOutputStreamVar("testOut1");
    Var * testOut2Var = getOutputStreamVar("testOut2");

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
    pb.createAssign(pb.createExtract(startsVar, pb.getInteger(0)), pb.createOr(record_starts,field_starts));
    pb.createAssign(pb.createExtract(testOutVar, pb.getInteger(0)), pb.createOr(pb.createAdvance(record_starts,1), pb.createAdvance(CSV_delimiters,1)));
    pb.createAssign(pb.createExtract(testOut1Var, pb.getInteger(0)), pb.createAdvance(record_starts,1));
    pb.createAssign(pb.createExtract(testOut2Var, pb.getInteger(0)), pb.createAdvance(CSV_delimiters,1));
}

void CSV_Marks::generatePabloMethod() {
    pablo::PabloBuilder pb(getEntryScope());
    PabloAST * field_starts = getInputStreamSet("field_starts")[0];
    PabloAST * escape = getInputStreamSet("escape")[0];
    PabloAST * record_starts = getInputStreamSet("record_starts")[0];
    PabloAST * first = pb.createNot(pb.createAdvance(pb.createOnes(),1));
    PabloAST * betweenRecords = pb.createAnd(pb.createLookahead(field_starts,1),record_starts);

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
    }
    
    pb.createAssign(pb.createExtract(marksVar, pb.getInteger(num_of_field)), escape);
    pb.createAssign(pb.createExtract(marksVar, pb.getInteger(num_of_field+1)), pb.createXor(betweenRecords,first));
    pb.createAssign(pb.createExtract(marksVar, pb.getInteger(num_of_field+2)), first);
    pb.createAssign(pb.createExtract(marksVar, pb.getInteger(num_of_field+3)), pb.createXor(betweenRecords,record_starts));
}

CSVTranslateFunctionType generatePipeline(CPUDriver & pxDriver, int n, vector<string>& Header_Vec, string s) {
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
    StreamSet * BasisBits = P->CreateStreamSet(8);
    P->CreateKernelCall<S2PKernel>(ByteStream, BasisBits);

    //  We need to know which input positions are dquotes and which are not.
    StreamSet * Dquote = P->CreateStreamSet(1);
    P->CreateKernelCall<CharacterClassKernelBuilder>(std::vector<re::CC *>{re::makeByte(0x22)}, BasisBits, Dquote);
    
     StreamSet * delimiter = P->CreateStreamSet(1);
    P->CreateKernelCall<CharacterClassKernelBuilder>(std::vector<re::CC *>{re::makeByte(0x2c)}, BasisBits, delimiter);
    // P->CreateKernelCall<DebugDisplayKernel>("delimiter", delimiter);

    StreamSet * CR = P->CreateStreamSet(1);
    P->CreateKernelCall<CharacterClassKernelBuilder>(std::vector<re::CC *>{re::makeByte(0x0d)}, BasisBits, CR);

    StreamSet * newline = P->CreateStreamSet(1);
    P->CreateKernelCall<CharacterClassKernelBuilder>(std::vector<re::CC *>{re::makeByte(0x0a)}, BasisBits, newline);

    StreamSet * Field_starts = P->CreateStreamSet(1);
    StreamSet * starts = P->CreateStreamSet(1);
    StreamSet * CSV_data_mask = P->CreateStreamSet(1);
    StreamSet * escapes = P->CreateStreamSet(1);
    StreamSet * Record_starts = P->CreateStreamSet(1);
    StreamSet * TranslatedBasis = P->CreateStreamSet(8,1);
    StreamSet * testOut = P->CreateStreamSet(1);
    StreamSet * testOut1 = P->CreateStreamSet(1);
    StreamSet * testOut2 = P->CreateStreamSet(1);
    P->CreateKernelCall<CSV_Masking>(Dquote, delimiter, newline, CR, BasisBits, TranslatedBasis, 
    Field_starts,Record_starts, starts,CSV_data_mask, escapes, testOut, testOut1, testOut2);
    //P->CreateKernelCall<DebugDisplayKernel>("CSV_data_mask", CSV_data_mask);
    //StreamSet * original = P->CreateStreamSet(1,8);
    //P->CreateKernelCall<P2SKernel>(BasisBits, original);
    //P->CreateKernelCall<StdOutKernel>(original);

    StreamSet * FilteredBasis = P->CreateStreamSet(8,1);
    FilterByMask(P,CSV_data_mask,TranslatedBasis,FilteredBasis);
    StreamSet * Filtered_field_starts = P->CreateStreamSet(1);
    FilterByMask(P,CSV_data_mask,Field_starts,Filtered_field_starts);
    StreamSet * Filtered_escapes = P->CreateStreamSet(1);
    FilterByMask(P,CSV_data_mask,escapes,Filtered_escapes);
    StreamSet * Filtered_record_starts = P->CreateStreamSet(1);
    FilterByMask(P,CSV_data_mask,Record_starts,Filtered_record_starts);
    
    
    // StreamSet * Filtered_mask = P->CreateStreamSet(1);
    // FilterByMask(P,CSV_data_mask,CSV_data_mask,Filtered_mask);
    // P->CreateKernelCall<DebugDisplayKernel>("Filtered_mask", Filtered_mask);
    // P->CreateKernelCall<DebugDisplayKernel>("Record_starts", Record_starts);
    // P->CreateKernelCall<DebugDisplayKernel>("start_dquote", testOut);
    // P->CreateKernelCall<DebugDisplayKernel>("advanceRecord", testOut1);
    // P->CreateKernelCall<DebugDisplayKernel>("advanceDelim", testOut2);
    //  P->CreateKernelCall<DebugDisplayKernel>("Filtered_field_starts", Filtered_field_starts);
    // P->CreateKernelCall<DebugDisplayKernel>("Filtered_escapes", Filtered_escapes);
    //  P->CreateKernelCall<DebugDisplayKernel>("Field_starts", Field_starts);
    //  P->CreateKernelCall<DebugDisplayKernel>("Filtered_record_starts", Filtered_record_starts);
    
    // StreamSet * FilteredByte = P->CreateStreamSet(1,8);           //for debug purpose
    // P->CreateKernelCall<P2SKernel>(FilteredBasis, FilteredByte);
    // P->CreateKernelCall<DebugDisplayKernel>("FilteredBytes", FilteredByte);
   
    //StreamSet* maskPlus = P->CreateStreamSet(1);
    //P->CreateKernelCall<AddSentinel>(Filtered_mask,maskPlus);
    //P->CreateKernelCall<DebugDisplayKernel>("maskPlus", maskPlus);
    //StreamSet* FilteredBasisPlus = P->CreateStreamSet(8);
    //P->CreateKernelCall<AddSentinel>(FilteredBasis,FilteredBasisPlus);

    unsigned marksize = Header_Vec.size();
    StreamSet * InsertMarks = P->CreateStreamSet(marksize);
    P->CreateKernelCall<CSV_Marks>(Filtered_field_starts,Filtered_escapes,Filtered_record_starts, n,InsertMarks);
    //P->CreateKernelCall<CSV_Marks>(startPlus,escapePlus,,n,InsertMarks);
    // P->CreateKernelCall<DebugDisplayKernel>("Filtered_record_starts", Filtered_record_starts);
    
    //P->CreateKernelCall<DebugDisplayKernel>("FilteredBasis", FilteredBasis);
    unsigned insertLengthBits = 5;  //needs to be changed later, will create problems if max field name length is greater than 31
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



    StreamSet * FilledBasis = P->CreateStreamSet(8);
    P->CreateKernelCall<StringReplaceKernel>(Header_Vec, ExpandedBasis, SpreadMask, ExpandedMarks, InsertIndex, FilledBasis);

    StreamSet * FilledBytes  = P->CreateStreamSet(1, 8);
    P->CreateKernelCall<P2SKernel>(FilledBasis, FilledBytes);
    
    //output to file
    //Scalar * outputFileName = P->getInputScalar("outputFileName");
    //P->CreateKernelCall<FileSink>(outputFileName, FilledBytes);
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