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

#include <kernel/streamutils/string_insert.h>
#include <kernel/streamutils/run_index.h>
#include <pablo/parse/pablo_source_kernel.h> 
using namespace kernel;
using namespace llvm;
using namespace pablo;
using namespace std;


int Get_Field_Count(const char delimiter, const char* argv);
void Get_Header(const char delimiter, const char* dir, vector<string>& v, int n);
//  These declarations are for command line processing.
//  See the LLVM CommandLine 2.0 Library Manual https://llvm.org/docs/CommandLine.html
static cl::OptionCategory CSV_Parsing_Options("CSV Quote Translation Options", "CSV Quote Translation Options.");
static cl::opt<std::string> inputFile(cl::Positional, cl::desc("<input file>"), cl::Required, cl::cat(CSV_Parsing_Options));
static cl::opt<std::string> outputFile(cl::Positional, cl::desc("<output file>"), cl::cat(CSV_Parsing_Options));
//
//  The Hexify Kernel is the logic that produces hexadecimal output
//  from a source bit stream set called spreadBasis and the insertMask
//  used to spread out the bits.
//
class CSV_Quote_Translate : public PabloKernel {
public:
    CSV_Quote_Translate(BuilderRef kb, StreamSet * dquote, StreamSet * basis, StreamSet * translatedBasis)
        : PabloKernel(kb, "CSV_Quote_Translate",
                      {Binding{"dquote", dquote, FixedRate(), LookAhead(1)}, Binding{"basis", basis}},
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
    // Translate "" to \"    ASCII value of " = 0x22, ASCII value of \ = 0x5C
    std::vector<PabloAST *> translated_basis(8, nullptr);
    translated_basis[0] = basis[0];                              // no changes
    translated_basis[1] = pb.createXor(basis[1], quote_escape);  // flip
    translated_basis[2] = pb.createXor(basis[2], quote_escape);  // flip
    translated_basis[3] = pb.createXor(basis[3], quote_escape);  // flip
    translated_basis[4] = pb.createXor(basis[4], quote_escape);  // flip
    translated_basis[5] = pb.createXor(basis[5], quote_escape);  // flip
    translated_basis[6] = pb.createXor(basis[6], quote_escape);  // flip
    translated_basis[7] = basis[7];                              // no changes
    
    Var * translatedVar = getOutputStreamVar("translatedBasis");
    for (unsigned i = 0; i < 8; i++) {
        pb.createAssign(pb.createExtract(translatedVar, pb.getInteger(i)), translated_basis[i]);
    }
}





class CSV_Masking : public PabloKernel {
public:
    CSV_Masking(BuilderRef kb, StreamSet * dquote, StreamSet * delimiter,  StreamSet * newline, StreamSet * CR, StreamSet * basis, 
    StreamSet * translatedBasis, StreamSet * Dquote_surrounding, StreamSet * field_starts, StreamSet * field_follows, 
     StreamSet * record_starts, StreamSet * record_follows, StreamSet * mask, StreamSet * escapes, StreamSet * header)
        : PabloKernel(kb, "CSV_Masking",
                      {
                          Binding{"dquote", dquote, FixedRate(), LookAhead(1)}, 
                          Binding{"basis", basis},
                          Binding{"newline",newline},
                          Binding{"delimiter",delimiter},
                          Binding{"CR",CR}},
                      {
                          Binding{"translatedBasis",translatedBasis},
                          Binding{"field_starts",field_starts},
                          Binding{"field_follows",field_follows},
                          Binding{"record_starts", record_starts},
                          Binding{"record_follows", record_follows},
                          Binding{"Dquote_surrounding", Dquote_surrounding},
                          Binding{"mask",mask},
                          Binding{"header",header},
                          Binding{"escapes",escapes}},
                      {},
                      {}) {}
protected:
    void generatePabloMethod() override;
};

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
    PabloAST * CSV_newlines = pb.createXor(newlines,pb.createOr(literal_CRs,literal_newlines));

    //header
    PabloAST * header = pb.createScanThru(pb.createOnes(),pb.createAdvance(CSV_newlines,1));
    PabloAST * body = pb.createNot(header);
    PabloAST * headerMask = pb.createXor(header,pb.createAnd(header,surrandingDquotes));

    //delimiters
    PabloAST * literal_delimiters = pb.createAnd(delimiters,pb.createIntrinsicCall(pablo::Intrinsic::InclusiveSpan, {start_dquote, end_dquote}));
    PabloAST * CSV_delimiters = pb.createXor(delimiters,literal_delimiters);



    

    
    PabloAST * toDelete = pb.createZeroes();
    toDelete = pb.createOr(pb.createOr3(CSV_delimiters,surrandingDquotes,literal_CRs),quote_escape);
    //toDelete = pb.createOr(toDelete,CRs);
    //toDelete = pb.createOr(toDelete,quote_escape);
    PabloAST * mask = pb.createNot(toDelete);

    PabloAST * head = pb.createNot(pb.createAdvance(pb.createAnd(pb.createOnes(),body),1));
    PabloAST * last = pb.createAdvance(pb.createAdd(head,body),-1);
    PabloAST * record_starts = pb.createOr(head,pb.createAdvance(CSV_newlines,1));
    //PabloAST * reocrd_follows = CSV_newlines;

    PabloAST * field_starts = pb.createAdd(pb.createOr(record_starts,pb.createAdvance(CSV_delimiters,1)),start_dquote);
    PabloAST * field_follows = pb.createOr(CSV_newlines,CSV_delimiters);

    PabloAST * escapes = pb.createOr(escaped_quote,literal_newlines);
   
    mask= pb.createXor(pb.createAnd(pb.createNot(header),mask),last);
    

    Var * field_startsVar = getOutputStreamVar("field_starts");
    Var * field_followsVar = getOutputStreamVar("field_follows");
    Var * record_startsVar = getOutputStreamVar("record_starts");
    Var * record_followsVar = getOutputStreamVar("record_follows");
    Var * Dquote_surroundingVar = getOutputStreamVar("Dquote_surrounding");
    Var * maskVar = getOutputStreamVar("mask");
    Var * escapeVar = getOutputStreamVar("escapes");
    Var * headerVar = getOutputStreamVar("header");


    // Translate \n to n    ASCII value of \n = 0x0d, ASCII value of n = 0x6e
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
    pb.createAssign(pb.createExtract(field_followsVar, pb.getInteger(0)), field_follows);
    pb.createAssign(pb.createExtract(record_startsVar, pb.getInteger(0)), record_starts);
    pb.createAssign(pb.createExtract(record_followsVar, pb.getInteger(0)), CSV_newlines);
    pb.createAssign(pb.createExtract(maskVar, pb.getInteger(0)), mask);
    pb.createAssign(pb.createExtract(Dquote_surroundingVar, pb.getInteger(0)), surrandingDquotes);
    pb.createAssign(pb.createExtract(escapeVar, pb.getInteger(0)), escapes);
    pb.createAssign(pb.createExtract(headerVar, pb.getInteger(0)), headerMask);
}


class CSV_BixNum : public PabloKernel {
public:
    //int num_of_field = 3;
    CSV_BixNum(BuilderRef kb, StreamSet * filtered_starts, StreamSet * filtered_start_marks)
        : PabloKernel(kb, "CSV_BixNum",
                      {Binding{"filtered_starts", filtered_starts}},
                      {Binding{"filtered_start_marks", filtered_start_marks}}){ }
protected:
    void generatePabloMethod() override;
};


void CSV_BixNum::generatePabloMethod() {
    pablo::PabloBuilder pb(getEntryScope());
    PabloAST * starts = getInputStreamSet("filtered_starts")[0];
    /*
    for(int i=1;i<=3;i++){
        for(int j=1;j<i;j++){
            marks= pb.createAdd(marks,pb.createEveryNth(marks,pb.getInteger(i)));
        }
    }
    */
    PabloAST * marks = pb.createAdd(starts,starts);
    Var * marksVar = getOutputStreamVar("filtered_start_marks");
    pb.createAssign(pb.createExtract(marksVar, pb.getInteger(0)),marks);
}




typedef void (*CSVTranslateFunctionType)(uint32_t fd, const char *);

CSVTranslateFunctionType generatePipeline(CPUDriver & pxDriver, vector<string>& Header_Vec) {
    // A Parabix program is build as a set of kernel calls called a pipeline.
    // A pipeline is construction using a Parabix driver object.
    auto & b = pxDriver.getBuilder();
    auto P = pxDriver.makePipeline({Binding{b->getInt32Ty(), "inputFileDecriptor"},Binding{b->getInt8PtrTy(), "outputFileName"}}, {});
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

    StreamSet * CR = P->CreateStreamSet(1);
    P->CreateKernelCall<CharacterClassKernelBuilder>(std::vector<re::CC *>{re::makeByte(0x0d)}, BasisBits, CR);

    StreamSet * newline = P->CreateStreamSet(1);
    P->CreateKernelCall<CharacterClassKernelBuilder>(std::vector<re::CC *>{re::makeByte(0x0a)}, BasisBits, newline);


    StreamSet * Field_starts = P->CreateStreamSet(1);
    StreamSet * Field_follows = P->CreateStreamSet(1);
    StreamSet * CSV_data_mask = P->CreateStreamSet(1);
    StreamSet * Record_starts = P->CreateStreamSet(1);
    StreamSet * Record_follows = P->CreateStreamSet(1);
    StreamSet * Dquote_surrounding = P->CreateStreamSet(1);
    StreamSet * escapes = P->CreateStreamSet(1);
    StreamSet * TranslatedBasis = P->CreateStreamSet(8,1);
    StreamSet * header = P->CreateStreamSet(1);
    P->CreateKernelCall<CSV_Masking>(Dquote, delimiter, newline, CR, BasisBits, TranslatedBasis, Dquote_surrounding, Field_starts, 
    Field_follows, Record_starts, Record_follows, CSV_data_mask, escapes, header);
    /*
        CSV_Masking(BuilderRef kb, dquote, delimiter, newline,  CR, basis, 
    translatedBasis,  Dquote_surrounding,  field_starts,  field_follows, 
      record_starts, record_follows,  mask, escapes)*/
    
    P->CreateKernelCall<DebugDisplayKernel>("Dquote", Dquote);
    P->CreateKernelCall<DebugDisplayKernel>("Dquote_surrounding", Dquote_surrounding);
    P->CreateKernelCall<DebugDisplayKernel>("delimiter", delimiter);
    P->CreateKernelCall<DebugDisplayKernel>("BasisBits", BasisBits);
    P->CreateKernelCall<DebugDisplayKernel>("Field_starts", Field_starts);
    P->CreateKernelCall<DebugDisplayKernel>("Field_follows", Field_follows);
    P->CreateKernelCall<DebugDisplayKernel>("Record_starts", Record_starts);
    P->CreateKernelCall<DebugDisplayKernel>("Record_follows", Record_follows);
    P->CreateKernelCall<DebugDisplayKernel>("CSV_data_mask", CSV_data_mask);
    P->CreateKernelCall<DebugDisplayKernel>("header", header);
    StreamSet * original = P->CreateStreamSet(1,8);
    P->CreateKernelCall<P2SKernel>(BasisBits, original);
    P->CreateKernelCall<StdOutKernel>(original);


    StreamSet * FilteredHeader = P->CreateStreamSet(8,1);
    FilterByMask(P,CSV_data_mask,TranslatedBasis,FilteredHeader);
    StreamSet * FilteredBasis = P->CreateStreamSet(8,1);
    FilterByMask(P,CSV_data_mask,TranslatedBasis,FilteredBasis);
    StreamSet * Filtered_starts = P->CreateStreamSet(1);
    FilterByMask(P,CSV_data_mask,Field_starts,Filtered_starts);
    StreamSet * Filtered_escapes = P->CreateStreamSet(1);
    FilterByMask(P,CSV_data_mask,escapes,Filtered_escapes);

    P->CreateKernelCall<DebugDisplayKernel>("Filtered_starts", Filtered_starts);
    P->CreateKernelCall<DebugDisplayKernel>("Filtered_escapes", Filtered_escapes);


    
    StreamSet * Filtered = P->CreateStreamSet(1,8);
    P->CreateKernelCall<P2SKernel>(FilteredBasis, Filtered);
    cout<<Filtered->shapeString()<<endl;
    P->CreateKernelCall<StdOutKernel>(Filtered);
    
    unsigned insertLengthBits = 4;

    StreamSet * const InsertBixNum = P->CreateStreamSet(insertLengthBits,1);

    //StreamSet * InsertMarks = P->CreateStreamSet(1);
    
    //P->CreateKernelCall<CSV_BixNum>(Filtered_starts,InsertMarks);
    P->CreateKernelCall<StringInsertBixNum>(Header_Vec, Filtered_starts, InsertBixNum);
    P->CreateKernelCall<DebugDisplayKernel>("InsertBixNum", InsertBixNum);
    StreamSet * const SpreadMask = InsertionSpreadMask(P, InsertBixNum, InsertPosition::Before);
    P->CreateKernelCall<DebugDisplayKernel>("SpreadMask", SpreadMask);

    StreamSet * ExpandedBasis = P->CreateStreamSet(8);
    StreamSet * const InsertIndex = P->CreateStreamSet(insertLengthBits);
    P->CreateKernelCall<RunIndex>(SpreadMask, InsertIndex, nullptr, true); //invert =  true);

    SpreadByMask(P, SpreadMask, FilteredBasis, ExpandedBasis);

    StreamSet * ExpandedMarks = P->CreateStreamSet(3);
    SpreadByMask(P, SpreadMask, Filtered_starts, ExpandedMarks);


    StreamSet * FilledBasis = P->CreateStreamSet(8);
    P->CreateKernelCall<StringReplaceKernel>(Header_Vec, ExpandedBasis, SpreadMask, ExpandedMarks, InsertIndex, FilledBasis);

    StreamSet * FilledBytes  = P->CreateStreamSet(1, 8);
    P->CreateKernelCall<P2SKernel>(FilledBasis, FilledBytes);
    P->CreateKernelCall<StdOutKernel>(FilledBytes);
    //cout<<s<<endl;
    
    
    //StreamSet * translatedBasis = P->CreateStreamSet(8);
    //P->CreateKernelCall<CSV_Quote_Translate>(Dquote, BasisBits, translatedBasis);

    // The computed output can be converted back to byte stream form by the
    // P2S kernel (parallel-to-serial).
    //StreamSet * translated = P->CreateStreamSet(1, 8);
    //P->CreateKernelCall<P2SKernel>(translatedBasis, translated);

    //  The StdOut kernel writes a byte stream to standard output.
    //P->CreateKernelCall<StdOutKernel>(translated);

    Scalar * outputFileName = P->getInputScalar("outputFileName");
    P->CreateKernelCall<FileSink>(outputFileName, FilledBytes);

    return reinterpret_cast<CSVTranslateFunctionType>(P->compile());
}

int main(int argc, char *argv[]) {
    //  ParseCommandLineOptions uses the LLVM CommandLine processor, but we also add
    //  standard Parabix command line options such as -help, -ShowPablo and many others.
    codegen::ParseCommandLineOptions(argc, argv, {&CSV_Parsing_Options, pablo::pablo_toolchain_flags(), codegen::codegen_flags()});
    //  A CPU driver is capable of compiling and running Parabix programs on the CPU.
    CPUDriver driver("csv_quote_xlator");
    
    const char delimiter = ',';
    const char * outname;
    int n = Get_Field_Count(delimiter, inputFile.c_str());
    vector<string> header;
    Get_Header(delimiter,inputFile.c_str(), header, n);

    //  Build and compile the Parabix pipeline by calling the Pipeline function above.
    CSVTranslateFunctionType fn = generatePipeline(driver,header);
    //  The compile function "fn"  can now be used.   It takes a file
    //  descriptor as an input, which is specified by the filename given by
    //  the inputFile command line option.
    const int fd = open(inputFile.c_str(), O_RDONLY);
    if (LLVM_UNLIKELY(fd == -1)) {
        llvm::errs() << "Error: cannot open " << inputFile << " for processing. Skipped.\n";
    } else {
        //  Run the pipeline.
        fn(fd,outputFile.c_str());
        close(fd);
    }
    return 0;
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
    
    									//In Windows, \r\n means carriage return and newline; while in Linux, \n does the same job.
#ifdef _WIN32 							// _WIN32 stands for both Windows 32-bit and 64-bit
#else									// If the operating system is not Windows, then \r should be discarded.
		str = str.replace(str.find("\r\n"), 1, "\n");
#endif

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


void Get_Header(const char delimiter, const char* dir, vector<string>& v, int n )
{
    ifstream in(dir);
    string header;
    for(int i=0;i<n-1;i++){
        getline(in,header,delimiter);
        v.push_back(header + ':');
    }
    getline(in,header);
    v.push_back(header + ':');
    cout<<" last header"<<header<<endl;
}