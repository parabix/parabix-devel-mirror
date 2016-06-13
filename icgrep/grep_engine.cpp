/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <grep_engine.h>
#include <IDISA/idisa_builder.h>
#include <IDISA/idisa_target.h>
#include <llvm/Support/CommandLine.h>
#include <re/re_toolchain.h>
#include <re/re_cc.h>

#include <pablo/pablo_toolchain.h>
#include <toolchain.h>
#include <utf_encoding.h>
#include <pablo/pablo_compiler.h>
#include <kernels/pipeline.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/Type.h>
#include <llvm/IR/Module.h>
#include <llvm/ExecutionEngine/MCJIT.h>
#include <llvm/IRReader/IRReader.h>
#include <llvm/Support/Debug.h>
#include <llvm/IR/Verifier.h>
#include <UCD/UnicodeNameData.h>

#include <fstream>
#include <sstream>
#include <iostream>
#include <string>
#include <stdint.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdexcept>
#include <cctype>


#include <llvm/Support/raw_os_ostream.h>

// mmap system
#include <boost/filesystem.hpp>
#include <boost/iostreams/device/mapped_file.hpp>
using namespace boost::iostreams;
using namespace boost::filesystem;

#include <fcntl.h>

#include <kernels/kernel.h>

static cl::OptionCategory bGrepOutputOptions("Output Options",
                                             "These options control the output.");

static cl::opt<bool> NormalizeLineBreaks("normalize-line-breaks", cl::desc("Normalize line breaks to std::endl."), cl::init(false),  cl::cat(bGrepOutputOptions));

static cl::opt<bool> ShowFileNames("H", cl::desc("Show the file name with each matching line."), cl::cat(bGrepOutputOptions));
static cl::alias ShowFileNamesLong("with-filename", cl::desc("Alias for -H"), cl::aliasopt(ShowFileNames));

static cl::opt<bool> ShowLineNumbers("n", cl::desc("Show the line number with each matching line."), cl::cat(bGrepOutputOptions));
static cl::alias ShowLineNumbersLong("line-number", cl::desc("Alias for -n"), cl::aliasopt(ShowLineNumbers));


bool isUTF_16 = false;

void GrepEngine::doGrep(const std::string & fileName, const int fileIdx, bool CountOnly, std::vector<uint64_t> & total_CountOnly, bool UTF_16) {
    path file(fileName);
    if (exists(file)) {
        if (is_directory(file)) {
            return;
        }
    } else {
        std::cerr << "Error: cannot open " << fileName << " for processing. Skipped.\n";
        return;
    }

    const auto fileSize = file_size(file);
    if (fileSize > 0) {
        try {
            mapped_file_source source(fileName, fileSize, 0);
            char * fileBuffer = const_cast<char *>(source.data());
            if (CountOnly) {
                total_CountOnly[fileIdx] = mGrepFunction_CountOnly(fileBuffer, fileSize, fileIdx);
            } else {
                mGrepFunction(fileBuffer, fileSize, fileIdx);
            }
            source.close();
        } catch (std::exception & e) {
            throw std::runtime_error("Boost mmap error: " + fileName + ": " + e.what());
        }
    } else {
        if (CountOnly) {
            mGrepFunction_CountOnly(nullptr, 0, fileIdx);
        } else {
            mGrepFunction(nullptr, 0, fileIdx);
        }
    }
}


void GrepEngine::grepCodeGen(std::string moduleName, re::RE * re_ast, bool CountOnly, bool UTF_16, bool isNameExpression) {
    isUTF_16 = UTF_16; 
    Module * M = new Module(moduleName, getGlobalContext());
    
    IDISA::IDISA_Builder * idb = IDISA::GetIDISA_Builder(M);

    kernel::PipelineBuilder pipelineBuilder(M, idb);

    Encoding::Type type;
    type = UTF_16 ? Encoding::Type::UTF_16 : Encoding::Type::UTF_8;
    unsigned bits;
    bits = UTF_16 ? 16 : 8;

    Encoding encoding(type, bits);

    mIsNameExpression = isNameExpression;
    re_ast = re::regular_expression_passes(encoding, re_ast);   
    pablo::PabloFunction * function = re::re2pablo_compiler(encoding, re_ast);
    

    pipelineBuilder.CreateKernels(function, UTF_16, isNameExpression);
    std::cerr << "CreateKernels complete\n";

    llvm::Function * grepIR = pipelineBuilder.ExecuteKernels(CountOnly, UTF_16);
    std::cerr << "ExecuteKernels complete\n";

    mEngine = JIT_to_ExecutionEngine(M);
    ApplyObjectCache(mEngine);
    icgrep_Linking(M, mEngine);
    std::cerr << "icgrep_Linking complete\n";

    //#ifndef NDEBUG
    verifyModule(*M, &dbgs());
    //#endif

    mEngine->finalizeObject();
    std::cerr << "finalizeObject complete\n";
    delete idb;

    if (CountOnly) {
        mGrepFunction_CountOnly = reinterpret_cast<GrepFunctionType_CountOnly>(mEngine->getPointerToFunction(grepIR));
    } else {
        mGrepFunction = reinterpret_cast<GrepFunctionType>(mEngine->getPointerToFunction(grepIR));
    }

}



re::CC *  GrepEngine::grepCodepoints() {

    setParsedCodePointSet();
    char * mFileBuffer = getUnicodeNameDataPtr();
    size_t mFileSize = getUnicodeNameDataSize();
    std::string mFileName = "Uname.txt";

    mGrepFunction(mFileBuffer, mFileSize, 0);

    return getParsedCodePointSet();
}

GrepEngine::~GrepEngine() {
    delete mEngine;
}


static int * total_count;
static std::stringstream * resultStrs = nullptr;
static std::vector<std::string> inputFiles;

void initResult(std::vector<std::string> filenames){
    const int n = filenames.size();
    if (n > 1) {
        ShowFileNames = true;
    }
    inputFiles = filenames;
    resultStrs = new std::stringstream[n];
    total_count = new int[n];
    for (unsigned i = 0; i < inputFiles.size(); ++i){
        total_count[i] = 0;
    }
    
}

extern "C" {
    void wrapped_report_match(uint64_t lineNum, uint64_t line_start, uint64_t line_end, const char * buffer, uint64_t filesize, int fileIdx) {
        int index = isUTF_16 ? 2 : 1;
        int idx = fileIdx;
          
        if (ShowFileNames) {
            resultStrs[idx] << inputFiles[idx] << ':';
        }
        if (ShowLineNumbers) {
            resultStrs[idx] << lineNum << ":";
        }
        
        if ((!isUTF_16 && buffer[line_start] == 0xA) && (line_start != line_end)) {
            // The line "starts" on the LF of a CRLF.  Really the end of the last line.
            line_start++;
        }
        if (((isUTF_16 && buffer[line_start] == 0x0) && buffer[line_start + 1] == 0xA) && (line_start != line_end)) {
            // The line "starts" on the LF of a CRLF.  Really the end of the last line.
            line_start += 2;
        }
        if (line_end == filesize) {
            // The match position is at end-of-file.   We have a final unterminated line.
            resultStrs[idx].write(&buffer[line_start * index], (line_end - line_start) * index);
            if (NormalizeLineBreaks) {
                resultStrs[idx] << '\n';  // terminate it
            }
            return;
        }
        unsigned char end_byte = (unsigned char)buffer[line_end]; 
        unsigned char penult_byte = (unsigned char)(buffer[line_end - 1]);
        if (NormalizeLineBreaks) {
            if (end_byte == 0x85) {
                // Line terminated with NEL, on the second byte.  Back up 1.
                line_end--;
            } else if (end_byte > 0xD) {
                // Line terminated with PS or LS, on the third byte.  Back up 2.
                isUTF_16 ? line_end-- : line_end -= 2;
            }
            resultStrs[idx].write(&buffer[line_start * index], (line_end - line_start) * index);
            resultStrs[idx] << '\n';
        }
        else {   
            if ((!isUTF_16 && end_byte == 0x0D) || (isUTF_16 && (end_byte == 0x0D && penult_byte == 0x0))) {
                // Check for line_end on first byte of CRLF;  note that we don't
                // want to access past the end of buffer.
                if (line_end + 1 < filesize) {
                    if (!isUTF_16 && buffer[line_end + 1] == 0x0A) {
                        // Found CRLF; preserve both bytes.
                        line_end++;
                    }
                    if (isUTF_16 && buffer[line_end + 1] == 0x0 && buffer[line_end + 2] == 0x0A) {
                        // Found CRLF; preserve both bytes.
                        line_end += 2;
                    }
                }
            }
            resultStrs[idx].write(&buffer[line_start * index], (line_end - line_start + 1) * index);
        }
    }
}

void PrintResult(bool CountOnly, std::vector<uint64_t> & total_CountOnly){
    if(CountOnly){
        if (!ShowFileNames) {
            for (unsigned i = 0; i < inputFiles.size(); ++i){
                std::cout << total_CountOnly[i] << std::endl;
            }
        }
        else {
            for (unsigned i = 0; i < inputFiles.size(); ++i){
                std::cout << inputFiles[i] << ':' << total_CountOnly[i] << std::endl;
            };
        }
        return;
    }
    
    for (unsigned i = 0; i < inputFiles.size(); ++i){
        std::cout << resultStrs[i].str();
    }
}

re::CC * parsedCodePointSet;

extern "C" {
    void insert_codepoints(uint64_t lineNum, uint64_t line_start, uint64_t line_end, const char * buffer) {
        re::codepoint_t c = 0;
        ssize_t line_pos = line_start;
        while (isxdigit(buffer[line_pos])) {
            if (isdigit(buffer[line_pos])) {
                c = (c << 4) | (buffer[line_pos] - '0');
            }
            else {
                c = (c << 4) | (tolower(buffer[line_pos]) - 'a' + 10);
            }
            line_pos++;
        }
        assert(((line_pos - line_start) >= 4) && ((line_pos - line_start) <= 6)); // UCD format 4 to 6 hex digits.       
        parsedCodePointSet->insert(c);
    }
}

void setParsedCodePointSet(){
    parsedCodePointSet = re::makeCC();
}

re::CC * getParsedCodePointSet(){
    return parsedCodePointSet;
}




void icgrep_Linking(Module * m, ExecutionEngine * e) {
    Module::FunctionListType & fns = m->getFunctionList();
    for (Module::FunctionListType::iterator it = fns.begin(), it_end = fns.end(); it != it_end; ++it) {
        std::string fnName = it->getName().str();
        if (fnName == "s2p_block") continue;
        if (fnName == "process_block") continue;
        if (fnName == "process_block_initialize_carries") continue;
        
        if (fnName == "wrapped_report_match") {
            e->addGlobalMapping(cast<GlobalValue>(it), (void *)&wrapped_report_match);
        }
        if (fnName == "insert_codepoints") {
            e->addGlobalMapping(cast<GlobalValue>(it), (void *)&insert_codepoints);
        }
#ifndef DISABLE_PREGENERATED_UCD_FUNCTIONS
        else {
            const UCD::ExternalProperty & ep = UCD::resolveExternalProperty(fnName);
            e->addGlobalMapping(cast<GlobalValue>(it), std::get<0>(ep));
        }
#endif
    }
}

