/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <string>
#include <iostream>
#include <fstream>

#include "icgrep.h"
#include "utf_encoding.h"
#include "compiler.h"
#include "pablo/pablo_compiler.h"
#include "do_grep.h"

#include "llvm/Support/CommandLine.h"

int main(int argc, char *argv[]) {
    int firstInputFile = 1;  // Normal case when first positional arg is a regex.  
    cl::list<std::string> inputFiles(cl::Positional, cl::desc("<regex> <input file ...>"), cl::OneOrMore);
    
    cl::opt<bool> CountOnly("c", cl::desc("Count and display the matching lines per file only."));
    cl::alias CountOnlyLong("-count", cl::desc("Alias for -c"), cl::aliasopt(CountOnly));
    
    cl::opt<bool> ShowFileNames("H", cl::desc("Show the file name with each matching line."));
    cl::alias ShowFileNamesLong("-with-filename", cl::desc("Alias for -H"), cl::aliasopt(ShowFileNames));
    
    cl::opt<bool> ShowLineNumbers("n", cl::desc("Show the line number with each matching line."));
    cl::alias ShowLineNumbersLong("-line-number", cl::desc("Alias for -n"), cl::aliasopt(ShowLineNumbers));
    
    cl::list<std::string> regexVector("e", cl::desc("Regular exprssion"), cl::ZeroOrMore);
    
    cl::opt<std::string> RegexFilename("f", cl::desc("Take regular expressions (one per line) from a file"), cl::value_desc("regex file"), cl::init(""));
  
    // Does -b mean the byte offset of the line, or the byte offset of the match start within the line?
    //static cl::opt<bool>ShowByteOffsets("b", cl::desc("Show the byte offset with each matching line."));
    //cl::alias ShowByteOffsetsLong("-byte-offset", cl::desc("Alias for -b"), cl::aliasopt(ShowByteOffsets));

    Encoding encoding(Encoding::Type::UTF_8, 8);

    cl::ParseCommandLineOptions(argc, argv);
    
    //std::vector<std::string> regexVector;
    if (RegexFilename != "") {
        std::ifstream regexFile(RegexFilename.c_str());
        std::string r;
        if (regexFile.is_open()) {
            while (std::getline(regexFile, r)) {
                regexVector.push_back(r);
            }
            regexFile.close();
        }
    }
    
    // if there are no regexes specified through -e or -f, the first positional argument
    // must be a regex, not an input file.
    
    if (regexVector.size() == 0) {
        regexVector.push_back(inputFiles[0]);
        firstInputFile = 1;
    }
    else {
        firstInputFile = 0;
    }
    
    const auto llvm_codegen = icgrep::compile(encoding, regexVector, false);

    if (llvm_codegen.process_block_fptr != 0) {
        void (*FP)(const Basis_bits &basis_bits, BitBlock carry_q[], BitBlock advance_q[], Output &output) = 
           (void (*)(const Basis_bits &basis_bits, BitBlock carry_q[], BitBlock advance_q[], Output &output))(void*)llvm_codegen.process_block_fptr;
        GrepExecutor grepEngine = GrepExecutor(llvm_codegen.carry_q_size, llvm_codegen.advance_q_size, FP);
        grepEngine.setCountOnlyOption(CountOnly);
        grepEngine.setShowLineNumberOption(ShowLineNumbers);
	    if (inputFiles.size() > (firstInputFile + 1) || ShowFileNames) {
            grepEngine.setShowFileNameOption();
        }
        for (unsigned i = firstInputFile; i != inputFiles.size(); ++i) {
            grepEngine.doGrep(inputFiles[i].c_str());
        }
    }
    
    return 0;
}

