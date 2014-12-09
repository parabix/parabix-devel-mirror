/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <string>

#include "icgrep.h"
#include "utf_encoding.h"
#include "compiler.h"
#include "pablo/pablo_compiler.h"
#include "do_grep.h"

#include "llvm/Support/CommandLine.h"

int main(int argc, char *argv[]) {
    static cl::opt<std::string> regexp1(cl::Positional, cl::Required, cl::desc("<regexp>"));
    static cl::list<std::string> inputFiles(cl::Positional, cl::desc("<input file ...>"), cl::OneOrMore);
    
    static cl::opt<bool>CountOnly("c", cl::desc("Count and display the matching lines per file only."));
    cl::alias CountOnlyLong("-count", cl::desc("Alias for -c"), cl::aliasopt(CountOnly));
    
    static cl::opt<bool>ShowFileNames("H", cl::desc("Show the file name with each matching line."));
    cl::alias ShowFileNamesLong("-with-filename", cl::desc("Alias for -H"), cl::aliasopt(ShowFileNames));
    
    static cl::opt<bool>ShowLineNumbers("n", cl::desc("Show the line number with each matching line."));
    cl::alias ShowLineNumbersLong("-line-number", cl::desc("Alias for -n"), cl::aliasopt(ShowLineNumbers));
    
    //cl::opt<std::string> RegexFilename("f", cl::desc("Take regular expressions (one per line) from a file"), cl::value_desc("<filename>"));
  
    // Does -b mean the byte offset of the line, or the byte offset of the match start within the line?
    //static cl::opt<bool>ShowByteOffsets("b", cl::desc("Show the byte offset with each matching line."));
    //cl::alias ShowByteOffsetsLong("-byte-offset", cl::desc("Alias for -b"), cl::aliasopt(ShowByteOffsets));

    Encoding encoding(Encoding::Type::UTF_8, 8);

    cl::ParseCommandLineOptions(argc, argv);

    const auto llvm_codegen = icgrep::compile(encoding, regexp1, false, false);

    if (llvm_codegen.process_block_fptr != 0) {
        void (*FP)(const Basis_bits &basis_bits, BitBlock carry_q[], BitBlock advance_q[], Output &output) = 
           (void (*)(const Basis_bits &basis_bits, BitBlock carry_q[], BitBlock advance_q[], Output &output))(void*)llvm_codegen.process_block_fptr;
        GrepExecutor grepEngine = GrepExecutor(llvm_codegen.carry_q_size, llvm_codegen.advance_q_size, FP);
        grepEngine.setCountOnlyOption(CountOnly);
        grepEngine.setShowLineNumberOption(ShowLineNumbers);
	if (inputFiles.size() > 1 || ShowFileNames) grepEngine.setShowFileNameOption();
        for (unsigned i = 0; i != inputFiles.size(); ++i) {
            grepEngine.doGrep(inputFiles[i].c_str());
        }
    }
    
    return 0;
}

