/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <grep_engine.h>
#include <IDISA/idisa_builder.h>
#include <IDISA/idisa_target.h>
#include <re/re_toolchain.h>
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



bool GrepEngine::finalLineIsUnterminated(const char * const fileBuffer, const size_t fileSize) {
    if (fileSize == 0) return false;
    unsigned char end_byte = static_cast<unsigned char>(fileBuffer[fileSize-1]);
    // LF through CR are line break characters
    if ((end_byte >= 0xA) && (end_byte <= 0xD)) return false;
    // Other line breaks require at least two bytes.
    if (fileSize == 1) return true;
    // NEL
    unsigned char penult_byte = static_cast<unsigned char>(fileBuffer[fileSize-2]);
    if ((end_byte == 0x85) && (penult_byte == 0xC2)) return false;
    if (fileSize == 2) return true;
    // LS and PS
    if ((end_byte < 0xA8) || (end_byte > 0xA9)) return true;
    return (static_cast<unsigned char>(fileBuffer[fileSize-3]) != 0xE2) || (penult_byte != 0x80);
}

void GrepEngine::doGrep(const std::string & fileName, const int fileIdx) {
    const path file(fileName);
    if (exists(file)) {
        if (is_directory(file)) {
            return;
        }
    } else {
        std::cerr << "Error: cannot open " << fileName << " for processing. Skipped.\n";
        return;
    }

    const size_t fileSize = file_size(file);
    if (fileSize > 0) {
        mapped_file_source file;
        try {
            file.open(fileName);
        } catch (std::exception &e) {
            throw std::runtime_error("Boost mmap error: " + fileName + ": " + e.what());
        }
        char * fileBuffer = const_cast<char *>(file.data());
        mGrepFunction(fileBuffer, fileSize, fileIdx, finalLineIsUnterminated(fileBuffer, fileSize));
        file.close();
    }
    else {
        mGrepFunction(nullptr, 0, fileIdx, false);
    }
}


void GrepEngine::grepCodeGen(std::string moduleName, re::RE * re_ast, bool isNameExpression) {
                            
    Module * M = new Module(moduleName, getGlobalContext());
    
    IDISA::IDISA_Builder * idb = GetIDISA_Builder(M);

    kernel::PipelineBuilder pipelineBuilder(M, idb);

    Encoding encoding(Encoding::Type::UTF_8, 8);
    mIsNameExpression = isNameExpression;
    re_ast = regular_expression_passes(encoding, re_ast);   
    pablo::PabloFunction * function = re2pablo_compiler(encoding, re_ast);
    

    pipelineBuilder.CreateKernels(function, isNameExpression);

    llvm::Function * grepIR = pipelineBuilder.ExecuteKernels();

    mEngine = JIT_to_ExecutionEngine(M);
    
    icgrep_Linking(M, mEngine);
    #ifndef NDEBUG
    verifyModule(*M, &dbgs());
    #endif
    mEngine->finalizeObject();
    delete idb;

    mGrepFunction = reinterpret_cast<GrepFunctionType>(mEngine->getPointerToFunction(grepIR));
}

re::CC *  GrepEngine::grepCodepoints() {

    setParsedCodePointSet();
    char * mFileBuffer = getUnicodeNameDataPtr();
    size_t mFileSize = getUnicodeNameDataSize();
    std::string mFileName = "Uname.txt";

    uint64_t finalLineUnterminated = 0;
    if(finalLineIsUnterminated(mFileBuffer, mFileSize))
        finalLineUnterminated = 1;    
    mGrepFunction(mFileBuffer, mFileSize, 0, finalLineUnterminated);

    return getParsedCodePointSet();
}

GrepEngine::~GrepEngine() {
    delete mEngine;
}
