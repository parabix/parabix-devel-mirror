/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <grep_engine.h>
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
#ifdef USE_BOOST_MMAP
#include <boost/filesystem.hpp>
#include <boost/iostreams/device/mapped_file.hpp>
using namespace boost::iostreams;
using namespace boost::filesystem;
#else
#include <sys/mman.h>
#endif
#include <fcntl.h>

#include <kernels/kernel.h>



bool GrepEngine::finalLineIsUnterminated() const {
    if (mFileSize == 0) return false;
    unsigned char end_byte = static_cast<unsigned char>(mFileBuffer[mFileSize-1]);
    // LF through CR are line break characters
    if ((end_byte >= 0xA) && (end_byte <= 0xD)) return false;
    // Other line breaks require at least two bytes.
    if (mFileSize == 1) return true;
    // NEL
    unsigned char penult_byte = static_cast<unsigned char>(mFileBuffer[mFileSize-2]);
    if ((end_byte == 0x85) && (penult_byte == 0xC2)) return false;
    if (mFileSize == 2) return true;
    // LS and PS
    if ((end_byte < 0xA8) || (end_byte > 0xA9)) return true;
    return (static_cast<unsigned char>(mFileBuffer[mFileSize-3]) != 0xE2) || (penult_byte != 0x80);
}

bool GrepEngine::openMMap(const std::string & fileName) {

    mFileName = fileName;

#ifdef USE_BOOST_MMAP
    const path file(mFileName);
    if (exists(file)) {
        if (is_directory(file)) {
            return false;
        }
    } else {
        std::cerr << "Error: cannot open " << mFileName << " for processing. Skipped.\n";
        return false;
    }

    mFileSize = file_size(file);
    mapped_file mFile;
    if (mFileSize == 0) {
        mFileBuffer = nullptr;
    }
    else {
        try {
            mFile.open(mFileName, mapped_file::priv, mFileSize, 0);
        } catch (std::ios_base::failure e) {
            std::cerr << "Error: Boost mmap of " << mFileName << ": " << e.what() << std::endl;
            return false;
        }
        mFileBuffer = mFile.data();
    }
#else
    struct stat infile_sb;
    const int fdSrc = open(mFileName.c_str(), O_RDONLY);
    if (fdSrc == -1) {
        std::cerr << "Error: cannot open " << mFileName << " for processing. Skipped.\n";
        return false;
    }
    if (fstat(fdSrc, &infile_sb) == -1) {
        std::cerr << "Error: cannot stat " << mFileName << " for processing. Skipped.\n";
        close (fdSrc);
        return false;
    }
    if (S_ISDIR(infile_sb.st_mode)) {
        close (fdSrc);
        return false;
    }
    mFileSize = infile_sb.st_size;
    if (mFileSize == 0) {
        mFileBuffer = nullptr;
    }
    else {
        mFileBuffer = (char *) mmap(NULL, mFileSize, PROT_READ, MAP_PRIVATE, fdSrc, 0);
        if (mFileBuffer == MAP_FAILED) {
            if (errno ==  ENOMEM) {
                std::cerr << "Error:  mmap of " << mFileName << " failed: out of memory\n";
                close (fdSrc);
            }
            else {
                std::cerr << "Error: mmap of " << mFileName << " failed with errno " << errno << ". Skipped.\n";
                close (fdSrc);
            }
            return false;
        }
    }
    close(fdSrc);

#endif
    return true;  // success
}

void GrepEngine::closeMMap() {
#ifdef USE_BOOST_MMAP
    mFile.close();
#else
    munmap((void *)mFileBuffer, mFileSize);
#endif   

}

void GrepEngine::doGrep() {
        
    llvm::raw_os_ostream out(std::cout);
    
    uint64_t finalLineUnterminated = 0;
    if(finalLineIsUnterminated())
        finalLineUnterminated = 1;
    
    mMainFcn(mFileBuffer, mFileSize, mFileName.c_str(), finalLineUnterminated);
    
    if (!mIsNameExpression) PrintTotalCount();
}

void GrepEngine::grepCodeGen(std::string moduleName, re::RE * re_ast, bool isNameExpression) {
                            
    Module * M = new Module("moduleName", getGlobalContext());
    
    IDISA::IDISA_Builder * idb = GetNativeIDISA_Builder(M, VectorType::get(IntegerType::get(getGlobalContext(), 64), BLOCK_SIZE/64));

    PipelineBuilder pipelineBuilder(M, idb);

    Encoding encoding(Encoding::Type::UTF_8, 8);
    mIsNameExpression = isNameExpression;
    re_ast = regular_expression_passes(encoding, re_ast);   
    pablo::PabloFunction * function = re2pablo_compiler(encoding, re_ast);

    pipelineBuilder.CreateKernels(function, isNameExpression);

    pipelineBuilder.ExecuteKernels();

    llvm::Function * main_IR = M->getFunction("Main");
    mEngine = JIT_to_ExecutionEngine(M);
    
    icgrep_Linking(M, mEngine);
    verifyModule(*M, &dbgs());
    mEngine->finalizeObject();
    delete idb;

    mMainFcn = (main_fcn_T) mEngine->getPointerToFunction(main_IR);
}


re::CC *  GrepEngine::grepCodepoints() {
    setParsedCodePointSet();
    mFileBuffer = getUnicodeNameDataPtr();
    mFileSize = getUnicodeNameDataSize();
    mFileName = "Uname.txt";
    doGrep();
    return getParsedCodePointSet();
}
