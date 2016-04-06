#ifndef DO_GREP_H
#define DO_GREP_H
/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <string>
#include <stdint.h>
#include <re/re_re.h>
#include <llvm/ExecutionEngine/ExecutionEngine.h>


namespace llvm { class raw_ostream; }

class GrepEngine {
    typedef void (*GrepFunctionType)(char * byte_data, size_t filesize, const char* filename, uint64_t finalLineUnterminated);
public:

    GrepEngine() {}

    ~GrepEngine();
  
    void grepCodeGen(std::string moduleName, re::RE * re_ast, bool isNameExpression = false);
    
    void doGrep(const std::string & fileName);
    
    re::CC *  grepCodepoints();
    
private:
   
    static bool finalLineIsUnterminated(const char * const fileBuffer, const size_t fileSize);

    GrepFunctionType mGrepFunction;
    
    bool mIsNameExpression;
    llvm::ExecutionEngine * mEngine;
};


#endif
