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


typedef void (*main_fcn_T)(char * byte_data, int filesize, const char* filename, uint64_t finalLineUnterminated);

namespace llvm { class raw_ostream; }

class GrepEngine {
public:

    GrepEngine() {};
  
    void grepCodeGen(std::string moduleName, re::RE * re_ast, bool isNameExpression = false);
    
    bool openMMap(const std::string & fileName);
    
    void closeMMap();

    void doGrep();
    
    re::CC *  grepCodepoints();

    ~GrepEngine() {
      delete mEngine;
    }
    
private:
   
    bool finalLineIsUnterminated() const;

    main_fcn_T mMainFcn;
    
    bool mIsNameExpression;
    std::string mFileName;
    size_t mFileSize;
    char * mFileBuffer;
    llvm::ExecutionEngine * mEngine;
};


#endif
