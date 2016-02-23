#ifndef DO_GREP_H
#define DO_GREP_H
/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <string>
#include <stdint.h>
#include <re/re_cc.h>


typedef void (*main_fcn_T)(char * byte_data, int filesize, const char* filename, uint64_t finalLineUnterminated);

namespace llvm { class raw_ostream; }

class GrepExecutor {
public:

    GrepExecutor(void * main_fnptr)
    : mMainFcn(reinterpret_cast<main_fcn_T>(main_fnptr)) {
        
    }
  
    void doGrep(const std::string & fileName);
private:
   
    bool finalLineIsUnterminated() const;

    main_fcn_T mMainFcn;
    
    std::string mFileName;
    size_t mFileSize;
    char * mFileBuffer;
};


#endif
