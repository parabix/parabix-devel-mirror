/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */
#ifndef DO_GREP_H
#define DO_GREP_H
#include <grep_type.h>  // for GrepType, GrepType::Normal
#include <string>       // for string
#include <vector>
namespace llvm { class ExecutionEngine; }
namespace llvm { class Module; }
namespace re { class CC; }
namespace re { class RE; }

class GrepEngine {
    typedef void (*GrepFunctionType)(char * byte_data, size_t filesize, const int fileIdx);
    typedef uint64_t (*GrepFunctionType_CountOnly)(char * byte_data, size_t filesize, const int fileIdx);
    typedef void (*GrepFunctionType_CPU)(char * rslt, char * LineBreak, char * byte_data, size_t filesize, const int fileIdx);
public:

    GrepEngine();
    ~GrepEngine();
  
    void grepCodeGen(std::string moduleName, re::RE * re_ast, bool CountOnly, bool UTF_16 = false, GrepType grepType = GrepType::Normal, const bool usingStdIn = false);
    void multiGrepCodeGen(std::string moduleName, std::vector<re::RE *> REs, bool CountOnly, bool UTF_16 = false, GrepType grepType = GrepType::Normal);
     
    void doGrep(const std::string & fileName, const int fileIdx, bool CountOnly, std::vector<size_t> & total_CountOnly);

    void doGrep(const int fileIdx, bool CountOnly, std::vector<size_t> & total_CountOnly);
    
    re::CC *  grepCodepoints();

    const std::vector<std::string> & grepPropertyValues(const std::string & propertyName);
    
private:
   
    GrepFunctionType mGrepFunction;
    GrepFunctionType_CountOnly mGrepFunction_CountOnly;
    GrepFunctionType_CPU mGrepFunction_CPU;

    GrepType mGrepType;
    llvm::ExecutionEngine * mEngine;
};

void icgrep_Linking(llvm::Module * m, llvm::ExecutionEngine * e);


re::CC * getParsedCodePointSet();
void setParsedCodePointSet();

void setParsedPropertyValues();

void initFileResult(std::vector<std::string> filenames);
void PrintResult(bool CountOnly, std::vector<size_t> & total_CountOnly);

#endif
