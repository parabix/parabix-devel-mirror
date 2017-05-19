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
public:

    GrepEngine();

    void grepCodeGen(const std::string & moduleName, std::vector<re::RE *> REs, bool CountOnly, bool UTF_16, GrepSource grepSource, GrepType grepType = GrepType::Normal);

    void grepCodeGen_nvptx(const std::string & moduleName, std::vector<re::RE *> REs, bool CountOnly, bool UTF_16);

    void doGrep(const std::string & fileName) const;

	uint64_t doGrep(const std::string & fileName, const uint32_t fileIdx) const;

    uint64_t doGrep(const int32_t fileDescriptor, const uint32_t fileIdx) const;
    
    void doGrep(const char * buffer, const uint64_t length, const uint32_t fileIdx) const;

    re::CC * grepCodepoints();

    const std::vector<std::string> & grepPropertyValues(const std::string & propertyName);
    
private:
   
    void * mGrepFunction;
};


re::CC * getParsedCodePointSet();
void setParsedCodePointSet();

void setParsedPropertyValues();

void initFileResult(std::vector<std::string> filenames);
void PrintResult(bool CountOnly, std::vector<size_t> & total_CountOnly);

#endif
