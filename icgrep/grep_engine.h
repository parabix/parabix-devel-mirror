/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */
#ifndef GREP_ENGINE_H
#define GREP_ENGINE_H
#include <grep_interface.h>
#include <grep_type.h>  // for GrepType, GrepType::Normal
#include <string>       // for string
#include <vector>
#include <re/re_parser.h>
#include <re/re_multiplex.h>

namespace re { class CC; }
namespace re { class RE; }
namespace llvm { namespace cl { class OptionCategory; } }
class Driver;


namespace grep {

// Thread function only.
void *DoGrepThreadFunction(void *args);
    
class GrepEngine {
public:

    GrepEngine();


    ~GrepEngine();

    void grepCodeGen(std::vector<re::RE *> REs, GrepModeType grepMode, bool UTF_16, GrepSource grepSource);

    void grepCodeGen_nvptx(std::vector<re::RE *> REs, GrepModeType grepMode, bool UTF_16);

    void doGrep(const std::string & fileName, std::string & PTXFilename) const;
    
    uint64_t doGrep(const std::string & fileName, const uint32_t fileIdx) const;

    uint64_t doGrep(const int32_t fileDescriptor, const uint32_t fileIdx) const;
        
private:
   
    Driver * mGrepDriver;
};

void initFileResult(std::vector<std::string> filenames);
    
void PrintResults();
    
}

#endif
