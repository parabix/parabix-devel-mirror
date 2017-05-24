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
#include <re/re_parser.h>  // for 

namespace re { class CC; }
namespace re { class RE; }
namespace llvm { namespace cl { class OptionCategory; } }
class Driver;


namespace grep {

// Regular expression syntax, interpretation and processing.
extern re::RE_Syntax RegexpSyntax;
extern bool IgnoreCaseFlag;
extern bool InvertMatchFlag;
extern bool LineRegexpFlag;
extern bool WordRegexpFlag;

// Grep input sources and interpretation
extern bool RecursiveFlag;
extern bool DereferenceRecursiveFlag;

// Grep output modes and flags.
enum GrepModeType {QuietMode, FilesWithMatch, FilesWithoutMatch, CountOnly, NormalMode};
extern GrepModeType Mode;


const llvm::cl::OptionCategory * grep_regexp_flags();
const llvm::cl::OptionCategory * grep_input_flags();
const llvm::cl::OptionCategory * grep_output_flags();

class GrepEngine {
public:

    GrepEngine();


    ~GrepEngine();

    void grepCodeGen(std::vector<re::RE *> REs, GrepModeType grepMode, bool UTF_16, GrepSource grepSource, GrepType grepType = GrepType::Normal);

    void grepCodeGen_nvptx(std::vector<re::RE *> REs, GrepModeType grepMode, bool UTF_16);

    void doGrep(const std::string & fileName) const;

	uint64_t doGrep(const std::string & fileName, const uint32_t fileIdx) const;

    uint64_t doGrep(const int32_t fileDescriptor, const uint32_t fileIdx) const;
    
    void doGrep(const char * buffer, const uint64_t length, const uint32_t fileIdx) const;

    re::CC * grepCodepoints();

    const std::vector<std::string> & grepPropertyValues(const std::string & propertyName);
    
private:
   
    Driver * mGrepDriver;
};


re::CC * getParsedCodePointSet();
void setParsedCodePointSet();

void setParsedPropertyValues();

void initFileResult(std::vector<std::string> filenames);
void PrintResult(GrepModeType grepMode, std::vector<size_t> & total_CountOnly);
}

#endif
