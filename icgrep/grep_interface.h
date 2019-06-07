/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 *
 *  This file defines all the options, parameters and exit codes for icgrep as a 
 *  command line utility.
 *
 */
#ifndef GREP_INTERFACE_H
#define GREP_INTERFACE_H
 
#include <string>       // for string
#include <vector>
#include <re/parsers/parser.h>  // for re::RE_Syntax

namespace re { class CC; }
namespace re { class RE; }
namespace llvm { namespace cl { class OptionCategory; } }


namespace argv {
    

void InitializeCommandLineInterface(int argc, char *argv[]);
    
/*
 *  A.  Regular expression syntax, interpretation and processing.
 */
 
// The syntax specified with =E, -F, -G, or -PROSITE. 
extern re::RE_Syntax RegexpSyntax;
    
// Regular expression interpretation corresponding to -i, -v, -w, -x flags.
extern bool IgnoreCaseFlag; // -i
extern bool InvertMatchFlag; // -v
extern bool LineRegexpFlag; // -x
extern bool WordRegexpFlag; // -w
extern std::vector<std::string> RegexpVector; // -e
extern std::string FileFlag; // -f

/*
 *  B.  Grep input options.
 */

// Use DirectoriesFlag==Recurse to test for recursive mode.
//extern bool TextFlag; // -a
//extern bool BinaryFlag; // -U
enum BinaryFilesMode {Binary, WithoutMatch, Text};
extern BinaryFilesMode BinaryFilesFlag;
    
extern bool NullDataFlag; // -z
extern bool UnicodeLinesFlag; // -Unicode-lines


/*
 *  C.  Grep output modes and options.
 */

//
//  Grep abbreviated output modes corresponding to -q, -l, -L, -c flags, or normal
//  matched line output mode (no abbreviated mode flag specified).
//
enum GrepModeType {QuietMode, FilesWithMatch, FilesWithoutMatch, CountOnly, NormalMode};
extern GrepModeType Mode;

enum ColoringType {alwaysColor, autoColor, neverColor};
extern ColoringType ColorFlag;

extern bool WithFilenameFlag; // -H
extern bool NoFilenameFlag; // -h
extern bool NullFlag; // -Z
extern bool LineNumberFlag; // -n
extern bool ByteOffsetFlag; // -b
extern bool UnixByteOffsetsFlag; // -u
extern bool InitialTabFlag; // -T
extern bool OnlyMatchingFlag; // -o
extern std::string LabelFlag; // -label
extern bool LineBufferedFlag; // -line-buffered
extern int AfterContext; // -A or -C
extern int BeforeContext; // -B or -C
extern int MaxCountFlag; // -m  (overridden and set to 1 with -q, -l, -L modes)
    

//
// icgrep exit codes are consistent with POSIX specifications.
enum ExitCode {
    MatchFoundExitCode = 0,       // At least one match was found.
    MatchNotFoundExitCode = 1,    // No matches were found.
    InternalFailureCode = 2,      // Fatal error code due to program logic or system problem.
    UsageErrorCode = 3            // Use of unsupported regexp syntax or error in command line parameters.
};

}

#endif
