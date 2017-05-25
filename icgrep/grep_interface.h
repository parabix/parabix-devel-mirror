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
#include <re/re_parser.h>  // for re::RE_Syntax

namespace re { class CC; }
namespace re { class RE; }
namespace llvm { namespace cl { class OptionCategory; } }


namespace grep {
    

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
    
/*
 *  B.  Grep input sources and interpretation.
 */
    
    
// Grep input source control corresponding to -r, -R flags.
extern bool RecursiveFlag; // -r
extern bool DereferenceRecursiveFlag; // -R
extern bool TextFlag; // -a
extern bool BinaryFlag; // -U
extern bool NullDataFlag; // -z
extern bool MmapFlag; // -mmap
    
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
    
extern bool NoMessagesFlag; // -s
extern bool WithFilenameFlag; // -H
extern bool NoFilenameFlag; // -h
extern bool NullFlag; // -Z
extern bool LineNumberFlag; // -n
extern bool ByteOffsetFlag; // -b
extern bool UnixByteOffsetsFlag; // -u
extern bool InitialTabFlag; // -T
extern bool OnlyMatchingFlag; // -o
extern bool LineBufferedFlag; // -line-buffered
extern bool NormalizeLineBreaksFlag; // -normalize-line-breaks
extern int AfterContextFlag; // -A
extern int BeforeContextFlag; // -B
extern int ContextFlag; // -C
extern int MaxCountFlag; // -m
    

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
