/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "grep_interface.h"

#include <llvm/Support/CommandLine.h>
#include <llvm/Support/ErrorHandling.h>
#include <llvm/Support/Signals.h>
#include <llvm/Support/raw_ostream.h>
#include <fileselect/file_select.h>
#include <toolchain/toolchain.h>
#include <re/parse/parser.h>
#include <re/adt/re_alt.h>
#include <re/toolchain/toolchain.h>
#include <fstream>
#include <string>

#include <pablo/pablo_toolchain.h>

using namespace llvm;

namespace argv {

/*
 *  A.  Regular expression syntax, interpretation and processing.
 */

static cl::OptionCategory RE_Options("A. Regular Expression Interpretation", "These options control regular expression parsing and interpretation");

re::RE_Syntax RegexpSyntax;
static cl::opt<re::RE_Syntax, true> RegexpSyntaxOption(cl::desc("Regular expression syntax: (default PCRE)"),
    cl::values(
        clEnumValN(re::RE_Syntax::ERE, "E", "Posix extended regular expression (ERE) syntax"),
        clEnumValN(re::RE_Syntax::FixedStrings, "F", "Fixed strings, separated by newlines"),
        clEnumValN(re::RE_Syntax::BRE, "G", "Posix basic regular expression (BRE) syntax"),
        clEnumValN(re::RE_Syntax::PCRE, "P", "Perl-compatible regular expression (PCRE) syntax"),
        clEnumValN(re::RE_Syntax::ERE, "extended-regexp", "Alias for -E"),
        clEnumValN(re::RE_Syntax::FixedStrings, "fixed-strings", "Alias for -F"),
        clEnumValN(re::RE_Syntax::BRE, "basic-regexp", "Alias for -G"),
        clEnumValN(re::RE_Syntax::PCRE, "perl-regexp", "Alias for -P"),
        clEnumValN(re::RE_Syntax::FileGLOB, "GLOB", "Posix GLOB syntax for file name patterns"),
        clEnumValN(re::RE_Syntax::PROSITE, "PROSITE", "PROSITE protein patterns syntax")
        CL_ENUM_VAL_SENTINEL), cl::cat(RE_Options), cl::Grouping, cl::location(RegexpSyntax), cl::init(re::RE_Syntax::PCRE));

bool IgnoreCaseFlag;
static cl::opt<bool, true> IgnoreCaseOption("i", cl::location(IgnoreCaseFlag), cl::desc("Ignore case distinctions in the pattern and the file."), cl::cat(RE_Options), cl::Grouping);
static cl::alias IgnoreCaseAlias("ignore-case", cl::desc("Alias for -i"), cl::aliasopt(IgnoreCaseOption));

bool InvertMatchFlag;
static cl::opt<bool, true> InvertMatchOption("v", cl::location(InvertMatchFlag), cl::desc("Invert match results: select non-matching lines."), cl::cat(RE_Options), cl::Grouping);
static cl::alias InvertMatchAlias("invert-match", cl::desc("Alias for -v"), cl::aliasopt(InvertMatchOption));

bool LineRegexpFlag;
static cl::opt<bool, true> LineRegexpOption("x", cl::location(LineRegexpFlag), cl::desc("Require that entire lines be matched."), cl::cat(RE_Options), cl::Grouping);
static cl::alias LineRegexpAlias("line-regexp", cl::desc("Alias for -x"), cl::aliasopt(LineRegexpOption));

bool WordRegexpFlag;
static cl::opt<bool, true> WordRegexpOption("w", cl::location(WordRegexpFlag), cl::desc("Require that that whole words be matched."), cl::cat(RE_Options), cl::Grouping);
static cl::alias WordRegexpAlias("word-regexp", cl::desc("Alias for -w"), cl::aliasopt(WordRegexpOption));

std::vector<std::string> RegexpVector;
static cl::list<std::string, std::vector<std::string>> RegexpOption("e", cl::location(RegexpVector), cl::desc("Regular expression"), cl::ZeroOrMore, cl::cat(RE_Options), cl::Grouping);
static cl::alias RegexpAlias("regexp", cl::desc("Alias for -e"), cl::aliasopt(RegexpOption));

std::string FileFlag;
static cl::opt<std::string, true> FileOption("f", cl::location(FileFlag), cl::desc("Take regular expressions (one per line) from a file."), cl::cat(RE_Options), cl::Grouping);
static cl::alias FileAlias("file", cl::desc("Alias for -f"), cl::aliasopt(FileOption));
    
/*
 *  B.  Grep input sources and interpretation.
 */
    
static cl::OptionCategory Input_Options("B. Input Options", "These options control the input.");

bool TextFlag;
static cl::opt<bool, true> TextOption("a", cl::location(TextFlag), cl::desc("Treat each input file as text, even if it is a binary file."), cl::cat(Input_Options), cl::Grouping);
static cl::alias TextAlias("text", cl::desc("Alias for -a"), cl::aliasopt(TextOption));

bool BinaryNonMatchingFlag;
static cl::opt<bool, true> BinaryNonMatchingOption("I", cl::location(BinaryNonMatchingFlag), cl::desc("Treat binary files as non-matching."), cl::cat(Input_Options), cl::Grouping);
static cl::alias BinaryNonMatchingAlias("binary-non-matching", cl::desc("Alias for -I"), cl::aliasopt(BinaryNonMatchingOption));

bool BinaryFlag;
static cl::opt<bool, true> BinaryOption("U", cl::location(BinaryFlag), cl::desc("Treat each input file as a binary file, without CRLF normalization."), cl::cat(Input_Options), cl::Grouping);
static cl::alias BinaryAlias("binary", cl::desc("Alias for -U"), cl::aliasopt(BinaryOption));

bool NullDataFlag;
static cl::opt<bool, true> NullDataOption("z", cl::location(NullDataFlag), cl::desc("Use the NUL character (codepoint 00) as the line-break character for input."), cl::cat(Input_Options), cl::Grouping);
static cl::alias NullDataAlias("null-data", cl::desc("Alias for -z"), cl::aliasopt(NullDataOption));

bool UnicodeLinesFlag;
static cl::opt<bool, true> UnicodeLinesOption("Unicode-lines", cl::location(UnicodeLinesFlag), cl::desc("Enable Unicode line breaks (LF/VT/FF/CR/NEL/LS/PS/CRLF)"), cl::cat(Input_Options));

BinaryFilesMode BinaryFilesFlag;
static cl::opt<BinaryFilesMode, true> BinaryFilesOption("binary-files", cl::desc("Processing mode for binary files:"),
                                                     cl::values(clEnumValN(Binary, "binary", "Report match/non-match without printing matches."),
                                                                clEnumValN(WithoutMatch, "without-match", "Always report as non-matching."),
                                                                clEnumValN(Text, "text", "Treat binary files as text.")
                                                                CL_ENUM_VAL_SENTINEL), cl::cat(Input_Options), cl::location(BinaryFilesFlag), cl::init(Text));
    

    
/*
 *  C.  Grep output modes and options.
 */
    
    
static cl::OptionCategory Output_Options("C. Output Options",
                                            "These options control the output.");
    
GrepModeType Mode;
static cl::opt<GrepModeType, true> GrepModeOption(cl::desc("Abbreviated output mode options:"),
    cl::values(
        clEnumValN(CountOnly, "c", "Display only the count of matching lines per file."),
        clEnumValN(FilesWithMatch, "l", "Display only the names of files that have at least one match to the pattern."),
        clEnumValN(FilesWithoutMatch, "L", "Display only the names of files that do not match the pattern."),
        clEnumValN(QuietMode, "q", "Do not generate any output and ignore errors; set the return to zero status if a match is found."),
        clEnumValN(CountOnly, "count", "Alias for -c"),
        clEnumValN(FilesWithMatch, "files-with-match", "Alias for -l"),
        clEnumValN(FilesWithoutMatch, "files-without-match", "Alias for -L"),
        clEnumValN(QuietMode, "quiet", "Alias for -q"),
        clEnumValN(QuietMode, "silent", "Alias for -q")
        CL_ENUM_VAL_SENTINEL), cl::cat(Output_Options), cl::Grouping, cl::location(Mode), cl::init(NormalMode));

bool WithFilenameFlag;
static cl::opt<bool, true> WithFilenameOption("H", cl::location(WithFilenameFlag), cl::desc("Show the file name with each matching line."), cl::cat(Output_Options), cl::Grouping);
static cl::alias WithFilenameAlias("with-filename", cl::desc("Alias for -H"), cl::aliasopt(WithFilenameOption));

bool NoFilenameFlag;
static cl::opt<bool, true> NoFilenameOption("h", cl::location(NoFilenameFlag), cl::desc("Do not show filenames with maches."), cl::cat(Output_Options), cl::Grouping);
static cl::alias NoFilenameAlias("no-filename", cl::desc("Alias for -h"), cl::aliasopt(NoFilenameOption));

bool NullFlag;
static cl::opt<bool, true> NullOption("Z", cl::location(NullFlag), cl::desc("Write NUL characters after filenames generated to output."), cl::cat(Output_Options), cl::Grouping);
static cl::alias NullAlias("null", cl::desc("Alias for -Z"), cl::aliasopt(NullOption));

bool LineNumberFlag;
static cl::opt<bool, true> LineNumberOption("n", cl::location(LineNumberFlag), cl::desc("Show the line number with each matching line."), cl::cat(Output_Options), cl::Grouping);
static cl::alias LineNumberAlias("line-number", cl::desc("Alias for -n"), cl::aliasopt(LineNumberOption));

bool ByteOffsetFlag;
static cl::opt<bool, true> ByteOffsetOption("b", cl::location(ByteOffsetFlag), cl::desc("Show the byte offset within the file for each matching line."), cl::cat(Output_Options), cl::Grouping);
static cl::alias ByteOffsetAlias("byte-offset", cl::desc("Alias for -b"), cl::aliasopt(ByteOffsetOption));

bool UnixByteOffsetsFlag;
static cl::opt<bool, true> UnixByteOffsetsOption("u", cl::location(UnixByteOffsetsFlag), cl::desc("If byte offsets are displayed, report offsets as if all lines are terminated with a single LF."), cl::cat(Output_Options), cl::Grouping);
static cl::alias UnixByteOffsetsAlias("unix-byte-offsets", cl::desc("Alias for -u"), cl::aliasopt(UnixByteOffsetsOption));

bool InitialTabFlag;
static cl::opt<bool, true> InitialTabOption("T", cl::location(InitialTabFlag), cl::desc("Line up matched line content using an inital tab character."), cl::cat(Output_Options), cl::Grouping);
static cl::alias InitialTabAlias("initial-tab", cl::desc("Alias for -T"), cl::aliasopt(InitialTabOption));

bool OnlyMatchingFlag;
static cl::opt<bool, true> OnlyMatchingOption("o", cl::location(OnlyMatchingFlag), cl::desc("Display only the exact strings that match the pattern, with possibly multiple matches per line."), cl::cat(Output_Options), cl::Grouping);
static cl::alias OnlyMatchingAlias("only-matching", cl::desc("Alias for -o"), cl::aliasopt(OnlyMatchingOption));

std::string LabelFlag;
    static cl::opt<std::string, true> LabelOption("label", cl::location(LabelFlag), cl::init("(standard input)"),
                                              cl::desc("Set a label for input lines matched from stdin."), cl::cat(Output_Options));

bool LineBufferedFlag;
static cl::opt<bool, true> LineBufferedOption("line-buffered", cl::location(LineBufferedFlag), cl::desc("Buffer lines to output."), cl::cat(Output_Options));

int AfterContext;
    static cl::opt<int, true> AfterContextOption("A", cl::location(AfterContext), cl::desc("Print <num> lines of context after each matching line."), cl::cat(Output_Options), cl::Prefix, cl::init(0));
static cl::alias AfterContextAlias("after-context", cl::desc("Alias for -A"), cl::aliasopt(AfterContextOption));

int BeforeContext;
static cl::opt<int, true> BeforeContextOption("B", cl::location(BeforeContext), cl::desc("Print <num>lines of context before each matching line."), cl::cat(Output_Options), cl::Prefix, cl::init(0));
static cl::alias BeforeContextAlias("before-context", cl::desc("Alias for -B"), cl::aliasopt(BeforeContextOption));

static cl::opt<int> Context("C", cl::desc("Print <num> lines of context before and after each matching line."), cl::cat(Output_Options), cl::Prefix, cl::init(0));
static cl::alias ContextAlias("context", cl::desc("Alias for -C"), cl::aliasopt(Context));

int MaxCountFlag;
static cl::opt<int, true> MaxCountOption("m", cl::location(MaxCountFlag),
                                         cl::desc("Process only the first <num> matches per file3."),
                                         cl::cat(Output_Options), cl::Grouping);
static cl::alias MaxCountAlias("max-count", cl::desc("Alias for -m"), cl::aliasopt(MaxCountOption));

ColoringType ColorFlag = neverColor;
#if 0
static cl::opt<ColoringType, true> Color("color", cl::desc("Set colorization of the output"), cl::location(ColorFlag), cl::cat(Output_Options), cl::init(neverColor),
                                 cl::values(clEnumValN(alwaysColor, "always", "Enable colorization"),
                                            clEnumValN(autoColor,   "auto", "Colorize output to stdout"),
                                            clEnumValN(neverColor,  "never", "Disable colorization")
                                            CL_ENUM_VAL_SENTINEL));
static cl::alias ColorAlias("colour", cl::desc("Alias for -color"), cl::aliasopt(Color));
#endif
//
// Handler for errors reported through llvm::report_fatal_error.  Report
// and signal error the InternalFailure exit code.
// 
static void icgrep_error_handler(void *UserData, const std::string &Message, bool GenCrashDiag) {
    // Modified from LLVM's internal report_fatal_error logic.
    #ifndef NDEBUG
    throw std::runtime_error(Message);
    #else
    SmallVector<char, 64> Buffer;
    raw_svector_ostream OS(Buffer);
    OS << "icgrep ERROR: " << Message << "\n";
    const auto MessageStr = OS.str();
    ssize_t written = ::write(2, MessageStr.data(), MessageStr.size());
    (void)written; // If something went wrong, we deliberately just give up.
    // Run the interrupt handlers to make sure any special cleanups get done, in
    // particular that we remove files registered with RemoveFileOnSignal.
    llvm::sys::RunInterruptHandlers();
    exit(InternalFailureCode);
    #endif
}

void InitializeCommandLineInterface(int argc, char *argv[]) {
    llvm::install_fatal_error_handler(&icgrep_error_handler);
    codegen::ParseCommandLineOptions(argc, argv, {&RE_Options, &Input_Options, &Output_Options, re::re_toolchain_flags(), pablo::pablo_toolchain_flags(), codegen::codegen_flags()});
    if (argv::RecursiveFlag || argv::DereferenceRecursiveFlag) {
        argv::DirectoriesFlag = argv::Recurse;
    }
    
    if (TextFlag) {
        if (BinaryNonMatchingFlag || (BinaryFilesFlag == WithoutMatch)) {
            llvm::report_fatal_error("Conflicting options for binary files.\n");
        }
        BinaryFilesFlag = Text;
    }
    if (BinaryNonMatchingFlag) {
        if (BinaryFilesFlag == Binary) {
            llvm::report_fatal_error("Conflicting options for binary files.\n");
        }
        BinaryFilesFlag = WithoutMatch;
    }
    if (BinaryFlag) {
        llvm::report_fatal_error("Sorry, -U is not yet supported.\n");
    }
    if (ByteOffsetFlag) {
        llvm::report_fatal_error("Sorry, -b is not yet supported.\n");
    }
    if (UnixByteOffsetsFlag) {
        llvm::report_fatal_error("Sorry, -u is not yet supported.\n");
    }
    if (OnlyMatchingFlag) {
        llvm::report_fatal_error("Sorry, -o is not yet supported.\n");
    }
    if (LineBufferedFlag) {
        llvm::report_fatal_error("Sorry, -line-buffered is not yet supported.\n");
    }
    if (Context != 0) {
        if (AfterContext == 0) AfterContext = Context;
        if (BeforeContext == 0) BeforeContext = Context;
    }
    if (ColorFlag!=neverColor) {
        llvm::report_fatal_error("Sorry, -color is not yet supported.\n");
    }
    if ((Mode == QuietMode) | (Mode == FilesWithMatch) | (Mode == FilesWithoutMatch)) {
        MaxCountFlag = 1;
    }
}
}
