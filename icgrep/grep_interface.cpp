/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <grep_interface.h>
#include <llvm/Support/CommandLine.h>
#include <llvm/Support/ErrorHandling.h>
#include <llvm/Support/Signals.h>
#include <llvm/Support/raw_ostream.h>
#include <toolchain/toolchain.h>
#include <re/re_toolchain.h>
#include <pablo/pablo_toolchain.h>

using namespace llvm;

namespace grep {

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
        clEnumValN(re::RE_Syntax::PROSITE, "PROSITE", "PROSITE protein patterns syntax"),
        clEnumValEnd), cl::cat(RE_Options), cl::Grouping, cl::location(RegexpSyntax), cl::init(re::RE_Syntax::PCRE));

bool IgnoreCaseFlag;
static cl::opt<bool, true> IgnoreCase("i", cl::location(IgnoreCaseFlag), cl::desc("Ignore case distinctions in the pattern and the file."), cl::cat(RE_Options), cl::Grouping);
static cl::alias IgnoreCaseAlias("ignore-case", cl::desc("Alias for -i"), cl::aliasopt(IgnoreCase));

bool InvertMatchFlag;
static cl::opt<bool, true> InvertMatch("v", cl::location(InvertMatchFlag), cl::desc("Invert match results: select non-matching lines."), cl::cat(RE_Options), cl::Grouping);
static cl::alias InvertMatchAlias("invert-match", cl::desc("Alias for -v"), cl::aliasopt(InvertMatch));

bool LineRegexpFlag;
static cl::opt<bool, true> LineRegexp("x", cl::location(LineRegexpFlag), cl::desc("Require that entire lines be matched."), cl::cat(RE_Options), cl::Grouping);
static cl::alias LineRegexpAlias("line-regexp", cl::desc("Alias for -x"), cl::aliasopt(LineRegexp));

bool WordRegexpFlag;
static cl::opt<bool, true> WordRegexp("w", cl::location(WordRegexpFlag), cl::desc("Require that that whole words be matched."), cl::cat(RE_Options), cl::Grouping);
static cl::alias WordRegexpAlias("word-regexp", cl::desc("Alias for -w"), cl::aliasopt(WordRegexp));
    
    
const cl::OptionCategory * grep_regexp_flags() {
    return &RE_Options;
}

/*
 *  B.  Grep input sources and interpretation.
 */
    
static cl::OptionCategory Input_Options("B. Input Options", "These options control the input.");

bool RecursiveFlag;
static cl::opt<bool, true> Recursive("r", cl::location(RecursiveFlag), cl::desc("Recursively process files within directories, (but follow only top-level symlinks unless -R)."), cl::cat(Input_Options), cl::Grouping);
static cl::alias RecursiveAlias("recursive", cl::desc("Alias for -r"), cl::aliasopt(Recursive));

bool DereferenceRecursiveFlag;
static cl::opt<bool, true> DereferenceRecursive("R", cl::location(DereferenceRecursiveFlag), cl::desc("Recursively process files within directories, following symlinks at all levels."), cl::cat(Input_Options), cl::Grouping);
static cl::alias DereferenceRecursiveAlias("dereference-recursive", cl::desc("Alias for -R"), cl::aliasopt(DereferenceRecursive));

bool TextFlag;
static cl::opt<bool, true> Text("a", cl::location(TextFlag), cl::desc("Treat each input file as text, even if it is a binary file."), cl::cat(Input_Options), cl::Grouping);
static cl::alias TextAlias("text", cl::desc("Alias for -a"), cl::aliasopt(Text));

bool BinaryFlag;
static cl::opt<bool, true> Binary("U", cl::location(BinaryFlag), cl::desc("Treat each input file as a binary file, without CRLF normalization."), cl::cat(Input_Options), cl::Grouping);
static cl::alias BinaryAlias("binary", cl::desc("Alias for -U"), cl::aliasopt(Binary));

bool NullDataFlag;
static cl::opt<bool, true> NullData("z", cl::location(NullDataFlag), cl::desc("Use the NUL character (codepoint 00) as the line-break character for input."), cl::cat(Input_Options), cl::Grouping);
static cl::alias NullDataAlias("null-data", cl::desc("Alias for -z"), cl::aliasopt(NullData));

bool MmapFlag;
static cl::opt<bool, true> Mmap("mmap", cl::location(MmapFlag), cl::desc("Use mmap for file input."), cl::cat(Input_Options));
    
    

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
        clEnumValN(QuietMode, "silent", "Alias for -q"),
        clEnumValEnd), cl::cat(Output_Options), cl::Grouping, cl::location(Mode), cl::init(NormalMode));

bool NoMessagesFlag;
static cl::opt<bool, true> NoMessages("s", cl::location(NoMessagesFlag), cl::desc("Suppress messages for file errors."), cl::cat(Output_Options), cl::Grouping);
static cl::alias NoMessagesAlias("no-messages", cl::desc("Alias for -s"), cl::aliasopt(NoMessages));

bool WithFilenameFlag;
static cl::opt<bool, true> WithFilename("H", cl::location(WithFilenameFlag), cl::desc("Show the file name with each matching line."), cl::cat(Output_Options), cl::Grouping);
static cl::alias WithFilenameAlias("with-filename", cl::desc("Alias for -H"), cl::aliasopt(WithFilename));

bool NoFilenameFlag;
static cl::opt<bool, true> NoFilename("h", cl::location(NoFilenameFlag), cl::desc("Do not show filenames with maches."), cl::cat(Output_Options), cl::Grouping);
static cl::alias NoFilenameAlias("no-filename", cl::desc("Alias for -h"), cl::aliasopt(NoFilename));

bool NullFlag;
static cl::opt<bool, true> Null("Z", cl::location(NullFlag), cl::desc("Write NUL characters after filenames generated to output."), cl::cat(Output_Options), cl::Grouping);
static cl::alias NullAlias("null", cl::desc("Alias for -Z"), cl::aliasopt(Null));

bool LineNumberFlag;
static cl::opt<bool, true> LineNumber("n", cl::location(LineNumberFlag), cl::desc("Show the line number with each matching line."), cl::cat(Output_Options), cl::Grouping);
static cl::alias LineNumberAlias("line-number", cl::desc("Alias for -n"), cl::aliasopt(LineNumber));

bool ByteOffsetFlag;
static cl::opt<bool, true> ByteOffset("b", cl::location(ByteOffsetFlag), cl::desc("Show the byte offset within the file for each matching line."), cl::cat(Output_Options), cl::Grouping);
static cl::alias ByteOffsetAlias("byte-offset", cl::desc("Alias for -b"), cl::aliasopt(ByteOffset));

bool UnixByteOffsetsFlag;
static cl::opt<bool, true> UnixByteOffsets("u", cl::location(UnixByteOffsetsFlag), cl::desc("If byte offsets are displayed, report offsets as if all lines are terminated with a single LF."), cl::cat(Output_Options), cl::Grouping);
static cl::alias UnixByteOffsetsAlias("unix-byte-offsets", cl::desc("Alias for -u"), cl::aliasopt(UnixByteOffsets));

bool InitialTabFlag;
static cl::opt<bool, true> InitialTab("T", cl::location(InitialTabFlag), cl::desc("Line up matched line content using an inital tab character."), cl::cat(Output_Options), cl::Grouping);
static cl::alias InitialTabAlias("initial-tab", cl::desc("Alias for -T"), cl::aliasopt(InitialTab));

bool OnlyMatchingFlag;
static cl::opt<bool, true> OnlyMatching("o", cl::location(OnlyMatchingFlag), cl::desc("Display only the exact strings that match the pattern, with possibly multiple matches per line."), cl::cat(Output_Options), cl::Grouping);
static cl::alias OnlyMatchingAlias("only-matching", cl::desc("Alias for -o"), cl::aliasopt(OnlyMatching));

bool LineBufferedFlag;
static cl::opt<bool, true> LineBuffered("line-buffered", cl::location(LineBufferedFlag), cl::desc("Buffer lines to output."), cl::cat(Output_Options));

bool NormalizeLineBreaksFlag;
static cl::opt<bool, true> NormalizeLineBreaks("normalize-line-breaks", cl::location(NormalizeLineBreaksFlag), cl::desc("Normalize line breaks to LF."), cl::cat(Output_Options));

size_t AfterContextFlag;
static cl::opt<size_t, true> AfterContext("A", cl::location(AfterContextFlag), cl::desc("Print <num> lines of context after each matching line."), cl::cat(Output_Options), cl::Grouping);
static cl::alias AfterContextAlias("after-context", cl::desc("Alias for -A"), cl::aliasopt(AfterContext));

size_t BeforeContextFlag;
static cl::opt<size_t, true> BeforeContext("B", cl::location(BeforeContextFlag), cl::desc("Print <num>lines of context before each matching line."), cl::cat(Output_Options), cl::Grouping);
static cl::alias BeforeContextAlias("before-context", cl::desc("Alias for -B"), cl::aliasopt(BeforeContext));

size_t ContextFlag;
static cl::opt<size_t, true> Context("C", cl::location(ContextFlag), cl::desc("Print <num> lines of context before and after each matching line."), cl::cat(Output_Options), cl::Grouping);
static cl::alias ContextAlias("context", cl::desc("Alias for -C"), cl::aliasopt(Context));

size_t MaxCountFlag;
static cl::opt<size_t, true> MaxCount("m", cl::location(MaxCountFlag), cl::desc("Process only the first <num> matches per file."), cl::cat(Output_Options), cl::Grouping);
static cl::alias MaxCountAlias("max-count", cl::desc("Alias for -m"), cl::aliasopt(MaxCount));
    
std::string LabelFlag;
static cl::opt<std::string, true> Label("label", cl::location(LabelFlag), cl::desc("Set a label for input lines matched from stdin."), cl::cat(Output_Options));
    
ColoringType ColorFlag;
static cl::opt<ColoringType, true> Color("color", cl::desc("Set colorization of the output"), cl::location(ColorFlag), cl::cat(Output_Options), cl::init(neverColor),
                                 cl::values(clEnumValN(alwaysColor, "always", "Enable colorization"),
                                            clEnumValN(autoColor,   "auto", "Colorize output to stdout"),
                                            clEnumValN(neverColor,  "never", "Disable colorization"),
                                            clEnumValEnd));
static cl::alias ColorAlias("colour", cl::desc("Alias for -color"), cl::aliasopt(Color));
//
// Handler for errors reported through llvm::report_fatal_error.  Report
// and signal error the InternalFailure exit code.
// 
static void icgrep_error_handler(void *UserData, const std::string &Message, bool GenCrashDiag) {
#ifndef NDEBUG
        throw std::runtime_error(Message);
#else
        // Modified from LLVM's internal report_fatal_error logic.
        SmallVector<char, 64> Buffer;
        raw_svector_ostream OS(Buffer);
        OS << "icgrep ERROR: " << Message << "\n";
        StringRef MessageStr = OS.str();
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
    AddParabixVersionPrinter();
#ifndef USE_LLVM_3_6
    cl::HideUnrelatedOptions(ArrayRef<const cl::OptionCategory *>{&RE_Options, &Input_Options, &Output_Options, re::re_toolchain_flags(), pablo::pablo_toolchain_flags(), codegen::codegen_flags()});
#endif
    cl::ParseCommandLineOptions(argc, argv);
    if (RegexpSyntax == re::RE_Syntax::FixedStrings) {
        llvm::report_fatal_error("Sorry, FixedStrings syntax is not fully supported.\n");
    }
    if (TextFlag) {
        llvm::report_fatal_error("Sorry, -a is not yet supported.\n");
    }
    if (BinaryFlag) {
        llvm::report_fatal_error("Sorry, -U is not yet supported.\n");
    }
    if (NullDataFlag) {
        llvm::report_fatal_error("Sorry, -z is not yet supported.\n");
    }
    if (NoMessagesFlag) {
        llvm::report_fatal_error("Sorry, -s is not yet supported.\n");
    }
    if (NullFlag) {
        llvm::report_fatal_error("Sorry, -Z is not yet supported.\n");
    }
    if (ByteOffsetFlag) {
        llvm::report_fatal_error("Sorry, -b is not yet supported.\n");
    }
    if (UnixByteOffsetsFlag) {
        llvm::report_fatal_error("Sorry, -u is not yet supported.\n");
    }
    if (InitialTabFlag) {
        llvm::report_fatal_error("Sorry, -T is not yet supported.\n");
    }
    if (OnlyMatchingFlag) {
        llvm::report_fatal_error("Sorry, -o is not yet supported.\n");
    }
    if (LineBufferedFlag) {
        llvm::report_fatal_error("Sorry, -line-buffered is not yet supported.\n");
    }
    if (AfterContextFlag) {
        llvm::report_fatal_error("Sorry, -A is not yet supported.\n");
    }
    if (BeforeContextFlag) {
        llvm::report_fatal_error("Sorry, -B is not yet supported.\n");
    }
    if (ContextFlag) {
        llvm::report_fatal_error("Sorry, -C is not yet supported.\n");
    }
    if (LabelFlag!="") {
        llvm::report_fatal_error("Sorry, -label is not yet supported.\n");
    }
    if (ColorFlag!=neverColor) {
        llvm::report_fatal_error("Sorry, -color is not yet supported.\n");
    }
}
}
