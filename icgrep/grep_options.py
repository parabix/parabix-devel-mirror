#!/usr/bin/python
# 
# A simple script to partially automate flag processing.

integer = "int"
boolean = "bool"
string = "std::string"
stringlist = "std::vector<std::string>"

GrepOptions = [[boolean, "i", "ignore-case", "Ignore case distinctions in the pattern and the file.", "RE_Options"],
                  [boolean, "v", "invert-match", "Invert match results: select non-matching lines.", "RE_Options"],
                  [boolean, "x", "line-regexp", "Require that entire lines be matched.", "RE_Options"],
                  [boolean, "w", "word-regexp", "Require that that whole words be matched.", "RE_Options"],
                  [stringlist, "e", "regexp", "Regular expression", "RE_Options"],
                  [string, "f", "file", "Take regular expressions (one per line) from a file.", "RE_Options"],

                  #[boolean, "F", "fixed-strings", "Interpret patterns as fixed strings, separated by newlines", "RE_Options"],
                  #[boolean, "G", "basic-regexp", "Interpret pattern using Posix basic regular expression (BRE) syntax", "RE_Options"],
                  #[boolean, "E", "extended-regexp", "Interpret pattern using Posix extended regular expression (ERE) syntax", "RE_Options"],
                  #[boolean, "P", "perl-regexp", "Interpret pattern using Perl-compatible regular expression (PCRE) syntax", "RE_Options"],
                  #[boolean, "", "PROSITE", "Interpret pattern using PROSITE protein pattern syntax", "RE_Options"],

                  [boolean, "r", "recursive", "Recursively process files within directories, (but follow only top-level symlinks unless -R).", "Input_Options"],
                  [boolean, "R", "dereference-recursive", "Recursively process files within directories, following symlinks at all levels.", "Input_Options"],
                  [boolean, "a", "text", "Treat each input file as text, even if it is a binary file.", "Input_Options"],
                  [boolean, "I", "binary-non-matching", "Treat binary files as non-matching.", "Input_Options"],
                  [boolean, "U", "binary", "Treat each input file as a binary file, without CRLF normalization.", "Input_Options"],
                  [boolean, "z", "null-data", "Use the NUL character (codepoint 00) as the line-break character for input.", "Input_Options"],
                  [boolean, "", "mmap", "Use mmap for file input.", "Input_Options"],
                  [string, "", "exclude", "Exclude files matching the given filename GLOB pattern.", "Input_Options"],
                  [string, "", "exclude-from", "Exclude files matching filename GLOB patterns from the given file.", "Input_Options"],
                  [string, "", "exclude-dir", "Exclude directories matching the given pattern.", "Input_Options"],
                  [string, "", "include", "Include only files matching the given filename GLOB pattern.", "Input_Options"],

                  #[boolean, "q", "quiet", "silent", "Do not generate any output and ignore errors; set the return to zero status if a match is found.", "Output_Options"],
                  #[boolean, "L", "files-without-match", "Display only the names of files that do not match the pattern.", "Output_Options"],
                  #[boolean, "l", "files-with-match", "Display only the names of files that have at least one match to the pattern.", "Output_Options"],
                  #[boolean, "c", "count", "Display only the count of matching lines per file.", "Output_Options"],
 
                  [boolean, "s", "no-messages", "Suppress messages for file errors.", "Output_Options"],


                  [boolean, "H", "with-filename", "Show the file name with each matching line.", "Output_Options"],
                  [boolean, "h", "no-filename", "Do not show filenames with maches.", "Output_Options"],
                  [boolean, "Z", "null", "Write NUL characters after filenames generated to output.", "Output_Options"],
                  [boolean, "n", "line-number", "Show the line number with each matching line.", "Output_Options"],

                  [boolean, "b", "byte-offset", "Show the byte offset within the file for each matching line.", "Output_Options"],
                  [boolean, "u", "unix-byte-offsets", "If byte offsets are displayed, report offsets as if all lines are terminated with a single LF.", "Output_Options"],
                  [boolean, "T", "initial-tab", "Line up matched line content using an inital tab character.", "Output_Options"],


                  [boolean, "o", "only-matching", "Display only the exact strings that match the pattern, with possibly multiple matches per line.", "Output_Options"],

                  [string, "", "label", "Set a label for input lines matched from stdin.", "Output_Options"],
                  [boolean, "", "line-buffered", "Buffer lines to output.", "Output_Options"],
                  [boolean, "", "normalize-line-breaks", "Normalize line breaks to LF.", "Output_Options"],

                  # NumericOptions
                  [integer, "A", "after-context", "Print <num> lines of context after each matching line.", "Output_Options"],
                  [integer, "B", "before-context", "Print <num>lines of context before each matching line.", "Output_Options"],
                  [integer, "C", "context", "Print <num> lines of context before and after each matching line.", "Output_Options"],
                  [integer, "m", "max-count", "Process only the first <num> matches per file.", "Output_Options"]]



def CamelCase(s) :
    capNext = True
    t = ""
    for c in s:
        if c == "-": capNext = True
        else:
            if capNext: t += c.upper()
            else: t += c
            capNext = False
    return t

OptionTemplate = r"""%s %sFlag;
static cl::opt<%s, true> %sOption("%s", cl::location(%sFlag), cl::desc("%s"), cl::cat(%s)%s);
"""

ZeroOrMoreTemplate = r"""%s %sVector;
static cl::list<std::string, %s> %sOption("%s", cl::location(%sVector), cl::desc("%s"), cl::ZeroOrMore, cl::cat(%s)%s);
"""

AliasTemplate = r"""static cl::alias %s("%s", cl::desc("Alias for -%s"), cl::aliasopt(%sOption));
"""

ExternTemplate =r"""extern %s %s%s; // -%s
"""

UnsupportedTemplate = r"""if (%sFlag) {
    llvm::report_fatal_error("Sorry, -%s is not yet supported.\n");
}
"""

if __name__ == "__main__":
    p = ""
    e = ""
    u= ""
    for option_spec in GrepOptions:
        datatype = option_spec[0]
        flag = option_spec[1]
        desc = option_spec[-2]
        cat = option_spec[-1]
        aliases = option_spec[2:-2]
        name = CamelCase(aliases[0])
        grouping = ", cl::Grouping"
        if flag == "":
            flag = aliases[0]
            aliases = aliases[1:]
            grouping = ""
        if datatype == stringlist:
            basetype = string
            p += ZeroOrMoreTemplate % (datatype, name, datatype, name, flag, name, desc, cat, grouping) 
        else:
            p += OptionTemplate % (datatype, name, datatype, name, flag, name, desc, cat, grouping) 
        for a in aliases:
            p+= AliasTemplate % (CamelCase(a) + "Alias", a, flag, name)
        p += "\n"
        if datatype == stringlist:
            e += ExternTemplate % (datatype, name, "Vector", flag)
        else:
            e += ExternTemplate % (datatype, name, "Flag", flag)
        u += UnsupportedTemplate % (name, flag)
    print p
    print e
    print u












