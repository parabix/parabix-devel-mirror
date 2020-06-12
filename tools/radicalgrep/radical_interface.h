#ifndef radical_interface_h
#define radical_interface_h

#include <vector>
#include <string>
#include <array>
#include <map>
#include <utility>
#include <set>
#include <regex>
#include <iostream>
#include <re/adt/re_alt.h>
#include <re/adt/re_seq.h>
#include <re/adt/re_start.h>
#include <re/adt/re_end.h>
#include <re/adt/re_utility.h>
#include <re/parse/parser.h>
#include <re/toolchain/toolchain.h>
#include <unicode/data/kRSKangXi.h>

enum ColoringType {alwaysColor, autoColor, neverColor};
extern ColoringType ColorFlag;
extern bool LineNumberFlag; 
extern bool WithFilenameFlag;
const int MatchFoundExitCode = 0;
const int MatchNotFoundExitCode = 1;

namespace BS
{
    using std::vector;
    using std::array;
    using std::string;
    using std::map;
    using std::set;

    class UnicodeSetTable
    {   
        public:
            const UCD::UnicodeSet&& get_uset(string radical, bool indexMode, bool mixedMode);    //Map the input radical to the corresponding UnicodeSet predefined in kRSKangXi.h
        private:
            static map<string, const UCD::UnicodeSet*> _unicodeset_radical_table;
            static map<string, const UCD::UnicodeSet*> radical_table;   //The map list all kinds of radicals and their corresponding UnicodeSet prodefined in kRSKangXi.h
            static map<string, const UCD::UnicodeSet*> mixed_table;
    };

    class RadicalValuesEnumerator
    {
        public:
            std::vector<re::RE*> createREs(bool indexMode, bool mixedMode, bool altMode);   //Search for the results
            void parse_input(string input_radical, bool altMode); //Search for the results by making CCs of each radical and pushing them the vector REs
            void reParse(string expr); //For -re mode; tokenizes {X/Y}
        private:
            std::vector<string> radical_list;   //Store the input radical(s)
            std::vector<string> reTemp; //For -alt mode; stores the tokenized radicals in reParse()
            std::vector<string> zi; //For -alt mode; stores the non-changed radical in rePare() (e.g. zi would store 亻 and 衣 of 亻_衣_{生/亅})
            std::vector<string> zi2;
            int position;
            int c1;
    };
}

#endif /* radical_interface_h */
