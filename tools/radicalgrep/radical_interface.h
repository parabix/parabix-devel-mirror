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

//Adapted from icgrep
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
            //Map the input radical to the corresponding UnicodeSet predefined in kRSKangXi.h
            const UCD::UnicodeSet&& get_uset(string radical, bool indexMode, bool mixedMode);
        private:
            //This map contains all 214 radical indices, which are mapped to their corresponding UnicodeSet predefined in kRSKangXi.h
            static map<string, const UCD::UnicodeSet*> _unicodeset_radical_table; 
            //The map list all kinds of radical characters and their corresponding UnicodeSet prodefined in kRSKangXi.h
            static map<string, const UCD::UnicodeSet*> radical_table;   
    };

    class RadicalValuesEnumerator
    {
        public:
            //Creates the regular expression of the input and returns it in the form of a vector
            std::vector<re::RE*> createREs(bool indexMode, bool mixedMode, bool altMode);  
            //Search for the results by making CCs of each radical and pushing them the vector REs
            void parse_input(string input_radical, bool altMode); 
            //For -alt mode; tokenizes expressions that are in the form of {X/Y}
            void reParse(string expr); 
        private:
            //Stores the parsed radicals from input
            std::vector<string> radical_list;  
            //For -alt mode; stores the tokenized radicals from reParse() 
            std::vector<string> reTemp; 
            //For -alt mode; stores the non-changed radical in reParse()
            //(e.g. zi would store 亻 and 衣 of 亻_衣_{生/亅})
            std::vector<string> zi; 
            //Similar to the zi vector, acts as another storage buffer for alt mode
            std::vector<string> zi2;
            //For -alt mode; these variables keep track of positions of certain characters 
            int position;
            int c1;
    };
}

#endif /* radical_interface_h */
