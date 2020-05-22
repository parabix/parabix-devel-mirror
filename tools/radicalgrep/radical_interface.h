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
            const UCD::UnicodeSet&& get_uset(string radical, bool indexMode)    //Map the input radical to the corresponding UnicodeSet predefined in kRSKangXi.h
            {
                if (indexMode) { //search using the index (e.g. 85_)
                    if(_unicodeset_radical_table.find(radical) != _unicodeset_radical_table.end())
                        return std::move(*_unicodeset_radical_table[radical]);
                    else
                        return std::move(UCD::UnicodeSet());
                } else { //search using the actual radical (e.g. 氵_)
                    if(radical_table.find(radical) != radical_table.end())
                        return std::move(*radical_table[radical]);
                    else
                        return std::move(UCD::UnicodeSet());
                }
                
            }
        private:
            static map<string, const UCD::UnicodeSet*> _unicodeset_radical_table;
            static map<string, const UCD::UnicodeSet*> radical_table;   //The map list all kinds of radicals and their corresponding UnicodeSet prodefined in kRSKangXi.h
    };

    static UnicodeSetTable ucd_radical;

    class RadicalValuesEnumerator
    {
        public:
            std::vector<re::RE*> createREs(bool indexMode);   //Search for the results
            void parse_input(string input_radical); //Parse the input "r1_r2_" or "r0_", disassemble the input radical(s) and store it (them) in vector
        private:
            std::vector<string> radical_list;   //Store the input radical(s)
            int radical_num;    //Store the number of input radical(s)
    };
}

#endif /* radical_interface_h */
