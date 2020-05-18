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
#include <unicode/data/kRSKangXi.h>

typedef std::pair<std::string, std::string> input_radical;

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
            const UCD::UnicodeSet&& get_uset(string radical)
            {   
                //For radical index input (e.g. 85_)
                /*if(_unicodeset_radical_table.find(radical) != _unicodeset_radical_table.end())
                    return std::move(*_unicodeset_radical_table[radical]);
                else
                    return std::move(UCD::UnicodeSet());*/

                if(radical_table.find(radical) != radical_table.end())
                    return std::move(*radical_table[radical]);
                else
                    return std::move(UCD::UnicodeSet());
            }
        private:
            static map<string, const UCD::UnicodeSet*> _unicodeset_radical_table;
            static map<string, const UCD::UnicodeSet*> radical_table;
    };

    static UnicodeSetTable ucd_radical;
}

input_radical parse_input(std::string CC_expr);

#endif /* radical_interface_h */
