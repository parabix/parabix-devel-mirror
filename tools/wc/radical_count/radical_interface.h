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
                if(_unicodeset_radical_table.find(radical) != _unicodeset_radical_table.end())
                    return std::move(*_unicodeset_radical_table[radical]);
                else
                    return std::move(UCD::UnicodeSet());
            }
        private:
            static map<string, const UCD::UnicodeSet*> _unicodeset_radical_table;
    };
    
}

#endif /* radical_interface_h */
