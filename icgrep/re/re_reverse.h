/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef RE_REVERSE_H
#define RE_REVERSE_H

//#include <map>                           // for map
namespace re { class RE; class Name;}

namespace re {
    
/*
class Reverser (
public:
    Reverser();
    */
    RE * reverse(RE * re);
    /*
private:
    using NameMap = std::map<std::pair<std::string, std::string>, re::Name *>;
    unsigned                    mCaptureGroupCount;
    NameMap                     mNameMap;
    Memoizer                    mMemoizer;
}
*/
}

#endif // RE_REVERSE_H
