#ifndef CC_NAMER_HPP
#define CC_NAMER_HPP

#include <unordered_map>
#include <vector>
#include <string>
#include "re/re_name.h"
#include "re/re_cc.h"


namespace cc {

class CC_NameMap {
public:

    typedef std::unordered_map<std::string, re::Name *>          NameMap;
    typedef std::vector<re::Name*>                              NameVector;
    typedef NameVector::const_iterator                          iterator;

    CC_NameMap() {}

    re::RE * process(re::RE * re, const re::CC_type t);

    inline iterator begin() const {
        return mNameVector.begin();
    }

    inline iterator end() const {
        return mNameVector.end();
    }

    std::string printMap();
    
private:

    inline re::Name * insert(NameMap::key_type && name, re::Name * re) {
        mNameMap.insert(std::make_pair(std::move(name), re));
        mNameVector.push_back(re);
        return re;
    }

private:
    NameMap                 mNameMap;
    NameVector              mNameVector;
};

}

#endif // CC_NAMER_HPP
