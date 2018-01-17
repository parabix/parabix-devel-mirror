/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef MULTIPLEX_CCS_H
#define MULTIPLEX_CCS_H

#include <vector>
#include <cc/alphabet.h>

namespace re { class CC; }

namespace cc {

class MultiplexedAlphabet : public Alphabet {
public:
    MultiplexedAlphabet(std::string alphabetName, const std::vector<const re::CC *> CCs);
    static inline bool classof(const Alphabet * a) {
        return a->getClassTypeId() == ClassTypeId::MultiplexedAlphabet;
    }
    static inline bool classof(const void *) {return false;}
    
    const unsigned getSize() const override {return mUnicodeSets.size() + 1;}

    const Alphabet * getSourceAlphabet() const {
        return mSourceAlphabet;
    }
    
    const std::vector<std::vector<unsigned>> & getExclusiveSetIDs() const {
        return mExclusiveSetIDs;
    }
    
    const std::vector<re::CC *> & getMultiplexedCCs() const {
        return mMultiplexedCCs;
    }
    
    re::CC * transformCC(const re::CC * sourceCC) const;
    
    re::CC * invertCC(const re::CC * transformedCC) const;
private:
    const Alphabet * mSourceAlphabet;
    const std::vector<const re::CC *> mUnicodeSets;
    std::vector<std::vector<unsigned>> mExclusiveSetIDs;
    std::vector<re::CC *> mMultiplexedCCs;
};
}


#endif
