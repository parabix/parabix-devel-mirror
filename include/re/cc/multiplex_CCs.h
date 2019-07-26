/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef MULTIPLEX_CCS_H
#define MULTIPLEX_CCS_H

#include <re/alphabet/alphabet.h>

namespace re { class CC; }

namespace cc {

class MultiplexedAlphabet final : public Alphabet {
public:
    MultiplexedAlphabet(const std::string alphabetName, const std::vector<re::CC *> CCs);

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
    const std::vector<re::CC *> mUnicodeSets;
    std::vector<std::vector<unsigned>> mExclusiveSetIDs;
    std::vector<re::CC *> mMultiplexedCCs;

    unsigned long findTargetCCIndex(const re::CC * sourceCC) const;
};

}


#endif
