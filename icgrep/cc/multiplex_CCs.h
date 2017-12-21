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
    
    std::vector<std::vector<unsigned>> getExclusiveSetIDs();
    
    std::vector<re::CC *> getMultiplexedCCs();
private:
    std::vector<std::vector<unsigned>> mExclusiveSetIDs;
    std::vector<re::CC *> mMultiplexedCCs;
};
}


#endif
