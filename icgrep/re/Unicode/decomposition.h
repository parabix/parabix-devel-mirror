/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef DECOMPOSITION_H
#define DECOMPOSITION_H

enum DecompositionOptions : int { NFD = 0, CaseFold = 1, NFKD = 2 };

namespace re { class RE; }

namespace UCD {

    re::RE * transform(re::RE * re, const DecompositionOptions opt = DecompositionOptions::NFD);

}


#endif
