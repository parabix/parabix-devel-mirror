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

/* Transforme an RE so that all string pieces and character classes
 are converted to NFD form (or NFKD form if the Compatible option
 is used.  The options may also including case folding. Examples:
 nfd_re = toNFD(r);
 nfkdi_re = toNFD(r, CaseFold | NFKD);
 */

re::RE * toNFD(re::RE * re, const DecompositionOptions opt = DecompositionOptions::NFD);

}


#endif
