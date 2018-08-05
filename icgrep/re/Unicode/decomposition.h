/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef DECOMPOSITION_H
#define DECOMPOSITION_H

namespace re { class RE; class CC;}

/*  NFD, NFKD and casefold decompositions of a character class.
    Each codepoint in a class is mapped to its decomposition
    under the appropriate mapping, which may be itself, another
    single codepoint (singleton decomposition) or a codepoint
    string (expanding decomposition).   In general, the result is
    a set of alternatives consisting of sequences for each expanding
    decomposition as well as a single character class for all the
    singledton decompositions as well as the codepoints that map to
    themselves.
*/

re::RE * NFD_CC(re::CC * cc);
    
re::RE * NFKD_CC(re::CC * cc);

re::RE * Casefold_CC(re::CC * cc);

/*  Systematic NFD, NFKD and casefold decomposition of all character
    classes in a regular expression.  */

re::RE * NFD_RE(re::RE * r);

re::RE * NFKD_RE(re::RE * r);

re::RE * Casefold_RE(re::RE * r);

#endif
