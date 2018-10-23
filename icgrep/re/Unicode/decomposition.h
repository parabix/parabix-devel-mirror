/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef DECOMPOSITION_H
#define DECOMPOSITION_H

#include <string>
#include <locale>
#include <codecvt>
#include <re/re_toolchain.h>
#include <UCD/unicode_set.h>

namespace re { class RE; class CC; class Seq;}
namespace UCD { class EnumeratedPropertyObject; class StringPropertyObject; class StringOverridePropertyObject;}

namespace UCD {
    enum DecompositionOptions : int {NFD = 0, CaseFold = 1, NFKD = 2};

    class NFD_Transformer : public re::RE_Transformer {
    public:
        /* Transforme an RE so that all string pieces and character classes
         are converted to NFD form (or NFKD form if the UCD::Compatible option
         is used.  The options may also including case folding.  Example:
         UCD::NFD_Transformer(UCD::CaseFold | UCD::NFKD).transformRE(r);
        */
        NFD_Transformer(DecompositionOptions opt = NFD);
        /* Helpers to convert and append an individual codepoint or a u32string
           to an existing NFD_string.   The process performs any necessary
           reordering of marks of the existing string and the appended data
           to ensure that the result is overall in NFD form.
           These may be used independently of RE transformation, for example:
           UCD::NFD_Transformer(UCD::CaseFold).NFD_append1(s, cp);
        */
        void NFD_append1(std::u32string & NFD_string, codepoint_t cp);
        void NFD_append(std::u32string & NFD_string, std::u32string & to_convert);
    protected:
        re::RE * transformCC(re::CC * cc) override;
        re::RE * transformSeq(re::Seq * seq) override;
        re::RE * transformGroup(re::Group * g) override;
        bool reordering_needed(std::u32string & prefix, codepoint_t suffix_cp);
    private:
        DecompositionOptions mOptions;
        EnumeratedPropertyObject * decompTypeObj;
        StringPropertyObject * decompMappingObj;
        EnumeratedPropertyObject * cccObj;
        StringOverridePropertyObject * caseFoldObj;
        const UnicodeSet & canonicalMapped;
        const UnicodeSet & cc0Set;
        const UnicodeSet selfNFKD;
        const UnicodeSet selfCaseFold;
        std::wstring_convert<std::codecvt_utf8<char32_t>, char32_t> conv;
    };
}
#endif
