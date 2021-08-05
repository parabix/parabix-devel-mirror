/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <re/unicode/equivalence.h>

#include <string>
#include <locale>
#include <codecvt>
#include <llvm/Support/Casting.h>
#include <llvm/Support/raw_ostream.h>
#include <re/adt/adt.h>
#include <re/adt/printer_re.h>
#include <re/alphabet/alphabet.h>
#include <re/transforms/re_transformer.h>
#include <re/unicode/decomposition.h>
#include <unicode/core/unicode_set.h>
#include "unicode/data/PropertyAliases.h"
#include "unicode/data/PropertyObjects.h"
#include <unicode/data/PropertyObjectTable.h>
#include "unicode/data/PropertyValueAliases.h"
#include <unicode/data/Equivalence.h>
#include <unicode/data/PrecomposedMappings.h>

using namespace re;
using namespace llvm;
namespace UCD {
    
std::u32string getStringPiece(Seq * s, unsigned position) {
    unsigned pos = position;
    unsigned size = s->size();
    std::u32string rslt;
    while ((pos < size) && isa<CC>((*s)[pos])) {
        CC * cc = cast<CC>((*s)[pos]);
        if (cc->getAlphabet() != &cc::Unicode) return rslt;
        if (cc->empty()) return rslt;
        codepoint_t lo = lo_codepoint(cc->front());
        codepoint_t hi = hi_codepoint(cc->back());
        if (lo != hi) // not a singleton CC; end of the string piece.
            return rslt;
        rslt.push_back(lo);
        pos++;
    }
    return rslt;
}
    

// Constants for computation of Hangul decompositions, see Unicode Standard, section 3.12.
const codepoint_t Hangul_SBase = 0xAC00;
const codepoint_t Hangul_LBase = 0x1100;
const codepoint_t Hangul_LMax = 0x1112;
const codepoint_t Hangul_VBase = 0x1161;
const codepoint_t Hangul_VMax = 0x1175;
const codepoint_t Hangul_TBase = 0x11A7;
const codepoint_t Hangul_TMax = 0x11C2;
const unsigned Hangul_TCount = 28;
const unsigned Hangul_NCount = 588;
//const unsigned Hangul_SCount = 11172;


typedef std::pair<codepoint_t, std::vector<unsigned>> MatchResult_t;

class ClusterMatchTransformer : public re::RE_Transformer {
public:
    /* Givan an RE in decomposed form (NFD + CaseFold and/or NFKD),
     produce an RE that will match all equivalent strings under the
     given options.   Note that embedded groups within the RE
     may change options.  Example:
     UCD:AllEquivalentsTransformer(UCD::Caseless).transformRE(r);
     */
    ClusterMatchTransformer(EquivalenceOptions opt = Canonical) :
        RE_Transformer("ClusterMatchTransformer"),
        mOptions(opt),
        cccObj(cast<EnumeratedPropertyObject>(getPropertyObject(ccc))),
        sUCobj(cast<StringPropertyObject>(getPropertyObject(suc))),
        sLCobj(cast<StringPropertyObject>(getPropertyObject(slc))),
        ccc0set(cccObj->GetCodepointSet(CCC_ns::NR)),
        selfUC(sUCobj->GetReflexiveSet()),
        selfLC(sLCobj->GetReflexiveSet())
        {}
protected:
    re::RE * transformSeq(re::Seq * seq) override;
    re::RE * transformGroup(re::Group * g) override;
    void find_precomposed(std::u32string & s, std::vector<unsigned> & sss,
                          unsigned j, std::vector<MatchResult_t> & matches);
    void getMatches(const Trie * t, std::u32string s, std::vector<unsigned> & ccc, unsigned pos,
                              std::vector<MatchResult_t> & matches) const;
    void getMatches(const Trie * t, std::u32string s, std::vector<unsigned> & ccc,
                              unsigned pos, bool skip, std::vector<unsigned> positions, std::vector<MatchResult_t> & matches) const;
    RE * addEquivalents(std::u32string NFD_string);
private:
    EquivalenceOptions mOptions;
    EnumeratedPropertyObject * cccObj;
    const StringPropertyObject * sUCobj;
    const StringPropertyObject * sLCobj;
    const UnicodeSet & ccc0set;
    const UnicodeSet selfUC;
    const UnicodeSet selfLC;
};

    

RE * ClusterMatchTransformer::transformGroup(Group * g) {
    re::Group::Mode mode = g->getMode();
    re::Group::Sense sense = g->getSense();
    auto r = g->getRE();
    EquivalenceOptions saveOptions = mOptions;
    if (mode == re::Group::Mode::CaseInsensitiveMode) {
        if (sense == re::Group::Sense::On) {
            mOptions = static_cast<EquivalenceOptions>(mOptions | UCD::Caseless);
        } else {
            mOptions = static_cast<EquivalenceOptions>(mOptions & ~UCD::Caseless);
        }
    } else if (mode == re::Group::Mode::CompatibilityMode) {
        if (sense == re::Group::Sense::On) {
            mOptions = static_cast<EquivalenceOptions>(mOptions | UCD::Compatible);
        } else {
            mOptions = static_cast<EquivalenceOptions>(mOptions & ~UCD::Compatible);
        }
    }
    RE * t = transform(r);
    mOptions = saveOptions;
    if (t == r) return g;
    return makeGroup(mode, t, sense);
}

RE * ClusterMatchTransformer::transformSeq(Seq * seq) {
    // find and process all string pieces
    unsigned size = seq->size();
    if (size == 0) return seq;
    std::vector<RE *> list;
    unsigned i = 0;
    bool changed = false;
    while (i < size) {
        std::u32string stringPiece = getStringPiece(seq, i);
        if (stringPiece.size() > 0) {
            RE * e = addEquivalents(stringPiece);
            if (Seq * t = dyn_cast<Seq>(e)) {
                unsigned tsize = t->size();
                if ((tsize != size) || (getStringPiece(t,0) != stringPiece)) changed = true;
            } else changed = true;
            list.push_back(addEquivalents(stringPiece));
            i += stringPiece.size();
        } else {
            RE * r = (*seq)[i];
            RE * t = transform(r);
            if (t != r) changed = true;
            list.push_back(t);
            i++;
        }
    }
    if (!changed) return seq;
    return makeSeq(list.begin(), list.end());
}
    
// The lookup process to find precomposed characters that match a
// given NFD string results a vector of match results.   Each
// match result is a pair consisting of the precomposed codepoint
// identified, plus a vector of suffix match positions.   Suffix
// match positions are the position of the final starter (ccc = 0)
// and the positions of following marks.  When there are multiple
// consecutive marks in one combining class, only the last of the
// marks is included.  Mark positions are ordered, but may not be
// consecutive, because of permissible reordering under Unicode rules.
    

void ClusterMatchTransformer::find_precomposed(std::u32string & NFD_string, std::vector<unsigned> & combining_class,
                                                unsigned j, std::vector<MatchResult_t> & matches) {
    codepoint_t cp = NFD_string[j];
    //
    // Canonical equivalents are added in every case.
    if ((cp >= Hangul_LBase) && (cp <= Hangul_LMax)) {
        // Algorithmic precomposition for Hangul.
        // We may have an LV precomposition or both an LV and an
        // LVT precomposition.
        if ((j+1 < NFD_string.size()) && (NFD_string[j+1] >= Hangul_VBase) && (NFD_string[j+1] <= Hangul_VMax)) {
            auto LIndex = cp - Hangul_LBase;
            auto VIndex = NFD_string[j+1] - Hangul_VBase;
            auto LVIndex = LIndex * Hangul_NCount + VIndex * Hangul_TCount;
            matches.push_back(MatchResult_t{Hangul_SBase + LVIndex, {j+1}});
            if ((j+2 < NFD_string.size()) && (NFD_string[j+2] > Hangul_TBase) && (NFD_string[j+2] <= Hangul_TMax)) {
                auto TIndex = NFD_string[j+2] - Hangul_TBase;
                matches.push_back(MatchResult_t{Hangul_SBase + LVIndex + TIndex, {j+2}});
            }
        }
    } else {
        getMatches(&NFD_Trie, NFD_string, combining_class, j, matches);
        if (hasOption(mOptions, UCD::Caseless)) {
            getMatches(&NFDi_Trie, NFD_string, combining_class, j, matches);
        }
        if (hasOption(mOptions, UCD::Compatible)) {
            getMatches(&NFKD_Trie, NFD_string, combining_class, j, matches);
        }
    }
}


void ClusterMatchTransformer::getMatches(const Trie * t, std::u32string s, std::vector<unsigned> & combining_class, unsigned pos,
                                         std::vector<MatchResult_t> & matches) const {
    getMatches(t, s, combining_class, pos, false, {}, matches);
}

void ClusterMatchTransformer::getMatches(const Trie * t, std::u32string NFD_string, std::vector<unsigned> & combining_class,
                                            unsigned pos, bool skip, std::vector<unsigned> positions, std::vector<MatchResult_t> & matches) const {
    std::wstring_convert<std::codecvt_utf8<char32_t>, char32_t> conv;
    if (t->branches.empty() || pos == NFD_string.size()) return;
    codepoint_t cp = NFD_string[pos];
    // If we have skipped any marks but now find a starter, matching fails.
    if (skip && (combining_class[cp] == 0)) return;
    auto f = t->branches.find(cp);
    // If we didn't find the codepoint, check for a caseless match.
    if ((f == t->branches.end()) && hasOption(mOptions, UCD::Caseless)) {
        if (!selfUC.contains(cp)) {
            std::u32string cp_UC = conv.from_bytes(sUCobj->GetStringValue(cp));
            assert(cp_UC.size() == 1);  // Simple upper case should alway yields a single codepoint.
            cp = cp_UC[0];
        } else if (!selfLC.contains(cp)) {
            std::u32string cp_LC = conv.from_bytes(sLCobj->GetStringValue(cp));
            assert(cp_LC.size() == 1);  // Simple lower case should alway yields a single codepoint.
            cp = cp_LC[0];
        }
        f = t->branches.find(cp);
    }
    if (f != t->branches.end()) {
        // Found a matching character.  Determine a new result position vector.
        std::vector<unsigned> result_positions;
        if ((combining_class[pos] == 0) || positions.empty()) result_positions = {pos};
        else {
            result_positions = positions;
            if ((positions.back() == pos - 1) && (combining_class[pos - 1] == combining_class[pos])) {
                result_positions[positions.size() - 1] = pos;
            } else {
                result_positions.push_back(pos);
            }
        }
        const Trie * subtrie = &(f->second);
        codepoint_t pc = subtrie->getPrecomposed();
        if (pc <= 0x10FFFF) {
            matches.push_back(MatchResult_t{pc, result_positions});
        }
        // There may be more matches; find them, with skip unchanged.
        getMatches(subtrie, NFD_string, combining_class, pos + 1, skip, result_positions, matches);
    }
    // Now check for matches that skip the current character.   This is only
    // possible when matching marks and we have at least one match.
    if (positions.empty() || ccc0set.contains(cp)) return;
    // Matching with gaps:  a precomposed sequence Q M122a can be matched with
    // Q M4 M120 M122a, but not with Q M4 M122b M122a where M122a and M122b have
    // the same CCC.  That is, if one mark is unmatched, then all following marks
    // in the same CCC must remain unmatched.   Note that marks are in nondescending
    // order by the NFD property.
    unsigned ccc = cccObj->GetEnumerationValue(cp);
    pos++;
    while ((pos < NFD_string.size()) && (cccObj->GetEnumerationValue(NFD_string[pos]) == ccc)) {
        pos++;
    }
    getMatches(t, NFD_string, combining_class, pos, /* skip = */ true, positions, matches);
}

    

RE * markCoverageExpression(std::u32string s, std::vector<unsigned> & combining_class, unsigned pos, std::vector<std::pair<unsigned, codepoint_t>> & precomposed_coverage) {
    CC * starter = makeCC(s[pos], &cc::Unicode);
    CC * all_starters = makeCC(s[pos], &cc::Unicode);
    for (auto & a : precomposed_coverage) {
        all_starters->insert(a.second);
    }
    // First determine all marks of the cluster that actually occur, as
    // well as equivalents for the four nonstarter decompositions.
    // 0344 = 0308 0301, 0F73 = 0F71 0F72, 0F75 = 0F71 0F74, 0F81 = 0F71 0F80
    // Note: no reordering possible for 0308 0301 sequences, both in ccc 230.
    CC * allMarksCC = makeCC();
    for (unsigned i = pos + 1; combining_class[i] != 0; i++) {
        allMarksCC->insert(s[i]);
        if ((s[i-1] == 0x308) && (s[i] == 0x301)) allMarksCC->insert(0x344);
    }
    bool has_xF71 = allMarksCC->contains(0xF71);
    if (has_xF71) {
        if (allMarksCC->contains(0xF72)) allMarksCC->insert(0xF73);
        if (allMarksCC->contains(0xF74)) allMarksCC->insert(0xF75);
        if (allMarksCC->contains(0xF80)) allMarksCC->insert(0xF81);
    }
    std::vector<RE *> coverage_terms = {all_starters, makeRep(allMarksCC, 0, Rep::UNBOUNDED_REP)};
    unsigned i = pos + 1; // mark index
    unsigned k = 0;       // precomposed_coverage index
    while (combining_class[i] != 0) {
        unsigned theCCC = combining_class[i];
        // Prepare a coverage expression for all the marks of this CCC.
        CC * combining_classCC = makeCC();
        for (unsigned j = i; combining_class[j] == theCCC; j++) {
            combining_classCC->insert(s[j]);
            if ((s[j-1] == 0x308) && (s[j] == 0x301)) combining_classCC->insert(0x344);
        }
        if (has_xF71) {
            if (combining_classCC->contains(0xF71)) {
                if (allMarksCC->contains(0xF72)) combining_classCC->insert(0xF73);
                if (allMarksCC->contains(0xF74)) combining_classCC->insert(0xF75);
                if (allMarksCC->contains(0xF80)) combining_classCC->insert(0xF81);
            }
            if (combining_classCC->contains(0xF72)) combining_classCC->insert(0xF73);
            if (combining_classCC->contains(0xF74)) combining_classCC->insert(0xF75);
            if (combining_classCC->contains(0xF80)) combining_classCC->insert(0xF81);
        }
        RE * markStar = makeRep(subtractCC(allMarksCC, combining_classCC), 0, Rep::UNBOUNDED_REP);
        std::vector<RE *> mark_terms;
        //if (exactCoverageMode) {
            mark_terms.push_back(starter);
            mark_terms.push_back(markStar);
        //}
        RE * pre308 = nullptr;
        do {
            CC * theMark = makeCC(s[i], &cc::Unicode);
            if (has_xF71) {
                if (s[i] == 0xF71) {
                    if (allMarksCC->contains(0xF72)) theMark->insert(0xF73);
                    if (allMarksCC->contains(0xF74)) theMark->insert(0xF75);
                    if (allMarksCC->contains(0xF80)) theMark->insert(0xF81);
                }
                if (s[i] == 0xF72) theMark->insert(0xF73);
                if (s[i] == 0xF74) theMark->insert(0xF75);
                if (s[i] == 0xF80) theMark->insert(0xF81);
            }
            if (s[i] == 0x308) {
                pre308 = makeSeq(mark_terms.begin(), mark_terms.end());
                mark_terms = std::vector<RE *>{pre308};
                mark_terms.push_back(theMark);
            } else if (pre308 && (s[i] == 0x301)) {
                mark_terms.push_back(theMark);
                RE * prefix = makeSeq(mark_terms.begin(), mark_terms.end());
                mark_terms = std::vector<RE *>{makeAlt({prefix, makeSeq({pre308, makeCC(0x344)})})};
                pre308 = nullptr;
            } else {
                pre308 = nullptr;
                mark_terms.push_back(theMark);
            }
            if ((k < precomposed_coverage.size()) && (precomposed_coverage[k].first == i)) {
                CC * s = makeCC();
                while ((k < precomposed_coverage.size()) && (precomposed_coverage[k].first == i)) {
                    s->insert(precomposed_coverage[k].second);
                    k++;
                }
                RE * prefix = makeSeq(mark_terms.begin(), mark_terms.end());
                mark_terms = std::vector<RE *>{makeAlt({prefix, s})};
            }
            mark_terms.push_back(markStar);
            i++;
        } while (combining_class[i] == theCCC);
        coverage_terms.push_back(makeLookBehindAssertion(makeSeq(mark_terms.begin(), mark_terms.end())));
    }
    return makeSeq(coverage_terms.begin(), coverage_terms.end());
}

// Given a list of match results consisting of (codepoint, position vector) pairs,
// produce a vector of (position, codepoint) pairs sorted by position.  As we
// expect the vectors to be short and the entries to be mostly generated in
// sorted order anyway, we use a simple insertion sort.

std::vector<std::pair<unsigned, codepoint_t>> coverageVector(std::vector<MatchResult_t> matches) {
    std::vector<std::pair<unsigned, codepoint_t>> coverage;
    for (auto & m : matches) {
        codepoint_t cp = m.first;
        for (unsigned k = 1; k < m.second.size(); k++) {
            unsigned posn = m.second[k];
            unsigned i = coverage.size();
            auto pair = std::make_pair(posn, cp);
            coverage.push_back(pair);
            while ((i > 0) && (posn < coverage[i-1].first)) {
                coverage[i] = coverage[i-1];
                i--;
            }
            coverage[i] = pair;
        }
    }
    return coverage;
}

//
//  Given an NFD string, construct an RE that will match all equivalent
//  strings under the given equivalence relation.   Three processes are
//  used:
//  (a) finding all possible precomposed equivalents to substrings within
//      the string.
//  (b) permitting all possible orderings of marks, subject to the constraints
//      of the canonical ordering algorithm, and
//  (c) replacing all characters with their equivalent character class under
//      the given equivalence relation.
//

RE * ClusterMatchTransformer::addEquivalents(std::u32string NFD_string) {
    // Compute a vector of canonical combining class values of each character
    // in the string, with an extra sentinel position, to allow simpl
    // comparisons of CCC values of current and next positions.
    std::vector<unsigned> combining_class(NFD_string.size()+1, 0);
    for (unsigned i = 0; i < NFD_string.size(); i++) {
        if (!ccc0set.contains(NFD_string[i])) {
            combining_class[i] = cccObj->GetEnumerationValue(NFD_string[i]);
        }
    }
    
    RE * prefixRE = makeSeq();
    // A vector of alternative prefix REs by NFD_string position.
    std::vector<std::vector<RE *>> prefixTerms(NFD_string.size());
    
    unsigned i = 0;
    while (i < NFD_string.size()) {
        std::vector<MatchResult_t> matches;
        unsigned cccCount = 0;
        unsigned markCount = 0;
        for (unsigned j = i+1; combining_class[j] != 0; j++) {
            cccCount += combining_class[j] != combining_class[j+1];
            markCount++;
        }
        if (combining_class[i] == 0) {
            find_precomposed(NFD_string, combining_class, i, matches);
        }
        if (cccCount > 1) {
            // We are at a cluster with multiple possible orderings.
            // We use mark coverage expressions to handle all possible orderings
            // as well as precomposed equivalents.
            std::vector<std::pair<unsigned, codepoint_t>> coverage = coverageVector(matches);
            prefixRE = makeSeq({prefixRE, markCoverageExpression(NFD_string, combining_class, i, coverage)});
        } else {
            for (unsigned k = 0; k < matches.size(); k++) {
                CC * precomposedCC = makeCC(matches[k].first, &cc::Unicode);
                unsigned last_cluster_pos = matches[k].second.back();
                prefixTerms[last_cluster_pos].push_back(makeSeq({prefixRE, precomposedCC}));
            }
            for (unsigned j = i; j <= i + markCount; j++) {
                prefixTerms[j].push_back(makeSeq({prefixRE, makeCC(NFD_string[j], &cc::Unicode)}));
                prefixRE = makeAlt(prefixTerms[j].begin(), prefixTerms[j].end());
            }
        }
        i += 1 + markCount;
    }
    return prefixRE;
}


RE * addClusterMatches(RE * r, EquivalenceOptions options) {
    return ClusterMatchTransformer(options).transformRE(r);
}
        
CC * add_equivalent_codepoints(CC * cc, EquivalenceOptions options) {
    if (cc->getAlphabet() != &cc::Unicode) return cc;
    UnicodeSet addedCC;
    for (const interval_t i : *cc) {
        for (codepoint_t cp = lo_codepoint(i); cp <= hi_codepoint(i); cp++) {
            addedCC = addedCC + equivalentCodepoints(cp, options);
        }
    }
    if ((addedCC - *cc).empty()) return cc;
    return makeCC(*cc + addedCC);
}
        

class EquivalentCodepointsTransformer : public re::RE_Transformer {
public:
    EquivalentCodepointsTransformer(EquivalenceOptions opt = Canonical);
protected:
    re::RE * transformCC(re::CC * cc) override;
    re::RE * transformGroup(re::Group * g) override;
private:
    EquivalenceOptions mOptions;
};

EquivalentCodepointsTransformer::EquivalentCodepointsTransformer(UCD::EquivalenceOptions opt) :
    RE_Transformer("EquivalentCodepoints"),
    mOptions(opt)
    {}


RE * EquivalentCodepointsTransformer::transformGroup(Group * g) {
    re::Group::Mode mode = g->getMode();
    re::Group::Sense sense = g->getSense();
    auto r = g->getRE();
    EquivalenceOptions saveOptions = mOptions;
    if (mode == re::Group::Mode::CaseInsensitiveMode) {
        if (sense == re::Group::Sense::On) {
            mOptions = static_cast<EquivalenceOptions>(mOptions | UCD::Caseless);
        } else {
            mOptions = static_cast<EquivalenceOptions>(mOptions & ~UCD::Caseless);
        }
    } else if (mode == re::Group::Mode::CompatibilityMode) {
        if (sense == re::Group::Sense::On) {
            mOptions = static_cast<EquivalenceOptions>(mOptions | UCD::Compatible);
        } else {
            mOptions = static_cast<EquivalenceOptions>(mOptions & ~UCD::Compatible);
        }
    } else {
        RE * t = transform(r);
        // Keep the group/mode info.
        if (t == r) return g;
        return makeGroup(mode, t, sense);
    }
    // Once transformation is complete, the +i or +K modes are
    // no longer needed.
    RE * t = transform(r);
    mOptions = saveOptions;
    if (t == r) return r;
    return t;
}


RE * EquivalentCodepointsTransformer::transformCC(CC * cc) {
    return add_equivalent_codepoints(cc, mOptions);
}

RE * addEquivalentCodepoints(RE * re, UCD::EquivalenceOptions eqopt) {
    return EquivalentCodepointsTransformer(eqopt).transformRE(re);
}

}
