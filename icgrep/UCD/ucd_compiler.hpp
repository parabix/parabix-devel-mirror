#ifndef UCDCOMPILER_HPP
#define UCDCOMPILER_HPP

#include <vector>
#include <cc/cc_compiler.h>
#include <re/re_cc.h>
#include <UCD/unicode_set.h>
#include <unordered_map>

class Encoding;

namespace UCD {

class UCDCompiler {

    using CC = re::CC;
    using PabloBuilder = pablo::PabloBuilder;
    using PabloAST = pablo::PabloAST;
    using codepoint_t = re::codepoint_t;
    using RangeList = std::vector<re::interval_t>;

public:
    UCDCompiler(cc::CC_Compiler & ccCompiler);

    PabloAST * generateWithDefaultIfHierarchy(const UnicodeSet & set, PabloBuilder & entry);

    PabloAST * generateWithIfHierarchy(const RangeList & ifRanges, const UnicodeSet & set, PabloBuilder & entry);

protected:

    PabloAST * generateWithIfHierarchy(const RangeList & ifRanges, const UnicodeSet & set, const codepoint_t lo, const codepoint_t hi, PabloBuilder & block);

    PabloAST * generateSubRanges(const UnicodeSet & set, const codepoint_t lo, const codepoint_t hi, PabloBuilder & block, PabloAST * target);

    PabloAST * sequenceGenerator(const RangeList && ranges, const unsigned byte_no, PabloBuilder & block, PabloAST * target, PabloAST * prefix);

    PabloAST * sequenceGenerator(const codepoint_t lo, const codepoint_t hi, const unsigned byte_no, PabloBuilder & block, PabloAST * target, PabloAST * prefix);

    PabloAST * ifTestCompiler(const codepoint_t lo, const codepoint_t hi, PabloBuilder & block);
    PabloAST * ifTestCompiler(const codepoint_t lo, const codepoint_t hi, const unsigned byte_no, PabloBuilder & block, PabloAST * target);

    PabloAST * makePrefix(const codepoint_t cp, const unsigned byte_no, PabloBuilder & pb);

    static RangeList byteDefinitions(const RangeList & list, const unsigned byte_no);

    template <typename RangeListOrUnicodeSet>
    static RangeList rangeIntersect(const RangeListOrUnicodeSet & list, const codepoint_t lo, const codepoint_t hi);

    static RangeList rangeGaps(const RangeList & list, const codepoint_t lo, const codepoint_t hi);

    static RangeList outerRanges(const RangeList & list);

    static RangeList innerRanges(const RangeList & list);

private:
    cc::CC_Compiler &       mCharacterClassCompiler;
    CC * const              mSuffix;
};

}

#endif // UCDCOMPILER_HPP
