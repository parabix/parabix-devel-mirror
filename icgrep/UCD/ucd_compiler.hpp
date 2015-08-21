#ifndef UCDCOMPILER_HPP
#define UCDCOMPILER_HPP

#include <vector>
#include <re/re_cc.h>

namespace cc {
    class CC_Compiler;
}

namespace pablo {
    class PabloBuilder;
    class PabloAST;
}

namespace UCD {

class UnicodeSet;

class UCDCompiler {

    using CC = re::CC;
    using PabloBuilder = pablo::PabloBuilder;
    using PabloAST = pablo::PabloAST;
    using codepoint_t = re::codepoint_t;
    using RangeList = std::vector<re::interval_t>;

public:
    UCDCompiler(cc::CC_Compiler & ccCompiler);

    PabloAST * generateWithDefaultIfHierarchy(const UnicodeSet & set, PabloBuilder & entry);

    PabloAST * generateWithoutIfHierarchy(const UnicodeSet & set, PabloBuilder & entry);

    PabloAST * generateWithIfHierarchy(const RangeList & ifRanges, const UnicodeSet & set, PabloBuilder & entry);

protected:

    PabloAST * generateWithIfHierarchy(const RangeList & ifRanges, const UnicodeSet & set, const codepoint_t lo, const codepoint_t hi, PabloBuilder & builder);

    PabloAST * generateSubRanges(const UnicodeSet & set, const codepoint_t lo, const codepoint_t hi, PabloBuilder & builder, PabloAST * target);

    PabloAST * sequenceGenerator(const RangeList && ranges, const unsigned byte_no, PabloBuilder & builder, PabloAST * target, PabloAST * prefix);

    PabloAST * sequenceGenerator(const codepoint_t lo, const codepoint_t hi, const unsigned byte_no, PabloBuilder & builder, PabloAST * target, PabloAST * prefix);

    PabloAST * ifTestCompiler(const codepoint_t lo, const codepoint_t hi, PabloBuilder & builder);

    PabloAST * ifTestCompiler(const codepoint_t lo, const codepoint_t hi, const unsigned byte_no, PabloBuilder & builder, PabloAST * target);

    PabloAST * makePrefix(const codepoint_t cp, const unsigned byte_no, PabloBuilder & builder, PabloAST * prefix);

    static RangeList byteDefinitions(const RangeList & list, const unsigned byte_no);

    template <typename RangeListOrUnicodeSet>
    static RangeList rangeIntersect(const RangeListOrUnicodeSet & list, const codepoint_t lo, const codepoint_t hi);

    static RangeList rangeGaps(const RangeList & list, const codepoint_t lo, const codepoint_t hi);

    static RangeList outerRanges(const RangeList & list);

    static RangeList innerRanges(const RangeList & list);

private:
    cc::CC_Compiler &       mCharacterClassCompiler;
    PabloAST *              mSuffixVar;
};

}

#endif // UCDCOMPILER_HPP
