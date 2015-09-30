#ifndef UCDCOMPILER_HPP
#define UCDCOMPILER_HPP

#include <re/re_cc.h>
#include <vector>
#ifdef USE_BOOST
#include <boost/container/flat_map.hpp>
#else
#include <unordered_map>
#endif

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
    #ifdef USE_BOOST
    using TargetMap = boost::container::flat_map<const UnicodeSet *, PabloAST *>;
    #else
    using TargetMap = std::unordered_map<const UnicodeSet *, PabloAST *>;
    #endif
    using Target = std::pair<const UnicodeSet *, PabloAST *>;
    using TargetVector = std::vector<Target>;

public:
    UCDCompiler(cc::CC_Compiler & ccCompiler);

    std::vector<PabloAST *> generateWithDefaultIfHierarchy(const std::vector<UnicodeSet> &sets, PabloBuilder & entry);

    std::vector<PabloAST *> generateWithoutIfHierarchy(const std::vector<UnicodeSet> & sets, PabloBuilder & entry);

protected:

    void generateRange(const RangeList & ifRanges, PabloBuilder & entry);

    void generateRange(const RangeList & ifRanges, const codepoint_t lo, const codepoint_t hi, PabloBuilder & builder);

    void generateSubRanges(const codepoint_t lo, const codepoint_t hi, PabloBuilder & builder);

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

    void addTargets(const std::vector<UnicodeSet> & sets);

    std::vector<PabloAST *> returnMarkers(const std::vector<UnicodeSet> &sets) const;

private:
    cc::CC_Compiler &       mCharacterClassCompiler;
    PabloAST *              mSuffixVar;
    TargetMap               mTargetMap;
};

}

#endif // UCDCOMPILER_HPP
