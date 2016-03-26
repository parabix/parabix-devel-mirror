#ifndef UCDCOMPILER_HPP
#define UCDCOMPILER_HPP

#include <re/re_cc.h>
#include <vector>
#include <boost/container/flat_map.hpp>

namespace cc {
    class CC_Compiler;
}

namespace re {
    class Name;
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
    using TargetMap = boost::container::flat_map<const UnicodeSet *, PabloAST *>;
    using Target = std::pair<const UnicodeSet *, PabloAST *>;
    using TargetVector = std::vector<Target>;

    static const RangeList defaultIfHierachy;
    static const RangeList noIfHierachy;

public:

    using NameMap = boost::container::flat_map<re::Name *, PabloAST *>;

    UCDCompiler(cc::CC_Compiler & ccCompiler);

    void generateWithDefaultIfHierarchy(NameMap & names, PabloBuilder & entry);

    void generateWithoutIfHierarchy(NameMap & names, PabloBuilder & entry);

    PabloAST * generateWithDefaultIfHierarchy(const UnicodeSet * set, PabloBuilder & entry);

    PabloAST * generateWithoutIfHierarchy(const UnicodeSet * set, PabloBuilder & entry);

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

    void addTargets(const NameMap & names);

    void updateNames(NameMap & names, PabloBuilder & entry);

private:
    cc::CC_Compiler &       mCharacterClassCompiler;
    PabloAST *              mSuffixVar;
    TargetMap               mTargetMap;
};

}

#endif // UCDCOMPILER_HPP
