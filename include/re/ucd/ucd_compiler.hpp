#ifndef UCDCOMPILER_HPP
#define UCDCOMPILER_HPP

#include <unicode/core/UCD_Config.h>

#include <vector>
#include <boost/container/flat_map.hpp>

namespace cc {
    class CC_Compiler;
}

namespace re {
    class Name;
    class CC;
}

namespace pablo {
    class PabloBuilder;
    class PabloAST;
    class Var;
}

namespace UCD {

class UnicodeSet;

class UCDCompiler {

    using CC = re::CC;
    using PabloBuilder = pablo::PabloBuilder;
    using PabloAST = pablo::PabloAST;
    using RangeList = std::vector<interval_t>;

    using TargetMap = boost::container::flat_map<const CC *, pablo::Var *>;
    using ValueMap = boost::container::flat_map<const CC *, PabloAST *>;
    using Values = std::vector<std::pair<ValueMap::key_type, ValueMap::mapped_type>>;

    static const RangeList defaultIfHierachy;
    static const RangeList noIfHierachy;

public:

    using NameMap = boost::container::flat_map<re::Name *, PabloAST *>;

    UCDCompiler(cc::CC_Compiler & ccCompiler);

    void numberOfInputStreams(const unsigned NumOfStreams);

    void generateWithDefaultIfHierarchy(NameMap & names, PabloBuilder & entry);

    void generateWithoutIfHierarchy(NameMap & names, PabloBuilder & entry);

protected:

    void generateRange(const RangeList & ifRanges, PabloBuilder & entry);

    void generateRange(const RangeList & ifRanges, const codepoint_t lo, const codepoint_t hi, PabloBuilder & builder);

    void generateSubRanges(const codepoint_t lo, const codepoint_t hi, PabloBuilder & builder);

    PabloAST * sequenceGenerator(const RangeList && ranges, const unsigned byte_no, PabloBuilder & builder, PabloAST * target, PabloAST * prefix);

    PabloAST * sequenceGenerator(const codepoint_t lo, const codepoint_t hi, const unsigned byte_no, PabloBuilder & builder, PabloAST * target, PabloAST * prefix);

    PabloAST * ifTestCompiler(const codepoint_t lo, const codepoint_t hi, PabloBuilder & builder);

    PabloAST * ifTestCompiler(const codepoint_t lo, const codepoint_t hi, const unsigned byte_no, PabloBuilder & builder, PabloAST * target);

    PabloAST * makePrefix(const codepoint_t cp, const unsigned byte_no, PabloBuilder & builder, PabloAST * prefix);

    static RangeList byteDefinitions(const RangeList & list, const unsigned byte_no, bool isUTF_16);

    template <typename RangeListOrUnicodeSet>
    static RangeList rangeIntersect(const RangeListOrUnicodeSet & list, const codepoint_t lo, const codepoint_t hi);

    static RangeList rangeGaps(const RangeList & list, const codepoint_t lo, const codepoint_t hi);

    static RangeList outerRanges(const RangeList & list);

    static RangeList innerRanges(const RangeList & list);

    void makeTargets(PabloBuilder & entry, NameMap & names);

private:
    cc::CC_Compiler &       mCodeUnitCompiler;
    PabloAST *              mSuffixVar;
    TargetMap               mTarget;
    ValueMap                mTargetValue;
    unsigned                mNumOfStreams;
};

}

#endif // UCDCOMPILER_HPP
