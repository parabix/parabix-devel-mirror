/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <grep/resolve_properties.h>

#include <string>
#include <grep/grep_engine.h>
#include <llvm/Support/Casting.h>
#include <re/adt/adt.h>
#include <re/parse/parser.h>
#include <re/analysis/re_analysis.h>
#include <re/unicode/re_name_resolve.h>
#include <re/unicode/resolve_properties.h>
#include <kernel/pipeline/driver/cpudriver.h>
#include <unicode/data/PropertyAliases.h>
#include <unicode/data/PropertyObjectTable.h>
#include <util/aligned_allocator.h>

using namespace llvm;

namespace grep {

class PropertyValueAccumulator : public grep::MatchAccumulator {
public:

    PropertyValueAccumulator(std::vector<std::string> & accumulatedPropertyValues)
    : mParsedPropertyValueSet(accumulatedPropertyValues) {}

    void accumulate_match(const size_t lineNum, char * line_start, char * line_end) override;
private:
    std::vector<std::string> & mParsedPropertyValueSet;
};

void PropertyValueAccumulator::accumulate_match(const size_t lineNum, char * line_start, char * line_end) {
    assert (line_start <= line_end);
    mParsedPropertyValueSet.emplace_back(line_start, line_end);
}



class SetByLineNumberAccumulator : public grep::MatchAccumulator {
public:

    SetByLineNumberAccumulator(const std::vector<UCD::codepoint_t> & cps, const UCD::UnicodeSet & defaultValueSet)
    : mCodepointTableByLineNum(cps)
    , mDefaultValueSet(defaultValueSet) {

    }

    void accumulate_match(const size_t lineNum, char * line_start, char * line_end) override;
    UCD::UnicodeSet && getAccumulatedSet() { return std::move(mAccumSet); }
private:
    const std::vector<UCD::codepoint_t> & mCodepointTableByLineNum;
    const UCD::UnicodeSet & mDefaultValueSet;
    UCD::UnicodeSet mAccumSet;
};

void SetByLineNumberAccumulator::accumulate_match(const size_t lineNum, char * /* line_start */, char * /* line_end */) {
    if (lineNum >= mCodepointTableByLineNum.size()) {
        mAccumSet.insert(mDefaultValueSet);
    } else {
        mAccumSet.insert(mCodepointTableByLineNum[lineNum]);
    }
}



const UCD::UnicodeSet GetCodepointSetMatchingPattern(UCD::PropertyObject * propObj, re::RE * pattern) {
    if (auto p = dyn_cast<UCD::BinaryPropertyObject>(propObj)) {
        return GetCodepointSetMatchingPattern(p, pattern);
    } else if (auto p = dyn_cast<UCD::EnumeratedPropertyObject>(propObj)) {
        return GetCodepointSetMatchingPattern(p, pattern);
    } else if (auto p = dyn_cast<UCD::ExtensionPropertyObject>(propObj)) {
        return GetCodepointSetMatchingPattern(p, pattern);
    } else if (auto p = dyn_cast<UCD::NumericPropertyObject>(propObj)) {
        return GetCodepointSetMatchingPattern(p, pattern);
    } else if (auto p = dyn_cast<UCD::StringPropertyObject>(propObj)) {
        return GetCodepointSetMatchingPattern(p, pattern);
    } else if (auto p = dyn_cast<UCD::StringOverridePropertyObject>(propObj)) {
        return GetCodepointSetMatchingPattern(p, pattern);
    } else {
        llvm::report_fatal_error("grep::GetCodepointSetMatchingPattern(UCD::PropertyObject *, re::RE *): operation is unsupported");
    }
}

const UCD::UnicodeSet GetCodepointSetMatchingPattern(UCD::BinaryPropertyObject * propObj, re::RE * pattern) {
    llvm::report_fatal_error("grep::GetCodepointSetMatchingPattern(UCD::BinaryPropertyObject *, re::RE *): not yet implemented");
}

const UCD::UnicodeSet GetCodepointSetMatchingPattern(UCD::EnumeratedPropertyObject * propObj, re::RE * pattern) {
    AlignedAllocator<char, 32> alloc;
    std::vector<std::string> accumulatedValues;

    const std::string & str = propObj->GetPropertyValueGrepString();

    const unsigned segmentSize = 8;
    const auto n = str.length();
    const auto w = 256 * segmentSize;
    const auto m = w - (n % w);

    char * aligned = alloc.allocate(n + m, 0);
    std::memcpy(aligned, str.data(), n);
    std::memset(aligned + n, 0, m);

    PropertyValueAccumulator accum(accumulatedValues);
    CPUDriver driver("driver");
    grep::InternalSearchEngine engine(driver);
    engine.setRecordBreak(grep::GrepRecordBreakKind::LF);
    engine.grepCodeGen(pattern);
    engine.doGrep(aligned, n, accum);
    alloc.deallocate(aligned, 0);

    UCD::UnicodeSet a;
    for (const auto & v : accumulatedValues) {
        const auto e = propObj->GetPropertyValueEnumCode(v);
        a.insert(propObj->GetCodepointSet(e));
    }
    return a;
}

const UCD::UnicodeSet GetCodepointSetMatchingPattern(UCD::ExtensionPropertyObject * propObj, re::RE * pattern) {
    AlignedAllocator<char, 32> alloc;
    std::vector<std::string> accumulatedValues;

    UCD::EnumeratedPropertyObject * baseObj = llvm::cast<UCD::EnumeratedPropertyObject>(UCD::property_object_table[propObj->base_property]);

    const std::string & str = baseObj->GetPropertyValueGrepString();

    const unsigned segmentSize = 8;
    const auto n = str.length();
    const auto w = 256 * segmentSize;
    const auto m = w - (n % w);

    char * aligned = alloc.allocate(n + m, 0);
    std::memcpy(aligned, str.data(), n);
    std::memset(aligned + n, 0, m);

    PropertyValueAccumulator accum(accumulatedValues);
    CPUDriver driver("driver");
    grep::InternalSearchEngine engine(driver);
    engine.setRecordBreak(grep::GrepRecordBreakKind::LF);
    engine.grepCodeGen(pattern);
    engine.doGrep(aligned, n, accum);
    alloc.deallocate(aligned, 0);

    UCD::UnicodeSet a;
    for (const auto & v : accumulatedValues) {
        int e = baseObj->GetPropertyValueEnumCode(v);
        a.insert(propObj->GetCodepointSet(e));
    }
    return a;
}

const UCD::UnicodeSet GetCodepointSetMatchingPattern(UCD::NumericPropertyObject * propObj, re::RE * pattern) {
    SetByLineNumberAccumulator accum(propObj->mExplicitCps, propObj->mNaNCodepointSet);
    CPUDriver driver("driver");
    grep::InternalSearchEngine engine(driver);
    engine.setRecordBreak(grep::GrepRecordBreakKind::LF);
    engine.grepCodeGen(pattern);
    engine.doGrep(propObj->mStringBuffer, propObj->mBufSize, accum);
    return accum.getAccumulatedSet();
}

const UCD::UnicodeSet GetCodepointSetMatchingPattern(UCD::StringPropertyObject * propObj, re::RE * pattern) {
    UCD::UnicodeSet matched(*re::matchableCodepoints(pattern) & propObj->mSelfCodepointSet);
    if (re::matchesEmptyString(pattern)) {
        matched.insert(propObj->mNullCodepointSet);
    }
    SetByLineNumberAccumulator accum(propObj->mExplicitCps, propObj->mNullCodepointSet);
    CPUDriver driver("driver");
    grep::InternalSearchEngine engine(driver);
    engine.setRecordBreak(grep::GrepRecordBreakKind::LF);
    engine.grepCodeGen(pattern);
    const unsigned bufSize = propObj->mStringOffsets[propObj->mExplicitCps.size()];
    engine.doGrep(propObj->mStringBuffer, bufSize, accum);
    matched.insert(accum.getAccumulatedSet());
    return matched;
}

const UCD::UnicodeSet GetCodepointSetMatchingPattern(UCD::StringOverridePropertyObject * propObj, re::RE * pattern) {
    UCD::UnicodeSet base_set = GetCodepointSetMatchingPattern(UCD::getPropertyObject(propObj->mBaseProperty), pattern)
                             - propObj->mOverriddenSet;
    SetByLineNumberAccumulator accum(propObj->mExplicitCps, UCD::UnicodeSet());
    CPUDriver driver("driver");
    grep::InternalSearchEngine engine(driver);
    engine.setRecordBreak(grep::GrepRecordBreakKind::LF);
    engine.grepCodeGen(pattern);
    const unsigned bufSize = propObj->mStringOffsets[propObj->mExplicitCps.size()];
    engine.doGrep(propObj->mStringBuffer, bufSize, accum);
    base_set.insert(accum.getAccumulatedSet());
    return base_set;
}



UCD::UnicodeSet resolveUnicodeSet(re::Name * const name) {
    if (name->getType() == re::Name::Type::UnicodeProperty) {
        std::string prop = name->getNamespace();
        std::string value = name->getName();
        if (prop.length() > 0 && value.length() > 0 && value[0] == '/') {
            prop = UCD::canonicalize_value_name(prop);
            auto propCode = UCD::getPropertyCode(prop);
            if (propCode == UCD::Undefined) {
                UCD::UnicodePropertyExpressionError("Expected a property name, but '" + name->getNamespace() + "' found instead");
            }
            auto propObj = UCD::getPropertyObject(propCode);

            // resolve regular expression
            re::RE * propValueRe = re::RE_Parser::parse(value.substr(1), re::DEFAULT_MODE, re::PCRE, false);
            propValueRe = re::resolveUnicodeNames(propValueRe); // Recursive name resolution may be required
            return GetCodepointSetMatchingPattern(propObj, propValueRe);
        } else {
            return UCD::resolveUnicodeSet(name);
        }
    } else {
        return UCD::resolveUnicodeSet(name);
    }
}

} // namespace grep
