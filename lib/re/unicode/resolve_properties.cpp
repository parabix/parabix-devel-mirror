/*
 *  Copyright (c) 2020 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */
#include <re/unicode/resolve_properties.h>

#include <llvm/Support/ErrorHandling.h>
#include <llvm/Support/raw_ostream.h>
#include <re/adt/adt.h>
#include <re/adt/re_name.h>
#include <re/parse/parser.h>
#include <re/compile/re_compiler.h>
#include <re/unicode/boundaries.h>
#include <unicode/data/PropertyAliases.h>
#include <unicode/data/PropertyObjects.h>
#include <unicode/data/PropertyObjectTable.h>
#include <unicode/data/PropertyValueAliases.h>
#include <util/aligned_allocator.h>

using namespace UCD;
using namespace re;
using namespace llvm;

namespace UCD {
    
void UnicodePropertyExpressionError(std::string errmsg) {
    llvm::report_fatal_error(errmsg);
}

struct PropertyResolver : public RE_Transformer {
    PropertyResolver(GrepLinesFunctionType grep) : RE_Transformer("PropertyResolver"), mGrep(grep) {}
    RE * transformPropertyExpression (PropertyExpression * exp) override;
    RE * resolveCC(std::string val, bool is_negated);
    RE * resolveBoundary(std::string val, bool is_negated);
    
    
private:
    GrepLinesFunctionType mGrep;
    int mPropCode;
    PropertyObject * mPropObj;
    
};
    
RE * PropertyResolver::resolveCC (std::string value, bool is_negated) {
    RE * resolved = nullptr;
    if ((value.length() > 0) && (value[0] == '/')) {
        if (mGrep == nullptr)
            llvm::report_fatal_error("Recursive property expression found, but no grep function supplied");
        re::RE * propValueRe = re::RE_Parser::parse(value.substr(1), re::DEFAULT_MODE, re::PCRE, false);
        return makeCC(mPropObj->GetCodepointSetMatchingPattern(propValueRe, mGrep), &cc::Unicode);
    }
    else if ((value.length() > 0) && (value[0] == '@')) {
        // resolve a @property@ or @identity@ expression.
        std::string otherProp = canonicalize_value_name(value.substr(1));
        auto propObj2 = getPropertyObject(getPropertyCode(otherProp));
        resolved = makeCC(mPropObj->GetPropertyIntersection(propObj2), &cc::Unicode);
    } else {
        resolved = makeCC(mPropObj->GetCodepointSet(value), &cc::Unicode);
    }
    if (is_negated) {
        resolved = makeDiff(makeAny(), resolved);
    }
    return resolved;
}

RE * PropertyResolver::resolveBoundary (std::string val, bool is_negated) {
    RE * resolved = nullptr;
    if (mPropCode == UCD::g) { // Grapheme cluster boundary
        resolved = generateGraphemeClusterBoundaryRule();
        if (is_negated) {
            resolved = makeDiff(makeAny(), resolved);
        }
    } else if (mPropCode == UCD::w) { // Unicode word boundary
        UnicodePropertyExpressionError("\\b{w} not yet supported.");
    } else if (isa<EnumeratedPropertyObject>(mPropObj) && (val == "")) {
        // Boundary between codepoints with any two different values for an
        // enumerated property.
        // TODO:  Pass in the operator, so that negated boundaries are generated in simplified form.
        resolved = EnumeratedPropertyBoundary(cast<EnumeratedPropertyObject>(mPropObj));
        if (is_negated) {
            resolved = makeDiff(makeAny(), resolved);
        }
    } else {
        std::string propName = getPropertyFullName(static_cast<property_t>(mPropCode));
        PropertyExpression * codepointProp = makePropertyExpression(propName, val);
        codepointProp->setPropertyCode(mPropCode);
        codepointProp->setResolvedRE(resolveCC(val, false));
        RE * a = makeLookAheadAssertion(codepointProp);
        RE * na = makeNegativeLookAheadAssertion(codepointProp);
        RE * b = makeLookBehindAssertion(codepointProp);
        RE * nb = makeNegativeLookBehindAssertion(codepointProp);
        if (is_negated) {
            resolved = makeAlt({makeSeq({b, a}), makeSeq({nb, na})});
        } else {
            resolved = makeAlt({makeSeq({b, na}), makeSeq({nb, a})});
        }
    }
    return resolved;
}

RE * PropertyResolver::transformPropertyExpression (PropertyExpression * exp) {
    mPropCode = exp->getPropertyCode();
    PropertyExpression::Operator op = exp->getOperator();
    std::string val = exp->getValueString();
    if (mPropCode < 0) {
        UnicodePropertyExpressionError("Property '" + exp->getPropertyIdentifier() + "' unlinked");
    }
    mPropObj = getPropertyObject(static_cast<UCD::property_t>(mPropCode));
    if (exp->getKind() == PropertyExpression::Kind::Boundary) {
        exp->setResolvedRE(resolveBoundary(val, op == PropertyExpression::Operator::NEq));
    } else {
        exp->setResolvedRE(resolveCC(val, op == PropertyExpression::Operator::NEq));
    }
    return exp;
}

RE * resolveProperties(RE * r, GrepLinesFunctionType grep) {
    return PropertyResolver(grep).transformRE(r);
}

    
struct PropertyLinker : public RE_Transformer {
    PropertyLinker() : RE_Transformer("PropertyLinker") {}
    RE * transformPropertyExpression (PropertyExpression * exp) override {
        std::string id = exp->getPropertyIdentifier();
        std::string canon = UCD::canonicalize_value_name(id);
        // In the case of a property expression without a value,
        // we may have a general category, script or some other special cases.
        if (exp->getValueString() == "") {
            const auto & gcObj = cast<EnumeratedPropertyObject>(getPropertyObject(gc));
            int valcode = gcObj->GetPropertyValueEnumCode(canon);
            if (valcode >= 0) {
                // Found a general category.
                exp->setValueString(gcObj->GetValueFullName(valcode));
                exp->setPropertyIdentifier(getPropertyFullName(gc));
                exp->setPropertyCode(gc);
                return exp;
            }
            const auto & scObj = cast<EnumeratedPropertyObject>(getPropertyObject(sc));
            valcode = scObj->GetPropertyValueEnumCode(canon);
            if (valcode >= 0) {
                // Found a script.
                exp->setValueString(scObj->GetValueFullName(valcode));
                exp->setPropertyIdentifier(getPropertyEnumName(sc));
                exp->setPropertyCode(sc);
                return exp;
            }
            if (canon == "ascii") {  // block:ascii special case
                exp->setValueString("ascii");
                exp->setPropertyIdentifier(getPropertyEnumName(blk));
                exp->setPropertyCode(blk);
                return exp;
            }
            if (canon == "assigned") {  // cn:n special case
                // general category != unassigned
                exp->setValueString("unassigned");
                exp->setPropertyIdentifier(getPropertyEnumName(gc));
                exp->setOperator(PropertyExpression::Operator::NEq);
                exp->setPropertyCode(gc);
                return exp;
            }
            if (canon == "any") return makeAny();
        }
        auto propCode = UCD::getPropertyCode(canon);
        if (propCode != UCD::Undefined) {
            exp->setPropertyCode(propCode);
            return exp;
        }
        return exp;
    }
};

RE * linkProperties(RE * r) {
    return PropertyLinker().transformRE(r);
}

struct PropertyStandardization : public RE_Transformer {
    PropertyStandardization() : RE_Transformer("PropertyStandardization") {}
    RE * transformPropertyExpression (PropertyExpression * exp) override {
        int prop_code = exp->getPropertyCode();
        if (prop_code < 0) return exp;  // No property code - leave unchanged.
        PropertyExpression::Operator op = exp->getOperator();
        std::string val_str = exp->getValueString();
        std::string canon = UCD::canonicalize_value_name(val_str);
        auto * propObj = getPropertyObject(static_cast<UCD::property_t>(prop_code));
        if (auto * obj = dyn_cast<EnumeratedPropertyObject>(propObj)) {
            int val_code = obj->GetPropertyValueEnumCode(canon);
            int enum_count = obj->GetEnumCount();
            bool lt_0 = (op == PropertyExpression::Operator::Less) && (val_code == 0);
            bool gt_count = (op == PropertyExpression::Operator::Greater) && (val_code == enum_count);
            if (lt_0 || gt_count) {
                // Impossible property.
                return makeAlt();
            }
            bool ge_0 = (op == PropertyExpression::Operator::GEq) && (val_code == 0);
            bool le_count = (op == PropertyExpression::Operator::LEq) && (val_code == enum_count);
            if (ge_0 || le_count) {  // always true cases
                return makeAny();
            }
            bool le_0 = (op == PropertyExpression::Operator::LEq) && (val_code == 0);
            bool ge_count = (op == PropertyExpression::Operator::GEq) && (val_code == enum_count);
            if (val_code < 0) return exp;
            if (le_0 || ge_count) {  // Equality cases
                exp->setOperator(PropertyExpression::Operator::Eq);
            } else if (op == PropertyExpression::Operator::LEq) {
                // Standardize to Less.
                val_code += 1;
                exp->setOperator(PropertyExpression::Operator::Less);
            } else if (op == PropertyExpression::Operator::GEq) {
                val_code -= 1;
                exp->setOperator(PropertyExpression::Operator::Greater);
            }
            exp->setValueString(obj->GetValueFullName(val_code));
            return exp;
        }
        if (auto * obj = dyn_cast<BinaryPropertyObject>(propObj)) {
            int val_code = obj->GetPropertyValueEnumCode(canon);
            // Standardize binary properties to positive form with an empty value string.
            if (val_code < 0) return exp;
            bool lt_F = (op == PropertyExpression::Operator::Less) && (val_code == 0);
            bool lt_T = (op == PropertyExpression::Operator::Less) && (val_code == 1);
            bool le_F = (op == PropertyExpression::Operator::LEq) && (val_code == 0);
            bool le_T = (op == PropertyExpression::Operator::LEq) && (val_code == 1);
            //bool gt_F = (op == PropertyExpression::Operator::Greater) && (val_code == 0);
            bool gt_T = (op == PropertyExpression::Operator::Greater) && (val_code == 1);
            bool ge_F = (op == PropertyExpression::Operator::GEq) && (val_code == 0);
            //bool ge_T = (op == PropertyExpression::Operator::GEq) && (val_code == 1);
            if (le_T || ge_F) {
                // All possible enum values.   Standardize to \p{any}.
                return makeAny();
            }
            if (lt_F || gt_T) {
                // No values.   return failure.
                return makeAlt();
            }
            bool eq_F = (op == PropertyExpression::Operator::Eq) && (val_code == 0);
            //bool eq_T = (op == PropertyExpression::Operator::Eq) && (val_code == 1);
            //bool ne_F = (op == PropertyExpression::Operator::NEq) && (val_code == 0);
            bool ne_T = (op == PropertyExpression::Operator::NEq) && (val_code == 1);
            if (lt_T || le_F || eq_F || ne_T) {
                // negated property.
                exp->setOperator(PropertyExpression::Operator::NEq);
                exp->setValueString("");
            } else { /*  if (gt_F || ge_T || eq_T || ne_F)  positive properties.  */
                exp->setOperator(PropertyExpression::Operator::Eq);
                exp->setValueString("");
            }
            return exp;
        }
        return exp;
    }
};

RE * standardizeProperties(RE * r) {
    return PropertyStandardization().transformRE(r);
}

struct SimplePropertyInliner : public RE_Transformer {
    const unsigned MAX_CC_SIZE_TO_INLINE = 1;
    const unsigned MAX_BOUNDARY_ALTS_TO_INLINE = 2;
    SimplePropertyInliner() : RE_Transformer("SimplePropertyInliner") {}
    RE * transformPropertyExpression (PropertyExpression * exp) override {
        if (exp->getKind() == PropertyExpression::Kind::Codepoint) {
            if (CC * cc = dyn_cast<CC>(exp->getResolvedRE())) {
                if (cc->size() <= MAX_CC_SIZE_TO_INLINE) {
                    return cc;
                }
            }
        }
        else {
            if (Alt * a = dyn_cast<Alt>(exp->getResolvedRE())) {
                if (a->size() <= MAX_BOUNDARY_ALTS_TO_INLINE) {
                    return a;
                }
            }
        }
        return exp;
    }
};

RE * inlineSimpleProperties(RE * r) {
    return SimplePropertyInliner().transformRE(r);
}

struct PropertyExternalizer : public RE_Transformer {
    PropertyExternalizer() : RE_Transformer("PropertyExternalizer") {}
    RE * transformPropertyExpression (PropertyExpression * exp) override {
        PropertyExpression::Operator op = exp->getOperator();
        std::string id = exp->getPropertyIdentifier();
        std::string val_str = exp->getValueString();
        std::string op_str = "";
        if (op == PropertyExpression::Operator::Less) op_str = "<";
        else if (op == PropertyExpression::Operator::LEq) op_str = "<=";
        else if (op == PropertyExpression::Operator::Greater) op_str = ">";
        else if (op == PropertyExpression::Operator::GEq) op_str = ">=";
        else if (op == PropertyExpression::Operator::NEq) op_str = "!=";
        val_str = op_str + val_str;
        Name * externName;
        if (exp->getKind() == PropertyExpression::Kind::Codepoint) {
            if (val_str == "")
                externName = makeName(id, Name::Type::UnicodeProperty);
            else externName = makeName(id, val_str, Name::Type::UnicodeProperty);
        } else {
            id = "\\b{" + id + "}";
            if (val_str == "" ) externName = makeName(id, Name::Type::ZeroWidth);
            else externName = makeName(id, val_str, Name::Type::ZeroWidth);
        }
        externName->setDefinition(exp->getResolvedRE());
        return externName;
    }
};

RE * externalizeProperties(RE * r) {
    return PropertyExternalizer().transformRE(r);
}

RE * linkAndResolve(RE * r, GrepLinesFunctionType grep) {
    RE * linked = linkProperties(r);
    RE * std = standardizeProperties(linked);
    return resolveProperties(std, grep);
}


}
