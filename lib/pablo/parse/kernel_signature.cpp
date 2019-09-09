/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <pablo/parse/kernel_signature.h>

#include <unordered_map>
#include <llvm/Support/raw_ostream.h>

namespace pablo {
namespace parse {

using AttrKind = kernel::Attribute::KindId;

#define ATTR_ENTRY(ATTRIBUTE, USES_AMOUNT) {"@" #ATTRIBUTE, {AttrKind::ATTRIBUTE, USES_AMOUNT}}

const std::unordered_map<std::string, std::pair<AttrKind, bool>> inputParameterAttributeLookupMap {
    ATTR_ENTRY(LookAhead, true),
    // ATTR_ENTRY(LookBehind, true), // not done
    ATTR_ENTRY(Principal, false),
    ATTR_ENTRY(Deferred, false),
    ATTR_ENTRY(ZeroExtended, false),
    // ATTR_ENTRY(IndependentRegionBegin, true), // not done
    // ATTR_ENTRY(IndependentRegionEnd, true), // not done
    // ATTR_ENTRY(RegionSelector, true), // not done
    // ATTR_ENTRY(SupressNonRegionZeroFill, false), // not done
    ATTR_ENTRY(RequiresPopCountArray, false),
    ATTR_ENTRY(Add, true),
    ATTR_ENTRY(Truncate, true),
    ATTR_ENTRY(Linear, false),
    ATTR_ENTRY(Misaligned, false),
    ATTR_ENTRY(BlockSize, true),
    // ATTR_ENTRY(ReverseAdapter, false), // not done
    // ATTR_ENTRY(SliceOffset, true), // not done
};

const std::unordered_map<std::string, std::pair<AttrKind, bool>> outputParameterAttributeLookupMap {
    ATTR_ENTRY(RoundUpTo, true),
    ATTR_ENTRY(ManagedBuffer, false),
    // ATTR_ENTRY(Expandable, false), // not done
    ATTR_ENTRY(Add, true),
    ATTR_ENTRY(Truncate, true),
    ATTR_ENTRY(Linear, false),
    ATTR_ENTRY(Misaligned, false),
    ATTR_ENTRY(BlockSize, true),
    // ATTR_ENTRY(ReverseAdapter, false), // not done
    // ATTR_ENTRY(SliceOffset, true), // not done
};

const std::unordered_map<std::string, std::pair<AttrKind, bool>> kernelAttributeLookupMap {
    ATTR_ENTRY(CanTerminateEarly, false),
    ATTR_ENTRY(MustExplicitlyTerminate, false),
    ATTR_ENTRY(MayFatallyTerminate, false),
    ATTR_ENTRY(SideEffecting, false),
    ATTR_ENTRY(Family, false),
    ATTR_ENTRY(InternallySynchronized, false),
    ATTR_ENTRY(InfrequentlyUsed, false),
};

boost::optional<kernel::Attribute> attributeFromString(
    std::shared_ptr<ErrorManager> & em,
    Token * attributeToken,
    boost::optional<uint32_t> amount,
    std::unordered_map<std::string, std::pair<AttrKind, bool>> const & lookupTable)
{
    assert(attributeToken->getType() == TokenType::ATTRIBUTE);
    auto const text = attributeToken->getText();
    auto const mapIter = lookupTable.find(text);
    if (mapIter == lookupTable.end()) {
        em->logError(attributeToken, "'" + text + "' is not valid here");
        return boost::none;
    }
    auto const kind = (*mapIter).second.first;
    auto const useAmount = (*mapIter).second.second;
    if (useAmount && !amount) {
        em->logError(attributeToken, "an amount value is required for this attribute", text + "(n)");
        return boost::none;
    }
    if (!useAmount && amount) {
        em->logError(attributeToken, "this attribute does not accept an amount value", "use '" + text + "'");
        return boost::none;
    }

    uint32_t k = 0;
    if (amount) {
        assert(useAmount); // sanity check
        k = *amount;
    }
    return kernel::Attribute(kind, k);
}

boost::optional<kernel::Attribute> inputParameterAttributeFromString(
    std::shared_ptr<ErrorManager> & em,
    Token * attributeToken,
    boost::optional<uint32_t> amount)
{
    return attributeFromString(em, attributeToken, amount, inputParameterAttributeLookupMap);
}

boost::optional<kernel::Attribute> outputParameterAttributeFromString(
    std::shared_ptr<ErrorManager> & em,
    Token * attributeToken,
    boost::optional<uint32_t> amount)
{
    return attributeFromString(em, attributeToken, amount, outputParameterAttributeLookupMap);
}

boost::optional<kernel::Attribute> kernelAttributeFromString(
    std::shared_ptr<ErrorManager> & em,
    Token * attributeToken,
    boost::optional<uint32_t> amount)
{
    return attributeFromString(em, attributeToken, amount, kernelAttributeLookupMap);
}

std::string attributeToString(kernel::Attribute attr) {
    return "???";
}

std::string PabloKernelSignature::asString() const noexcept {
    std::string str;
    llvm::raw_string_ostream out(str);
    out << "kernel " << getName() << " :: [";
    for (size_t i = 0; i < getInputBindings().size(); ++i) {
        std::string name;
        pablo::parse::PabloType * type;
        auto binding = getInputBindings()[i];
        out << binding.type->asString() << " " << binding.name;
        if (i != getInputBindings().size() - 1) {
            out << ", ";
        }
    }
    out << "] -> [";
    for (size_t i = 0; i < getOutputBindings().size(); ++i) {
        std::string name;
        pablo::parse::PabloType * type;
        auto binding = getOutputBindings()[i];
        out << binding.type->asString() << " " << binding.name;
        if (i != getOutputBindings().size() - 1) {
            out << ", ";
        }
    }
    out << "]";
    return out.str();
}

} // namespace pablo::parse
} // namespace pablo
