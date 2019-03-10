#include "attributes.h"

#include <llvm/Support/raw_ostream.h>
#include <boost/preprocessor/stringize.hpp>

namespace kernel {

void Attribute::print(llvm::raw_ostream & out) const noexcept {
    #define NAME(DEF) \
        case KindId::DEF : out << BOOST_PP_STRINGIZE(DEF); break
    #define NAME_AMOUNT(DEF) \
        case KindId::DEF : out << BOOST_PP_STRINGIZE(DEF) << mAmount; break
    switch (getKind()) {
        NAME_AMOUNT(LookAhead);
        NAME_AMOUNT(LookBehind);
        NAME(Principal);
        NAME(Deferred);
        NAME(ZeroExtended);
        NAME_AMOUNT(IndependentRegionBegin); NAME_AMOUNT(IndependentRegionEnd);
        NAME_AMOUNT(RegionSelector);
        NAME(SupressNonRegionZeroFill);
        NAME(RequiresPopCountArray); NAME(RequiresNegatedPopCountArray);
        NAME_AMOUNT(Add);
        NAME_AMOUNT(RoundUpTo);
        NAME(ManagedBuffer);
        NAME(Misaligned);
        NAME_AMOUNT(BlockSize);
        NAME(ReverseAdapter);
        NAME_AMOUNT(SliceOffset);
        NAME(Expandable);
        NAME(CanTerminateEarly);
        NAME(MustExplicitlyTerminate);
        NAME(SideEffecting);
        NAME(Family);
        NAME(InternallySynchronized);
        NAME(InfrequentlyUsed);
        NAME(None);
    }
    #undef NAME
    #undef NAME_AMOUNT
}

void AttributeSet::print(llvm::raw_ostream & out) const noexcept {
    if (hasAttributes()) {
        char joiner = '{';
        for (const Attribute & a : getAttributes()) {
            out << joiner;
            a.print(out);
            joiner = ',';
        }
        out << '}';
    }
}

Attribute & AttributeSet::addAttribute(Attribute attribute) {
    for (auto i = begin(), i_end = end(); i != i_end; ++i) {
        Attribute & attr = const_cast<Attribute &>(*i);
        if (attr.getKind() == attribute.getKind()) {
            attr.mAmount = attribute.mAmount;
            return attr;
        }
    }
    emplace_back(attribute);
    return back();
}

Attribute * AttributeSet::__findAttribute(const AttributeId id) const {
    for (auto i = begin(), i_end = end(); i != i_end; ++i) {
        if (i->getKind() == id) {
            return const_cast<Attribute *>(&*i);
        }
    }
    return nullptr;
}

}
