#ifndef BINDING_H
#define BINDING_H

#include "processing_rate.h"
#include "attributes.h"
#include <llvm/ADT/STLExtras.h>

namespace llvm { class Type; }
namespace llvm { class raw_ostream; }

namespace kernel {

class Kernel;
class Relationship;

struct Binding : public AttributeSet {

    friend class Kernel;
    friend class PipelineBuilder;
    friend class PipelineCompiler;
    friend class PipelineKernel;

    // TODO: use templatized var-args to simplify the constructors? would need to default in the processing rate and verify only one was added.

    Binding(std::string name, Relationship * const value, ProcessingRate r = FixedRate(1));
    Binding(std::string name, Relationship * const value, ProcessingRate r, Attribute && attribute);
    Binding(std::string name, Relationship * const value, ProcessingRate r, std::initializer_list<Attribute> attributes);

    Binding(llvm::Type * const scalarType, std::string name, ProcessingRate r = FixedRate(1));
    Binding(llvm::Type * const scalarType, std::string name, ProcessingRate r, Attribute && attribute);
    Binding(llvm::Type * const scalarType, std::string name, ProcessingRate r, std::initializer_list<Attribute> attributes);

    Binding(llvm::Type * const scalarType, std::string name, Relationship * const value, ProcessingRate r = FixedRate(1));
    Binding(llvm::Type * const scalarType, std::string name, Relationship * const value, ProcessingRate r, Attribute && attribute);
    Binding(llvm::Type * const scalarType, std::string name, Relationship * const value, ProcessingRate r, std::initializer_list<Attribute> attributes);

    Binding(const Binding & original, ProcessingRate r);

    const std::string & getName() const LLVM_READNONE {
        return mName;
    }

    const ProcessingRate & getRate() const LLVM_READNONE {
        return mRate;
    }

    bool isPrincipal() const LLVM_READNONE {
        return hasAttribute(AttributeId::Principal);
    }

    bool hasLookahead() const LLVM_READNONE {
        return hasAttribute(AttributeId::LookAhead);
    }

    unsigned const getLookahead() const LLVM_READNONE {
        return findAttribute(AttributeId::LookAhead).amount();
    }

    bool isDeferred() const LLVM_READNONE {
        return hasAttribute(AttributeId::Deferred);
    }

    llvm::Type * getType() const {
        return mType;
    }

    Relationship * getRelationship() const {
        return mRelationship;
    }

    void setRelationship(Relationship * const value);

    LLVM_READNONE unsigned getNumElements() const;

    LLVM_READNONE unsigned getFieldWidth() const;

    void print(const Kernel * const kernel, llvm::raw_ostream & out) const noexcept;

private:
    const std::string       mName;
    const ProcessingRate    mRate;
    llvm::Type * const      mType;
    Relationship *          mRelationship;
};

using Bindings = std::vector<Binding>;

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief isCountable
 ** ------------------------------------------------------------------------------------------------------------- */
LLVM_READNONE inline bool isCountable(const Binding & binding) {
    const ProcessingRate & rate = binding.getRate();
    switch (rate.getKind()) {
        case ProcessingRate::KindId::Fixed:
        case ProcessingRate::KindId::PopCount:
        case ProcessingRate::KindId::NegatedPopCount:
        case ProcessingRate::KindId::PartialSum:
        case ProcessingRate::KindId::Greedy:
            return true;
        default:
            return false;
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief isNonFixedCountable
 ** ------------------------------------------------------------------------------------------------------------- */
LLVM_READNONE inline bool isNonFixedCountable(const Binding & binding) {
    const ProcessingRate & rate = binding.getRate();
    switch (rate.getKind()) {
        case ProcessingRate::KindId::PopCount:
        case ProcessingRate::KindId::NegatedPopCount:
        case ProcessingRate::KindId::PartialSum:
        case ProcessingRate::KindId::Greedy:
            return true;
        default:
            return false;
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief isAddressable
 ** ------------------------------------------------------------------------------------------------------------- */
LLVM_READNONE inline bool isAddressable(const Binding & binding) {
    if (LLVM_UNLIKELY(binding.isDeferred())) {
        return true;
    }
    const ProcessingRate & rate = binding.getRate();
    switch (rate.getKind()) {
        case ProcessingRate::KindId::Bounded:
        case ProcessingRate::KindId::Unknown:
            return true;
        default:
            return false;
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief requiresItemCount
 ** ------------------------------------------------------------------------------------------------------------- */
LLVM_READNONE inline bool requiresItemCount(const Binding & binding) {
    return isAddressable(binding) || isNonFixedCountable(binding);
}


}

#endif // BINDING_H
