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
        return mValue;
    }

    LLVM_READNONE unsigned getNumElements() const;

    LLVM_READNONE unsigned getFieldWidth() const;

protected:

    void print(const Kernel * const kernel, llvm::raw_ostream & out) const noexcept;

    void setRelationship(Relationship * const value);

private:
    const std::string       mName;
    const ProcessingRate    mRate;
    llvm::Type * const      mType;
    Relationship *          mValue;
};

using Bindings = std::vector<Binding>;


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief isCountable
 ** ------------------------------------------------------------------------------------------------------------- */
LLVM_READNONE inline bool isCountable(const Binding & binding) {
    if (LLVM_UNLIKELY(binding.isDeferred())) {
        return false;
    }
    const ProcessingRate & rate = binding.getRate();
    return rate.isFixed() || rate.isPopCount() || rate.isNegatedPopCount();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief isAddressable
 ** ------------------------------------------------------------------------------------------------------------- */
LLVM_READNONE inline bool isAddressable(const Binding & binding) {
    if (LLVM_UNLIKELY(binding.isDeferred())) {
        return true;
    }
    const ProcessingRate & rate = binding.getRate();
    return rate.isBounded() || rate.isUnknown();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief isAnyPopCount
 ** ------------------------------------------------------------------------------------------------------------- */
LLVM_READNONE inline bool isAnyPopCount(const Binding & binding) {
    const ProcessingRate & rate = binding.getRate();
    return rate.isPopCount() || rate.isNegatedPopCount();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief requiresItemCount
 ** ------------------------------------------------------------------------------------------------------------- */
LLVM_READNONE inline bool requiresItemCount(const Binding & binding) {
    return isAddressable(binding) || isAnyPopCount(binding);
}


}

#endif // BINDING_H
