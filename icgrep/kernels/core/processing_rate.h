#ifndef PROCESSING_RATE_H
#define PROCESSING_RATE_H

#include <assert.h>
#include <boost/rational.hpp>
#include <llvm/ADT/StringRef.h>

namespace llvm { class raw_ostream; }

namespace kernel {

// Processing rate attributes are required for all stream set bindings. They describe
// the relationship between processed items (inputs) and produced items (outputs).
//
// For example, the 3-to-4 kernel converts every 3 input items into 4 output items.
// Thus it has a FixedRate(3) for its input stream and FixedRate(4) for its output
// stream. Processing these every 3 items individually would be time consuming. Instead
// the kernel processes a strides' worth of "iterations" and automatically scales the
// FixedRates accordingly.
//
// NOTE: fixed and bounded rates should be the smallest number of input items for the
// smallest number of output items that can be logically produced by a kernel.

class Kernel;
struct Binding;

struct ProcessingRate  {

    friend struct Binding;

    enum class KindId {
        Fixed, Bounded, PopCount, NegatedPopCount, PartialSum, Relative, Unknown
    };

    using RateValue = boost::rational<unsigned>;

    KindId getKind() const { return mKind; }

    RateValue getRate() const {
        return mLowerBound;
    }

    RateValue getLowerBound() const {
        return mLowerBound;
    }

    RateValue getUpperBound() const {
        return mUpperBound;
    }

    const llvm::StringRef getReference() const {
        assert (hasReference());
        return mReference;
    }

    bool isFixed() const {
        return mKind == KindId::Fixed;
    }

    bool isBounded() const {
        return mKind == KindId::Bounded;
    }

    bool isRelative() const {
        return mKind == KindId::Relative;
    }

    bool isPopCount() const {
        return mKind == KindId::PopCount;
    }

    bool isNegatedPopCount() const {
        return mKind == KindId::NegatedPopCount;
    }

    bool isPartialSum() const {
        return mKind == KindId::PartialSum;
    }

    bool isUnknown() const {
        return mKind == KindId::Unknown;
    }

    bool hasReference() const {
        switch (mKind) {
            case KindId::PopCount:
            case KindId::NegatedPopCount:
            case KindId::PartialSum:
            case KindId::Relative:
                assert (mReference.data());
                return true;
            default:
                return false;
        }
    }

    bool isDerived() const {
        return isRelative();
    }

    bool operator == (const ProcessingRate & other) const {
        return mKind == other.mKind && mLowerBound == other.mLowerBound && mUpperBound == other.mUpperBound && mReference == other.mReference;
    }

    bool operator != (const ProcessingRate & other) const {
        return !(*this == other);
    }

    bool operator < (const ProcessingRate & other) const {
        if (mKind < other.mKind) {
            return true;
        } else if (mLowerBound < other.mLowerBound) {
            return true;
        } else if (mUpperBound < other.mUpperBound) {
            return true;
        } else {
            return mReference < other.mReference;
        }
    }

    ProcessingRate() : ProcessingRate(KindId::Fixed, RateValue{1}, RateValue{1}) { }
    ProcessingRate(ProcessingRate &&) = default;
    ProcessingRate(const ProcessingRate &) = default;
    ProcessingRate & operator = (const ProcessingRate & other) = default;

    ProcessingRate(const KindId k, const RateValue lb, const RateValue ub)
    : mKind(k)
    , mLowerBound(lb)
    , mUpperBound(ub)
    , mReference() {

    }

    ProcessingRate(const KindId k, const RateValue lb, const RateValue ub, const llvm::StringRef ref)
    : mKind(k)
    , mLowerBound(lb)
    , mUpperBound(ub)
    , mReference(ref) {

    }

    void print(const Kernel * const kernel, llvm::raw_ostream & out) const noexcept;

private:
    KindId          mKind;
    RateValue       mLowerBound;
    RateValue       mUpperBound;
    llvm::StringRef mReference;
};

inline ProcessingRate FixedRate(const ProcessingRate::RateValue rate) {
    assert (rate.numerator() > 0);
    return ProcessingRate(ProcessingRate::KindId::Fixed, rate, rate);
}

inline ProcessingRate FixedRate() {
    return FixedRate(ProcessingRate::RateValue{1});
}

inline ProcessingRate BoundedRate(const unsigned lower, const unsigned upper) {
    if (lower == upper) {
        return FixedRate(lower);
    } else {
        assert (upper > lower);
        return ProcessingRate(ProcessingRate::KindId::Bounded, ProcessingRate::RateValue(lower), ProcessingRate::RateValue(upper));
    }
}

inline ProcessingRate UnknownRate(const unsigned lower = 0) {
    return ProcessingRate(ProcessingRate::KindId::Unknown, lower, 0);
}

inline ProcessingRate RateEqualTo(llvm::StringRef ref) {
    return ProcessingRate(ProcessingRate::KindId::Relative, 1, 1, ref);
}

inline ProcessingRate PopcountOf(llvm::StringRef ref) {
    return ProcessingRate(ProcessingRate::KindId::PopCount, 0, 1, ref);
}

inline ProcessingRate PopcountOfNot(llvm::StringRef ref) {
    return ProcessingRate(ProcessingRate::KindId::NegatedPopCount, 0, 1, ref);
}

inline ProcessingRate PartialSum(llvm::StringRef ref) {
    return ProcessingRate(ProcessingRate::KindId::PartialSum, 0, 1, ref);
}

ProcessingRate::RateValue lcm(const ProcessingRate::RateValue & x, const ProcessingRate::RateValue & y);

ProcessingRate::RateValue gcd(const ProcessingRate::RateValue & x, const ProcessingRate::RateValue & y);

unsigned floor(const ProcessingRate::RateValue & r);

unsigned ceiling(const ProcessingRate::RateValue & r);

bool permits(const Kernel * const hostKernel, const Binding & host,
             const Kernel * const visitorKernel, const Binding & visitor);

}

#endif // PROCESSING_RATE_H
