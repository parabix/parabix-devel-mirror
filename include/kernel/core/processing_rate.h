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

    enum KindId : unsigned {
        // countable rates
        Fixed, PopCount, NegatedPopCount, PartialSum,
        // addressable (non-countable) rates
        Relative, Bounded, Greedy, Unknown,
        // internal
        __Count
    };

    using Rational = boost::rational<size_t>;

    KindId getKind() const { return mKind; }

    Rational getRate() const {
        return mLowerBound;
    }

    Rational getLowerBound() const {
        return mLowerBound;
    }

    Rational getUpperBound() const {
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

    bool isGreedy() const {
        return mKind == KindId::Greedy;
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

    ProcessingRate() : ProcessingRate(KindId::Fixed, Rational{1}, Rational{1}) { }
    ProcessingRate(ProcessingRate &&) = default;
    ProcessingRate(const ProcessingRate &) = default;
    ProcessingRate & operator = (const ProcessingRate & other) = default;

    ProcessingRate(const KindId k, const Rational lb, const Rational ub)
    : mKind(k)
    , mLowerBound(lb)
    , mUpperBound(ub)
    , mReference() {

    }

    ProcessingRate(const KindId k, const Rational lb, const Rational ub, const llvm::StringRef ref)
    : mKind(k)
    , mLowerBound(lb)
    , mUpperBound(ub)
    , mReference(ref) {

    }

    void print(const Kernel * const kernel, llvm::raw_ostream & out) const noexcept;

private:
    KindId          mKind;
    Rational       mLowerBound;
    Rational       mUpperBound;
    llvm::StringRef mReference;
};

inline ProcessingRate FixedRate(const ProcessingRate::Rational rate = ProcessingRate::Rational{1}) {
    assert (rate.numerator() > 0);
    return ProcessingRate(ProcessingRate::KindId::Fixed, rate, rate);
}

inline ProcessingRate BoundedRate(const unsigned lower, const unsigned upper) {
    if (lower == upper) {
        return FixedRate(lower);
    } else {
        assert (upper > lower);
        return ProcessingRate(ProcessingRate::KindId::Bounded, ProcessingRate::Rational(lower), ProcessingRate::Rational(upper));
    }
}

inline ProcessingRate GreedyRate(const ProcessingRate::Rational lower = ProcessingRate::Rational{0}) {
    return ProcessingRate(ProcessingRate::KindId::Greedy, lower, 0);
}

inline ProcessingRate UnknownRate(const ProcessingRate::Rational lower = ProcessingRate::Rational{0}) {
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

ProcessingRate::Rational lcm(const ProcessingRate::Rational & x, const ProcessingRate::Rational & y);

ProcessingRate::Rational gcd(const ProcessingRate::Rational & x, const ProcessingRate::Rational & y);

unsigned floor(const ProcessingRate::Rational & r);

unsigned ceiling(const ProcessingRate::Rational & r);

}

#endif // PROCESSING_RATE_H
