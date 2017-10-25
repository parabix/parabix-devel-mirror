#ifndef PROCESSING_RATE_H
#define PROCESSING_RATE_H

#include <string>
#include <assert.h>

namespace kernel {

// Processing rate attributes are required for all stream set bindings for a kernel.
// These attributes describe the number of items that are processed or produced as
// a ratio in comparison to a reference stream set, normally the principal input stream set
// by default (or the principal output stream set if there is no input).
//
// The default ratio is FixedRatio(1) which means that there is one item processed or
// produced for every item of the reference stream.
// FixedRatio(m, n) means that for every group of n items of the refrence stream,
// there are m items in the output stream (rounding up).
//
// Kernels which produce a variable number of items use MaxRatio(n), for a maximum
// of n items produced or consumed per principal input or output item.  MaxRatio(m, n)
// means there are at most m items for every n items of the reference stream.
//
// RoundUpToMultiple(n) means that number of items produced is the same as the
// number of reference items, rounded up to an exact multiple of n.
//

struct ProcessingRate  {

    enum class KindId {
        Fixed, Bounded, Unknown, DirectlyRelative, PopCountRelative
    };

    KindId getKind() const { return mKind; }

    unsigned getRate() const {
        assert (isFixed());
        assert (mN > 0 && mN == mM);
        return mN;
    }

    unsigned getLowerBound() const {
        assert (isFixed() || isBounded() || isUnknown());
        return mN;
    }

    unsigned getUpperBound() const {
        assert (isFixed() || isBounded());
        assert (isFixed() ? mM == mN : mM > mN);
        return mM;
    }

    const std::string & getReference() const {
        assert (isExactlyRelative());
        return mReference;
    }

    const unsigned getNumerator() const {
        assert (isExactlyRelative());
        assert (mM > 0);
        return mM;
    }

    const unsigned getDenominator() const {
        assert (isExactlyRelative());
        assert (mN > 0);
        return mN;
    }

    bool isFixed() const {
        return mKind == KindId::Fixed;
    }

    bool isBounded() const {
        return mKind == KindId::Bounded;
    }

    bool isExactlyRelative() const {
        return mKind == KindId::DirectlyRelative;
    }

    bool isUnknown() const {
        return mKind == KindId::Unknown;
    }

    bool isDerived() const {
        return isExactlyRelative(); // isFixed() ||
    }

    bool operator == (const ProcessingRate & other) const {
        return mKind == other.mKind && mN == other.mN && mM == other.mM && mReference == other.mReference;
    }

    bool operator != (const ProcessingRate & other) const {
        return !(*this == other);
    }

    ProcessingRate & operator = (const ProcessingRate & other) {
        mKind = other.mKind;
        mN = other.mN;
        mM = other.mM;
        mReference = other.mReference;
        return *this;
    }

    friend ProcessingRate FixedRate(const unsigned);
    friend ProcessingRate BoundedRate(const unsigned, const unsigned);
    friend ProcessingRate UnknownRate(const unsigned);
    friend ProcessingRate RateEqualTo(std::string);

protected:

    ProcessingRate(const KindId k, const unsigned n, const unsigned m, const std::string && ref = "") : mKind(k), mN(n), mM(m), mReference(ref) {}
private:
    KindId mKind;
    unsigned mN;
    unsigned mM;
    std::string mReference;
};

inline ProcessingRate FixedRate(const unsigned rate = 1) {
    return ProcessingRate(ProcessingRate::KindId::Fixed, rate, rate);
}

inline ProcessingRate BoundedRate(const unsigned lower, const unsigned upper) {
    if (lower == upper) {
        return FixedRate(lower);
    } else {
        return ProcessingRate(ProcessingRate::KindId::Bounded, lower, upper);
    }
}

inline ProcessingRate UnknownRate(const unsigned lower = 0) {
    return ProcessingRate(ProcessingRate::KindId::Unknown, lower, 0);
}

inline ProcessingRate RateEqualTo(std::string ref) {
    return ProcessingRate(ProcessingRate::KindId::DirectlyRelative, 1, 1, std::move(ref));
}

}

#endif // PROCESSING_RATE_H
