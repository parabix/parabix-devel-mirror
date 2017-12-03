#ifndef PROCESSING_RATE_H
#define PROCESSING_RATE_H

#include <string>
#include <assert.h>
#include <boost/rational.hpp>

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

    friend struct Binding;

    enum class KindId {
        Fixed, Bounded, Unknown, Relative, PopCount
    };

    using RateValue = boost::rational<unsigned>;

    KindId getKind() const { return mKind; }

    RateValue getRate() const {
        assert (isFixed() || isRelative());
        return mLowerBound;
    }

    RateValue getLowerBound() const {
        assert (isFixed() || isBounded() || isUnknown());
        return mLowerBound;
    }

    RateValue getUpperBound() const {
        assert (isFixed() || isBounded());
        assert (isFixed() ? mUpperBound == mLowerBound : mUpperBound > mLowerBound);
        return mUpperBound;
    }

    const std::string & getReference() const {
        assert (isRelative());
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

    bool isUnknown() const {
        return mKind == KindId::Unknown;
    }

    bool isDerived() const {
        return isRelative(); // isFixed() ||
    }

    bool operator == (const ProcessingRate & other) const {
        return mKind == other.mKind && mLowerBound == other.mLowerBound && mUpperBound == other.mUpperBound && mReference == other.mReference;
    }

    bool operator != (const ProcessingRate & other) const {
        return !(*this == other);
    }

    friend ProcessingRate FixedRate(const unsigned);
    friend ProcessingRate BoundedRate(const unsigned, const unsigned);
    friend ProcessingRate UnknownRate(const unsigned);
    friend ProcessingRate RateEqualTo(std::string);
    friend ProcessingRate PopcountOf(std::string, const ProcessingRate::RateValue);

    ProcessingRate(ProcessingRate &&) = default;
    ProcessingRate(const ProcessingRate &) = default;
    ProcessingRate & operator = (const ProcessingRate & other) = default;

protected:    
    ProcessingRate(const KindId k, const unsigned n, const unsigned m, const std::string && ref = "") : mKind(k), mLowerBound(n), mUpperBound(m), mReference(ref) {}
    ProcessingRate(const KindId k, const RateValue n, const RateValue m, const std::string && ref = "") : mKind(k), mLowerBound(n), mUpperBound(m), mReference(ref) {}
private:
    KindId mKind;
    RateValue mLowerBound;
    RateValue mUpperBound;
    std::string mReference;
};

inline ProcessingRate FixedRate(const unsigned rate = 1) {
    return ProcessingRate(ProcessingRate::KindId::Fixed, rate, rate);
}

inline ProcessingRate BoundedRate(const unsigned lower, const unsigned upper) {
    using RateValue = boost::rational<unsigned>;
    if (lower == upper) {
        return FixedRate(lower);
    } else {
        return ProcessingRate(ProcessingRate::KindId::Bounded, RateValue(lower), RateValue(upper));
    }
}

/**
 * @brief UnknownRate
 *
 * The produced item count per stride should never be dependent on an unknown rate input stream.
 */
inline ProcessingRate UnknownRate(const unsigned lower = 0) {
    return ProcessingRate(ProcessingRate::KindId::Unknown, lower, 0);
}

inline ProcessingRate RateEqualTo(std::string ref) {
    return ProcessingRate(ProcessingRate::KindId::Relative, 1, 0, std::move(ref));
}

inline ProcessingRate PopcountOf(std::string ref, const ProcessingRate::RateValue ratio = ProcessingRate::RateValue{1}) {
    return ProcessingRate(ProcessingRate::KindId::PopCount, ratio, ProcessingRate::RateValue{0}, std::move(ref));
}

ProcessingRate::RateValue lcm(const ProcessingRate::RateValue & x, const ProcessingRate::RateValue & y);

ProcessingRate::RateValue gcd(const ProcessingRate::RateValue & x, const ProcessingRate::RateValue & y);

}

#endif // PROCESSING_RATE_H
