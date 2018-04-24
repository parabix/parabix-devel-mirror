#ifndef PROCESSING_RATE_H
#define PROCESSING_RATE_H

#include <string>
#include <assert.h>
#include <boost/rational.hpp>

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




struct ProcessingRate  {

    friend struct Binding;

    enum class KindId {
        Fixed, Bounded, Unknown, Relative, PopCount, NegatedPopCount
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

    const std::string & getReference() const {
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

    bool isUnknown() const {
        return mKind == KindId::Unknown;
    }

    bool hasReference() const {
        return isRelative() || isPopCount() || isNegatedPopCount();
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

    friend ProcessingRate FixedRate(const unsigned);
    friend ProcessingRate BoundedRate(const unsigned, const unsigned);
    friend ProcessingRate UnknownRate(const unsigned);
    friend ProcessingRate RateEqualTo(std::string);
    friend ProcessingRate PopcountOf(std::string);
    friend ProcessingRate PopcountOfNot(std::string);

    ProcessingRate(ProcessingRate &&) = default;
    ProcessingRate(const ProcessingRate &) = default;
    ProcessingRate & operator = (const ProcessingRate & other) = default;

protected:    
    ProcessingRate(const KindId k, const RateValue lb, const RateValue ub, const std::string && ref = "")
    : mKind(k)
    , mLowerBound(lb)
    , mUpperBound(ub)
    , mReference(ref) {
        assert (isFixed() ? mUpperBound == mLowerBound : (isBounded() ? mUpperBound > mLowerBound :  mUpperBound >= mLowerBound));
    }
private:
    const KindId mKind;
    const RateValue mLowerBound;
    const RateValue mUpperBound;
    const std::string mReference;
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
    return ProcessingRate(ProcessingRate::KindId::Relative, 1, 1, std::move(ref));
}

inline ProcessingRate PopcountOf(std::string ref) {
    return ProcessingRate(ProcessingRate::KindId::PopCount, 0, 1, std::move(ref));
}

inline ProcessingRate PopcountOfNot(std::string ref) {
    return ProcessingRate(ProcessingRate::KindId::NegatedPopCount, 0, 1, std::move(ref));
}

ProcessingRate::RateValue lcm(const ProcessingRate::RateValue & x, const ProcessingRate::RateValue & y);

ProcessingRate::RateValue gcd(const ProcessingRate::RateValue & x, const ProcessingRate::RateValue & y);

unsigned ceiling(const ProcessingRate::RateValue & r);

}

#endif // PROCESSING_RATE_H
