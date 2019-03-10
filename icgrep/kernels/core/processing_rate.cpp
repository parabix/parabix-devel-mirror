#include "processing_rate.h"
#include "kernel.h"
#include <llvm/Support/Compiler.h>
#include <llvm/Support/raw_ostream.h>

namespace kernel {

using RateValue = ProcessingRate::RateValue;
using RateId = ProcessingRate::KindId;
using StreamPort = Kernel::StreamSetPort;
using PortType = Kernel::PortType;

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief lcm
 ** ------------------------------------------------------------------------------------------------------------- */
RateValue lcm(const RateValue & x, const RateValue & y) {
    if (LLVM_LIKELY(x.denominator() == 1 && y.denominator() == 1)) {
        return RateValue(boost::lcm(x.numerator(), y.numerator()), 1);
    } else { // LCM((a/b),(c/d)) = LCM ((a*LCM(b,d))/b, (c*LCM(b,d))/d) / LCM(b,d)
        const auto d = boost::lcm(x.denominator(), y.denominator());
        const auto n = boost::lcm((x.numerator() * d) / x.denominator(), (y.numerator() * d) / y.denominator());
        return RateValue(n, d);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief gcd
 ** ------------------------------------------------------------------------------------------------------------- */
RateValue gcd(const RateValue & x, const RateValue & y) {
    const auto n = boost::gcd(x.numerator(), y.numerator());
    if (LLVM_LIKELY(x.denominator() == 1 && y.denominator() == 1)) {
        return RateValue(n, 1);
    } else { // GCD((a/b),(c/d)) = GCD(a,c) / LCM(b,d)
        return RateValue(n, boost::lcm(x.denominator(), y.denominator()));
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief roundUp
 ** ------------------------------------------------------------------------------------------------------------- */
unsigned floor(const RateValue & r) {
    if (LLVM_LIKELY(r.denominator() == 1)) {
        return r.numerator();
    } else {
        return r.numerator() / r.denominator();
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief roundUp
 ** ------------------------------------------------------------------------------------------------------------- */
unsigned ceiling(const RateValue & r) {
    if (LLVM_LIKELY(r.denominator() == 1)) {
        return r.numerator();
    } else {
        return (r.numerator() + r.denominator() - 1) / r.denominator();
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief write
 ** ------------------------------------------------------------------------------------------------------------- */
void write(const RateValue & v, llvm::raw_ostream & out) noexcept {
    if (LLVM_LIKELY(v.denominator() == 1)) {
        out << v.numerator();
    } else {
        out << '(' << v.numerator() << '/' << v.denominator() << ')';
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief print
 ** ------------------------------------------------------------------------------------------------------------- */
void ProcessingRate::print(const Kernel * const kernel, llvm::raw_ostream & out) const noexcept {
    switch (mKind) {
        case KindId::Fixed:
            out << 'F';
            write(mLowerBound, out);
            return;
        case KindId::Bounded:
            out << 'B';
            write(mLowerBound, out);
            out << '-';
            write(mUpperBound, out);
            return;
        case KindId::Unknown:
            out << 'U';
            write(mLowerBound, out);
            return;
        case KindId::PopCount:
            out << 'P';
            break;
        case KindId::NegatedPopCount:
            out << 'N';
            break;
        case KindId::Relative:
            out << 'R';
            break;
    }
    write(mLowerBound, out);
    const StreamPort ref = kernel->getStreamPort(mReference);
    switch (ref.Type) {
        case PortType::Input:
            out << 'I';
            break;
        case PortType::Output:
            out << 'O';
            break;
    }
    out << ref.Number;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief equals
 ** ------------------------------------------------------------------------------------------------------------- */
inline bool equals(const ProcessingRate & A, const RateValue & AF, const ProcessingRate & B, const RateValue & BF) {
    return (A.getLowerBound() == B.getLowerBound()) && (A.getUpperBound() == B.getUpperBound()) && AF == BF;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief atLeast
 ** ------------------------------------------------------------------------------------------------------------- */
inline bool atLeast(const ProcessingRate & A, const RateValue & AF, const ProcessingRate & B, const RateValue & BF) {
    return A.getLowerBound() * AF <= B.getLowerBound() * BF;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief atMost
 ** ------------------------------------------------------------------------------------------------------------- */
inline bool atMost(const ProcessingRate & A, const RateValue & AF, const ProcessingRate & B, const RateValue & BF) {
    return B.getUpperBound() * BF <= A.getUpperBound() * AF;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief within
 ** ------------------------------------------------------------------------------------------------------------- */
inline bool within(const ProcessingRate & A, const RateValue & AF, const ProcessingRate & B, const RateValue & BF) {
    return atLeast(A, AF, B, BF) && atMost(A, AF, B, BF);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief permits
 ** ------------------------------------------------------------------------------------------------------------- */
bool permits(const Kernel * const hostKernel,
             const Binding & hostBinding,
             const RateValue & hostFactor,
             const Kernel * const visitorKernel,
             const Binding & visitorBinding,
             const RateValue & visitorFactor) {

    const ProcessingRate & hostRate = hostBinding.getRate();
    const RateId hostRateId = hostRate.getKind();
    const ProcessingRate & visitorRate = visitorBinding.getRate();
    const RateId visitorRateId = visitorRate.getKind();

    if (LLVM_LIKELY(hostRateId == visitorRateId)) {
        switch (hostRateId) {
            case RateId::Fixed:
                return equals(hostRate, hostFactor, visitorRate, visitorFactor);
            case RateId::PopCount:
            case RateId::NegatedPopCount:
            case RateId::Relative: { // is the reference rate processed at the same rate?
                const Binding & branchRef = hostKernel->getStreamBinding(hostRate.getReference());
                const Binding & sourceRef = visitorKernel->getStreamBinding(visitorRate.getReference());
                return permits(hostKernel, branchRef, hostFactor * hostRate.getRate(),
                               visitorKernel, sourceRef, visitorFactor * visitorRate.getRate());
            }
            case RateId::Bounded:
                return within(hostRate, hostFactor, visitorRate, visitorFactor);
            case RateId::Unknown:
                return atLeast(hostRate, hostFactor, visitorRate, visitorFactor);
            default: return false;
        }
    } else {
        switch (hostRateId) {
            case RateId::Fixed:
                return false;
            case RateId::Bounded:
                switch (visitorRateId) {
                    case RateId::Fixed:
                    case RateId::PopCount:
                    case RateId::NegatedPopCount:
                        return within(hostRate, hostFactor, visitorRate, visitorFactor);
                    case RateId::Relative: { // is the reference rate processed at the same rate?
                        const Binding & sourceRef = visitorKernel->getStreamBinding(visitorRate.getReference());
                        return permits(hostKernel, hostBinding, hostFactor,
                                       visitorKernel, sourceRef, visitorFactor * visitorRate.getRate());
                    }
                    default: return false;
                }
            default: return false;
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief permits
 ** ------------------------------------------------------------------------------------------------------------- */
bool permits(const Kernel * const hostKernel, const Binding & host,
             const Kernel * const visitorKernel, const Binding & visitor) {
    const RateValue ONE{1};
    return permits(hostKernel, host, ONE, visitorKernel, visitor, ONE);
}


}
