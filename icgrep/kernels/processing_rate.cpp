#include "processing_rate.h"
#include <llvm/Support/Compiler.h>
#include <llvm/Support/raw_ostream.h>
#include <kernels/kernel.h>

namespace kernel {

using RateValue = ProcessingRate::RateValue;

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
        case KindId::Greedy:
            out << 'G';
            break;
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
    const Kernel::StreamSetPort ref = kernel->getStreamPort(mReference);
    switch (ref.first) {
        case Kernel::Port::Input:
            out << 'I';
            break;
        case Kernel::Port::Output:
            out << 'O';
            break;
    }
    out << ref.second;
}

}
