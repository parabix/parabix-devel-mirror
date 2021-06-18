#include <kernel/core/processing_rate.h>

#include <kernel/core/kernel.h>
#include <llvm/Support/Compiler.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/Support/ErrorHandling.h>
#include <array>

namespace kernel {

using Rational = ProcessingRate::Rational;
using RateId = ProcessingRate::KindId;
using StreamSetPort = Kernel::StreamSetPort;
using PortType = Kernel::PortType;

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief lcm
 ** ------------------------------------------------------------------------------------------------------------- */
Rational lcm(const Rational & x, const Rational & y) {
    if (LLVM_LIKELY(x.denominator() == 1 && y.denominator() == 1)) {
        return Rational(boost::lcm(x.numerator(), y.numerator()), 1);
    } else { // LCM((a/b),(c/d)) = LCM ((a*LCM(b,d))/b, (c*LCM(b,d))/d) / LCM(b,d)
        const auto d = boost::lcm(x.denominator(), y.denominator());
        const auto n = boost::lcm((x.numerator() * d) / x.denominator(), (y.numerator() * d) / y.denominator());
        return Rational(n, d);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief gcd
 ** ------------------------------------------------------------------------------------------------------------- */
Rational gcd(const Rational & x, const Rational & y) {
    const auto n = boost::gcd(x.numerator(), y.numerator());
    if (LLVM_LIKELY(x.denominator() == 1 && y.denominator() == 1)) {
        return Rational(n, 1);
    } else { // GCD((a/b),(c/d)) = GCD(a,c) / LCM(b,d)
        return Rational(n, boost::lcm(x.denominator(), y.denominator()));
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief roundUp
 ** ------------------------------------------------------------------------------------------------------------- */
unsigned floor(const Rational & r) {
    if (LLVM_LIKELY(r.denominator() == 1)) {
        return r.numerator();
    } else {
        return r.numerator() / r.denominator();
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief roundUp
 ** ------------------------------------------------------------------------------------------------------------- */
unsigned ceiling(const Rational & r) {
    if (LLVM_LIKELY(r.denominator() == 1)) {
        return r.numerator();
    } else {
        return (r.numerator() + r.denominator() - 1) / r.denominator();
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief write
 ** ------------------------------------------------------------------------------------------------------------- */
void write(const Rational & v, llvm::raw_ostream & out) noexcept {
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

    std::array<char, KindId::__Count> C;
    C[KindId::Fixed] = 'F';
    C[KindId::PopCount] = 'P';
    C[KindId::NegatedPopCount] = 'N';
    C[KindId::PartialSum] = 'S';
    C[KindId::Relative] = 'R';
    C[KindId::Bounded] = 'B';
    C[KindId::Greedy] = 'G';
    C[KindId::Unknown] = 'U';

    switch (mKind) {
        case KindId::Fixed:
        case KindId::Greedy:
        case KindId::Unknown:
            out << C[mKind];
            write(mLowerBound, out);
            return;
        case KindId::Bounded:
            out << C[KindId::Bounded];
            write(mLowerBound, out);
            out << '-';
            write(mUpperBound, out);
            return;
        case KindId::PopCount:
        case KindId::NegatedPopCount:
        case KindId::PartialSum:
        case KindId::Relative:
            break;
        case KindId::__Count:
            llvm_unreachable("ProcessingRate __Count should not be used.");
    }
    out << C[mKind];
    write(mLowerBound, out);
    const Bindings & inputs = kernel->getInputStreamSetBindings();
    const auto n = inputs.size();
    for (unsigned i = 0; i != n; ++i) {
        if (LLVM_LIKELY(inputs[i].getName() == mReference)) {
            out << 'I' << i;
            return;
        }
    }
    const Bindings & outputs = kernel->getOutputStreamSetBindings();
    const auto m = outputs.size();
    for (unsigned i = 0; i != m; ++i) {
        if (LLVM_LIKELY(outputs[i].getName() == mReference)) {
            out << 'O' << i;
            return;
        }
    }
}

}
