#include "processing_rate.h"
#include <llvm/Support/Compiler.h>

namespace kernel {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief lcm
 ** ------------------------------------------------------------------------------------------------------------- */
ProcessingRate::RateValue lcm(const ProcessingRate::RateValue & x, const ProcessingRate::RateValue & y) {
    if (LLVM_LIKELY(x.denominator() == 1 && y.denominator() == 1)) {
        return ProcessingRate::RateValue(boost::lcm(x.numerator(), y.numerator()), 1);
    } else { // LCM((a/b),(c/d)) = LCM ((a*LCM(b,d))/b, (c*LCM(b,d))/d) / LCM(b,d)
        const auto d = boost::lcm(x.denominator(), y.denominator());
        const auto n = boost::lcm((x.numerator() * d) / x.denominator(), (y.numerator() * d) / y.denominator());
        return ProcessingRate::RateValue(n, d);
    }
}

ProcessingRate::RateValue gcd(const ProcessingRate::RateValue & x, const ProcessingRate::RateValue & y) {
    const auto n = boost::gcd(x.numerator(), y.numerator());
    if (LLVM_LIKELY(x.denominator() == 1 && y.denominator() == 1)) {
        return ProcessingRate::RateValue(n, 1);
    } else { // GCD((a/b),(c/d)) = GCD(a,c) / LCM(b,d)
        return ProcessingRate::RateValue(n, boost::lcm(x.denominator(), y.denominator()));
    }
}

}
