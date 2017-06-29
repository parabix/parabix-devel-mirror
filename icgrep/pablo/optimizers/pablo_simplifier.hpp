#ifndef PABLO_SIMPLIFIER_HPP
#define PABLO_SIMPLIFIER_HPP

namespace pablo {

class PabloKernel;

class Simplifier {
public:
    static bool optimize(PabloKernel * kernel);
protected:
    Simplifier() = default;
};

}
#endif // PABLO_SIMPLIFIER_HPP
