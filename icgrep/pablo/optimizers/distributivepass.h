#ifndef DISTRIBUTIVEPASS_H
#define DISTRIBUTIVEPASS_H

namespace pablo {

class PabloFunction;
class PabloBlock;
class Variadic;

class DistributivePass {
public:
    static bool optimize(PabloFunction & function);
protected:
    static bool process(PabloBlock * const block);
    static bool distribute(PabloBlock * const block);
    DistributivePass() = default;
};

}

#endif // DISTRIBUTIVEPASS_H
