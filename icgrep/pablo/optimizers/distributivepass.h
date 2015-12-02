#ifndef DISTRIBUTIVEPASS_H
#define DISTRIBUTIVEPASS_H

namespace pablo {

class PabloFunction;
class PabloBlock;
class Variadic;

class DistributivePass {
public:
    static void optimize(PabloFunction & function);
protected:
    static void distribute(PabloBlock * const block);
    static void process(PabloBlock * const block);
    DistributivePass() = default;
};

}

#endif // DISTRIBUTIVEPASS_H
