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
    void distribute(PabloBlock * const block);
    void distribute(Variadic * const var);
    DistributivePass() = default;
private:
    bool unmodified;
};

}

#endif // DISTRIBUTIVEPASS_H
