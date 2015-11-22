#ifndef DISTRIBUTIVEPASS_H
#define DISTRIBUTIVEPASS_H

namespace pablo {

class PabloFunction;

class DistributivePass {
public:
    static bool optimize(PabloFunction & function);
protected:

    DistributivePass() = default;
};

}

#endif // DISTRIBUTIVEPASS_H
