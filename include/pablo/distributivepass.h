#ifndef PABLO_DISTRIBUTIVEPASS_H
#define PABLO_DISTRIBUTIVEPASS_H

namespace pablo {

class PabloKernel;

class DistributivePass {
public:
    static bool optimize(pablo::PabloKernel * const kernel);
};

}

#endif // DISTRIBUTIVEPASS_H
