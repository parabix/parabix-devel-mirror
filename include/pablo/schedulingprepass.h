#ifndef SCHEDULINGPREPASS_H
#define SCHEDULINGPREPASS_H

namespace pablo {

class PabloKernel;

class SchedulingPrePass {
public:
    static bool optimize(PabloKernel * kernel);
};

}

#endif // SCHEDULINGPREPASS_H
