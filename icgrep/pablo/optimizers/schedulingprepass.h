#ifndef SCHEDULINGPREPASS_H
#define SCHEDULINGPREPASS_H

namespace pablo {

class PabloFunction;

class SchedulingPrePass {
public:
    static bool optimize(PabloFunction & function);
protected:
    SchedulingPrePass() = default;
};

}

#endif // SCHEDULINGPREPASS_H
