#ifndef SSAPASS_H
#define SSAPASS_H

namespace pablo {

class PabloKernel;

class SSAPass {
public:
    static bool transform(PabloKernel * kernel);
protected:
    SSAPass() = default;
};

}

#endif // SSAPASS_H
