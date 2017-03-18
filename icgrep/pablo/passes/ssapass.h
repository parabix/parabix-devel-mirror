#ifndef SSAPASS_H
#define SSAPASS_H

namespace pablo {

class PabloKernel;
class PabloBlock;


class SSAPass {
public:
    static bool transform(PabloKernel * kernel);
protected:
    SSAPass() = default;
private:
    static void toSSAForm(PabloBlock * const block);
};

#endif // SSAPASS_H
