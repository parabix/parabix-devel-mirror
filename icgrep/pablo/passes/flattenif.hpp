#ifndef FLATTENIF_H
#define FLATTENIF_H
#include <pablo/codegenstate.h>

namespace pablo {

class PabloFunction;
class PabloBlock;
class Statement;
class PabloAST;

class FlattenIf {
    friend class DistributivePass;
    friend class FactorizeDFG;
public:
    static void transform(PabloFunction & function);
protected:
    FlattenIf() = default;
private:
    static void flattenIf(PabloBlock * const block);
};

}

#endif // FLATTENIF_H
