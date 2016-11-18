#ifndef FLATTENIF_H
#define FLATTENIF_H

namespace pablo {

class PabloKernel;
class PabloBlock;

class FlattenIf {
public:
    static void transform(PabloKernel * function);
protected:
    FlattenIf() = default;
private:
    static void flattenIf(PabloBlock * const block);
};

}

#endif // FLATTENIF_H
