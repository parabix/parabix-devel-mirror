#ifndef FLATTENIF_H
#define FLATTENIF_H

namespace pablo {

class PabloFunction;
class PabloBlock;

class FlattenIf {
public:
    static void transform(PabloFunction & function);
protected:
    FlattenIf() = default;
private:
    static void flattenIf(PabloBlock * const block);
};

}

#endif // FLATTENIF_H
