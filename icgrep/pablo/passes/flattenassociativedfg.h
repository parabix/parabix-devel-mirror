#ifndef FLATTENASSOCIATIVEDFG_H
#define FLATTENASSOCIATIVEDFG_H

namespace pablo {

class PabloFunction;
class PabloBlock;
class Variadic;
class Not;
class Assign;


class FlattenAssociativeDFG {
public:
    static void transform(PabloFunction & function);
protected:

    static void flatten(PabloBlock * const block);
    static void flatten(Variadic * const var);
    static void applyNegationInwards(Not * const var, PabloBlock * const block);

    static void extract(PabloBlock * const block);
    static void extractNegationsOutwards(Variadic * const var, PabloBlock * const block);
    static void removeCommonCalculation(Assign * const def);

    FlattenAssociativeDFG() = default;
};

}

#endif // FLATTENASSOCIATIVEDFG_H
