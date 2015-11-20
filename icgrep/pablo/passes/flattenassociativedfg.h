#ifndef FLATTENASSOCIATIVEDFG_H
#define FLATTENASSOCIATIVEDFG_H


namespace pablo {

class PabloFunction;
class PabloBlock;
class Variadic;

class FlattenAssociativeDFG {
public:
    static void process(PabloFunction & function);
protected:
    static void traverse(PabloBlock * const block);
    static bool flatten(PabloBlock * const block);
    static bool factorize(PabloBlock * const block);
    static bool flatten(Variadic * const var, PabloBlock * const block);
    FlattenAssociativeDFG() = default;
};

}

#endif // FLATTENASSOCIATIVEDFG_H
