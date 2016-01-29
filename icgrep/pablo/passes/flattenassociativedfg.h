#ifndef FLATTENASSOCIATIVEDFG_H
#define FLATTENASSOCIATIVEDFG_H

namespace pablo {

class PabloFunction;
class PabloBlock;
class Statement;
class Variadic;
class Not;
class Assign;
class PabloAST;

class CanonicalizeDFG {
    friend class DistributivePass;
    friend class FactorizeDFG;
public:
    static void transform(PabloFunction & function);
protected:
    static void canonicalize(PabloBlock * const block);
    static Variadic * canonicalize(Variadic * var);
    static void deMorgansExpansion(PabloBlock * const block);
    static void deMorgansExpansion(Not * const negation, PabloBlock * const block);
    static void deMorgansReduction(PabloBlock * const block);
    static void deMorgansReduction(Variadic * const var, PabloBlock * const block);
    static void tryToPartiallyExtractVariadic(PabloBlock * const block);
    static void tryToPartiallyExtractVariadic(Variadic * const var);
    static void removeFalseScopeDependencies(PabloFunction & function);
    CanonicalizeDFG() = default;
};

}

#endif // FLATTENASSOCIATIVEDFG_H
