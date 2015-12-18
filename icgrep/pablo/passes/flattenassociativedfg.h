#ifndef FLATTENASSOCIATIVEDFG_H
#define FLATTENASSOCIATIVEDFG_H

namespace pablo {

class PabloFunction;
class PabloBlock;
class Statement;
class Variadic;
class Not;
class Assign;

class FlattenAssociativeDFG {
    friend class DistributivePass;
    friend class FactorizeDFG;
public:
    static void transform(PabloFunction & function);
protected:
    static void coalesce(PabloBlock * const block, const bool traverse);
    static void coalesce(Variadic * const var);
    static void deMorgansExpansion(Not * const var, PabloBlock * const block);
    static void deMorgansReduction(PabloBlock * const block);
    static void deMorgansReduction(Variadic * const var, PabloBlock * const block);
    static void removeFalseScopeDependencies(PabloBlock * const block);
    static void removeFalseScopeDependencies(Assign * const def);
    static void tryToPartiallyExtractVariadic(Variadic * const var);
    FlattenAssociativeDFG() = default;
};

}

#endif // FLATTENASSOCIATIVEDFG_H
