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
    static void deMorgansReduction(PabloBlock * const block, const bool traverse);
    static void deMorgansReduction(Variadic * const var, PabloBlock * const block);
    static void tryToPartiallyExtractVariadic(PabloBlock * const block);
    static void tryToPartiallyExtractVariadic(Variadic * const var);
    static void removeFalseScopeDependencies(PabloFunction & function);
    FlattenAssociativeDFG() = default;
};

}

#endif // FLATTENASSOCIATIVEDFG_H
