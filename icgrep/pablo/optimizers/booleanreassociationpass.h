#ifndef BOOLEANREASSOCIATIONPASS_H
#define BOOLEANREASSOCIATIONPASS_H

#include <pablo/codegenstate.h>
#include <boost/container/flat_map.hpp>
#include <boost/graph/adjacency_list.hpp>

namespace pablo {

class BooleanReassociationPass {
public:
    using Graph = boost::adjacency_list<boost::hash_setS, boost::vecS, boost::bidirectionalS, PabloAST *>;

    static bool optimize(PabloFunction & function);
protected:
    BooleanReassociationPass();
    void resolveScopes(PabloFunction & function);
    void resolveScopes(PabloBlock &block);
    void processScopes(PabloFunction & function);
    void processScopes(PabloBlock & block, std::vector<Statement *> && terminals);
    void processScope(PabloBlock & block, std::vector<Statement *> && terminals);
    void summarizeAST(PabloBlock & block, std::vector<Statement *> && terminals, Graph & G);
private:
    boost::container::flat_map<PabloBlock *, Statement *> mResolvedScopes;
};

}

#endif // BOOLEANREASSOCIATIONPASS_H
