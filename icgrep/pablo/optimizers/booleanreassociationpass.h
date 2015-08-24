#ifndef BOOLEANREASSOCIATIONPASS_H
#define BOOLEANREASSOCIATIONPASS_H

#include <pablo/codegenstate.h>

namespace pablo {

class BooleanReassociationPass {   
    using Terminals = std::vector<Statement *>;
public:
    static bool optimize(PabloFunction & function);
protected:
    BooleanReassociationPass();
    void scan(PabloFunction & function);
    void scan(PabloBlock & block, Terminals && terminals);
};

}

#endif // BOOLEANREASSOCIATIONPASS_H
