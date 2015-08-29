#ifndef BOOLEANREASSOCIATIONPASS_H
#define BOOLEANREASSOCIATIONPASS_H

#include <pablo/codegenstate.h>

namespace pablo {

class BooleanReassociationPass {   
public:
    static bool optimize(PabloFunction & function);
protected:
    BooleanReassociationPass();
    void scan(PabloFunction & function);
    void scan(PabloBlock & block, std::vector<Statement *> && terminals);
    void processScope(PabloBlock & block, std::vector<Statement *> && terminals);
};

}

#endif // BOOLEANREASSOCIATIONPASS_H
