#ifndef BOOLEANREASSOCIATIONPASS_H
#define BOOLEANREASSOCIATIONPASS_H

#include <pablo/codegenstate.h>

namespace pablo {

class BooleanReassociationPass {   

//    using Vertex = Graph::vertex_descriptor;
//    using Map = std::unordered_map<PabloAST *, Vertex>;
//    using Queue = boost::circular_buffer<PabloAST *>;
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
