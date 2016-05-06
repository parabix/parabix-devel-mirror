#include <pablo/passes/flattenif.hpp>
#include <pablo/codegenstate.h>
#include <pablo/expression_map.hpp>
#include <pablo/function.h>
#include <pablo/printer_pablos.h>
#include <pablo/analysis/pabloverifier.hpp>
#include <boost/container/flat_set.hpp>

#include <pablo/printer_pablos.h>
#include <iostream>

namespace pablo {
    
bool has_if(PabloBlock * const block){
    Statement * stmt = block->front();
    int if_flag = false;
    while(stmt){
        if (isa<If>(stmt)){
            if_flag = true;
            return if_flag;
        }
        stmt = stmt->getNextNode();
    }
    return if_flag;
}

void flattenIf_helper(PabloBlock * const block){
    Statement * stmt = block->front();
    /*std::string tmp;
     llvm::raw_string_ostream msg(tmp);
     msg << "stmt: ";
     PabloPrinter::print(stmt, msg);*/
    while(stmt){
        if (If * ifNode = dyn_cast<If>(stmt)){
            Statement * prior = stmt->getPrevNode();
            PabloBlock * body = ifNode->getBody();
            //Simplifier::flattenIf(body);
            Statement * body_stmt = body->front();
            Statement * next;
            while(body_stmt){
                next = body_stmt->getNextNode();
                if (isa<Assign>(body_stmt)) {
                    ifNode->removeDefined(cast<Assign>(body_stmt));
                }
                body_stmt->insertAfter(prior);
                prior = body_stmt;
                body_stmt = next;
            }
            stmt = stmt->eraseFromParent();
            
        }
        stmt = stmt->getNextNode();
    }
}

void FlattenIf::flattenIf(PabloBlock * const block){
    bool flag = has_if(block);
    while(flag){
        flattenIf_helper(block);
        flag = has_if(block);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief transform
 ** ------------------------------------------------------------------------------------------------------------- */
void FlattenIf::transform(PabloFunction & function) {
    flattenIf(function.getEntryBlock());
    #ifndef NDEBUG
    PabloVerifier::verify(function, "flatten-if");
    #endif
}

}
