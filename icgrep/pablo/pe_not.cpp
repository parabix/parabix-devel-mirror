/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <pablo/pe_not.h>
#include <pablo/codegenstate.h>


namespace pablo {

PabloAST * OptimizeNot::operator ()(PabloAST * expr) {
    if (isa<Ones>(expr)) {
	return cg.createZeroes();
    }
    else if (isa<Zeroes>(expr)){
        return cg.createOnes();        
    }
    else if (Not * pe_not = dyn_cast<Not>(expr)) {
        return pe_not->getExpr();
    }
    return new Not(expr);
}

}
