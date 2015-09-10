#include "pabloverifier.hpp"
#include <pablo/function.h>
#include <pablo/codegenstate.h>
#include <pablo/printer_pablos.h>
#include <unordered_set>

namespace pablo {

struct OrderingVerifier {
    OrderingVerifier() : mParent(nullptr) {}
    OrderingVerifier(const OrderingVerifier & parent) : mParent(&parent) {}
    bool count(const PabloAST * expr) const {
        if (mSet.count(expr)) {
            return true;
        } else if (mParent) {
            return mParent->count(expr);
        }
        return false;
    }
    void insert(const PabloAST * expr) {
        mSet.insert(expr);
    }
private:
    const OrderingVerifier * const mParent;
    std::unordered_set<const PabloAST *> mSet;
};

void isTopologicallyOrdered(const PabloBlock & block, const OrderingVerifier & parent) {
    OrderingVerifier ov(parent);
    for (const Statement * stmt : block) {
        for (unsigned i = 0; i != stmt->getNumOperands(); ++i) {
            const PabloAST * op = stmt->getOperand(i);
            if ((isa<Statement>(op) || isa<Var>(op)) && ov.count(op) == false) {
                // TODO: make this actually test whether the operand is ever defined,
                // or if it was defined in a scope that cannot be reached?
                std::string tmp;
                raw_string_ostream str(tmp);
                str << "PabloVerifier: function is not topologically ordered! ";
                PabloPrinter::print(stmt->getOperand(i), str);
                str << " was used before definition!";
                throw std::runtime_error(str.str());
            }
        }
        ov.insert(stmt);
        if (LLVM_UNLIKELY(isa<If>(stmt))) {
            isTopologicallyOrdered(cast<If>(stmt)->getBody(), ov);
            for (const Assign * def : cast<If>(stmt)->getDefined()) {
                ov.insert(def);
            }
        } else if (LLVM_UNLIKELY(isa<While>(stmt))) {
            isTopologicallyOrdered(cast<While>(stmt)->getBody(), ov);
            for (const Next * var : cast<While>(stmt)->getVariants()) {
                ov.insert(var);
            }
        }
    }
}

void isTopologicallyOrdered(const PabloFunction & function) {
    OrderingVerifier ov;
    for (unsigned i = 0; i != function.getNumOfParameters(); ++i) {
        ov.insert(function.getParameter(i));
    }
    isTopologicallyOrdered(function.getEntryBlock(), ov);
}

void PabloVerifier::verify(const PabloFunction & function) {
    isTopologicallyOrdered(function);
}

}
