/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "pabloAST.h"
#include "pe_advance.h"
#include "pe_and.h"
#include "pe_call.h"
#include "pe_matchstar.h"
#include "pe_not.h"
#include "pe_or.h"
#include "pabloAST.h"
#include "pe_scanthru.h"
#include "pe_sel.h"
#include "pe_var.h"
#include "pe_xor.h"
#include "pe_zeroes.h"
#include "pe_ones.h"
#include <pablo/codegenstate.h>
#include <llvm/Support/Compiler.h>

namespace pablo {

PabloAST::Allocator PabloAST::mAllocator;

/*

    Return true if expr1 and expr2 can be proven equivalent according to some rules,
    false otherwise.  Note that false may be returned i some cases when the exprs are
    equivalent.

*/

bool equals(const PabloAST * expr1, const PabloAST * expr2) {
    assert (expr1 && expr2);
    if (expr1->getClassTypeId() == expr2->getClassTypeId()) {
        if ((isa<const Zeroes>(expr1)) || (isa<const Ones>(expr1))) {
            return true;
        }
        else if (const Var * var1 = dyn_cast<const Var>(expr1)) {
            if (const Var * var2 = cast<const Var>(expr2)) {
                return (var1->getName() == var2->getName());
            }
        }
        else if (const Not* not1 = dyn_cast<const Not>(expr1)) {
            if (const Not* not2 = cast<const Not>(expr2)) {
                return equals(not1->getExpr(), not2->getExpr());
            }
        }
        else if (const And* and1 = dyn_cast<const And>(expr1)) {
            if (const And* and2 = cast<const And>(expr2)) {
                if (equals(and1->getExpr1(), and2->getExpr1())) {
                    return equals(and1->getExpr2(), and2->getExpr2());
                }
                else if (equals(and1->getExpr1(), and2->getExpr2())) {
                    return equals(and1->getExpr2(), and2->getExpr1());
                }
            }
        }
        else if (const Or * or1 = dyn_cast<const Or>(expr1)) {
            if (const Or* or2 = cast<const Or>(expr2)) {
                if (equals(or1->getExpr1(), or2->getExpr1())) {
                    return equals(or1->getExpr2(), or2->getExpr2());
                }
                else if (equals(or1->getExpr1(), or2->getExpr2())) {
                    return equals(or1->getExpr2(), or2->getExpr1());
                }
            }
        }
        else if (const Xor * xor1 = dyn_cast<const Xor>(expr1)) {
            if (const Xor * xor2 = cast<const Xor>(expr2)) {
                if (equals(xor1->getExpr1(), xor2->getExpr1())) {
                    return equals(xor1->getExpr2(), xor2->getExpr2());
                }
                else if (equals(xor1->getExpr1(), xor2->getExpr2())) {
                    return equals(xor1->getExpr2(), xor2->getExpr1());
                }
            }
        }
        else if (const Sel* sel1 = dyn_cast<const Sel>(expr1)) {
            if (const Sel* sel2 = cast<const Sel>(expr2)) {
                if (equals(sel1->getCondition(), sel2->getCondition())) {
                    if (equals(sel1->getTrueExpr(), sel2->getTrueExpr())) {
                        return equals(sel1->getFalseExpr(), sel2->getFalseExpr());
                    }
                }
            }
        }
    }
    return false;
}

void PabloAST::setMetadata(const std::string & name, PMDNode * node) {
    if (LLVM_UNLIKELY(mMetadataMap == nullptr)) {
        mMetadataMap = new PMDNodeMap();
    }
    mMetadataMap->insert(std::make_pair(name, node));
}

PMDNode * PabloAST::getMetadata(const std::string & name) {
    if (LLVM_UNLIKELY(mMetadataMap == nullptr)) {
        return nullptr;
    }
    auto f = mMetadataMap->find(name);
    if (f == mMetadataMap->end()) {
        return nullptr;
    }
    return f->second;
}


void Statement::insertBefore(Statement * const statement) {
    assert (statement);
    assert (statement != this);
    assert (statement->mParent);
    removeFromParent();
    mParent = statement->mParent;
    if (LLVM_UNLIKELY(mParent->mFirst == statement)) {
        mParent->mFirst = this;
    }
    mNext = statement;
    mPrev = statement->mPrev;
    statement->mPrev = this;
    if (LLVM_LIKELY(mPrev != nullptr)) {
        mPrev->mNext = this;
    }
}

void Statement::insertAfter(Statement * const statement) {
    assert (statement);
    assert (statement != this);
    assert (statement->mParent);
    removeFromParent();
    mParent = statement->mParent;
    if (LLVM_UNLIKELY(mParent->mLast == statement)) {
        mParent->mLast = this;
    }
    mPrev = statement;
    mNext = statement->mNext;
    statement->mNext = this;
    if (LLVM_LIKELY(mNext != nullptr)) {
        mNext->mPrev = this;
    }
}

void Statement::removeFromParent() {
    if (LLVM_LIKELY(mParent != nullptr)) {
        if (LLVM_UNLIKELY(mParent->mFirst == this)) {
            mParent->mFirst = mNext;
        }
        if (LLVM_UNLIKELY(mParent->mLast == this)) {
            mParent->mLast = mPrev;
        }
        if (LLVM_LIKELY(mPrev != nullptr)) {
            mPrev->mNext = mNext;
        }
        if (LLVM_LIKELY(mNext != nullptr)) {
            mNext->mPrev = mPrev;
        }
    }
    mPrev = nullptr;
    mNext = nullptr;
    mParent = nullptr;
}

void Statement::replaceWith(Statement * const statement) {
    if (LLVM_UNLIKELY(mParent != nullptr)) {
        statement->removeFromParent();
    }
    statement->mParent = mParent;
    statement->mNext = mNext;
    statement->mPrev = mPrev;
    if (LLVM_LIKELY(mPrev != nullptr)) {
        mPrev->mNext = statement;
    }
    if (LLVM_LIKELY(mNext != nullptr)) {
        mNext->mPrev = statement;
    }
    mParent=nullptr;
    mNext=nullptr;
    mPrev=nullptr;
}

Statement::~Statement() {

}

void StatementList::push_back(Statement * const statement) {
    if (LLVM_UNLIKELY(mLast == nullptr)) {
        mFirst = mLast = statement;
    }
    else {
        statement->insertAfter(mLast);
        mLast = statement;
    }
}

}
