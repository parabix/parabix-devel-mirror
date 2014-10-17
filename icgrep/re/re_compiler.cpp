/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "re_compiler.h"
//Regular Expressions
#include <re/re_name.h>
#include <re/re_any.h>
#include <re/re_start.h>
#include <re/re_end.h>
#include <re/re_alt.h>
#include <re/re_cc.h>
#include <re/re_seq.h>
#include <re/re_rep.h>


//Pablo Expressions
#include <pablo/codegenstate.h>
#include <pablo/pe_advance.h>
#include <pablo/pe_all.h>
#include <pablo/pe_and.h>
#include <pablo/pe_call.h>
#include <pablo/pe_charclass.h>
#include <pablo/pe_matchstar.h>
#include <pablo/pe_not.h>
#include <pablo/pe_or.h>
#include <pablo/pabloAST.h>
#include <pablo/pe_scanthru.h>
#include <pablo/pe_sel.h>
#include <pablo/pe_var.h>
#include <pablo/pe_xor.h>
#include <pablo/ps_assign.h>
#include <pablo/ps_if.h>
#include <pablo/ps_while.h>

#include <assert.h>
#include <stdexcept>

//Set the 'internal.nonfinal' bit stream for the utf-8 multi-byte encoding.
//#define USE_IF_FOR_NONFINAL

using namespace pablo;

namespace re {

RE_Compiler::RE_Compiler(PabloBlock & baseCG, std::map<std::string, std::string> name_map)
: mCG(baseCG)
, mLineFeed(nullptr)
, mInitial(nullptr)
, mNonFinal(nullptr)
, m_name_map(name_map)
{

}

void RE_Compiler::compile(RE * re, PabloBlock & cg) {

    mLineFeed = cg.createVar(m_name_map.find("LineFeed")->second);

    const std::string initial = "initial";
    const std::string nonfinal = "nonfinal";

    if (hasUnicode(re)) {
        //Set the 'internal.initial' bit stream for the utf-8 multi-byte encoding.        
        PabloAST * u8single = cg.createVar(m_name_map.find("UTF8-SingleByte")->second);
        PabloAST * u8pfx2 = cg.createVar(m_name_map.find("UTF8-Prefix2")->second);
        PabloAST * u8pfx3 = cg.createVar(m_name_map.find("UTF8-Prefix3")->second);
        PabloAST * u8pfx4 = cg.createVar(m_name_map.find("UTF8-Prefix4")->second);
        PabloAST * u8pfx = cg.createOr(cg.createOr(u8pfx2, u8pfx3), u8pfx4);
        mInitial = cg.createVar(cg.createAssign(initial, cg.createOr(u8pfx, u8single)));
        #ifdef USE_IF_FOR_NONFINAL
        mNonFinal = cg.createVar(cg.createAssign(gs_nonfinal, cg.createAll(0)));
        #endif
        PabloAST * u8scope32 = cg.createAdvance(u8pfx3);
        PabloAST * u8scope42 = cg.createAdvance(u8pfx4);
        PabloAST * u8scope43 = cg.createAdvance(u8scope42);
        #ifdef USE_IF_FOR_NONFINAL
        PabloBlock it(cg);
        it.createAssign(gs_nonfinal, it.createOr(it.createOr(u8pfx, u8scope32), it.createOr(u8scope42, u8scope43)));
        cg.createIf(u8pfx, std::move(it));
        #else        
        mNonFinal = cg.createVar(cg.createAssign(nonfinal, cg.createOr(cg.createOr(u8pfx, u8scope32), cg.createOr(u8scope42, u8scope43))));
        #endif
    }
    else {
        mInitial = cg.createAll(0);
        mNonFinal = cg.createAll(0);
    }

    Assign * start_marker = cg.createAssign("start", cg.createAll(1));
    PabloAST * result = process(re, start_marker, cg);

    //These three lines are specifically for grep.
    cg.createAssign(cg.ssa("marker"), cg.createAnd(cg.createMatchStar(cg.createVarIfAssign(result), cg.createNot(mLineFeed)), mLineFeed));
}


Assign * RE_Compiler::process(RE * re, Assign * target, PabloBlock & cg) {
    if (Name * name = dyn_cast<Name>(re)) {
        target = process(name, target, cg);
    }
    else if (Seq* seq = dyn_cast<Seq>(re)) {
        target = process(seq, target, cg);
    }
    else if (Alt * alt = dyn_cast<Alt>(re)) {
        target = process(alt, target, cg);
    }
    else if (Rep * rep = dyn_cast<Rep>(re)) {
        target = process(rep, target, cg);
    }
    else if (isa<Any>(re)) {
        // Move the markers forward through any nonfinal UTF-8 bytes to the final position of each character.
		PabloAST * marker = cg.createVar(target);
        marker = cg.createAnd(marker, mInitial);
        marker = cg.createScanThru(marker, mNonFinal);
        PabloAST * dot = cg.createNot(mLineFeed);
        target = cg.createAssign(cg.ssa("dot"), cg.createAdvance(cg.createAnd(marker, dot)));
    }
    else if (isa<Start>(re)) {
        PabloAST * sol = cg.createNot(cg.createAdvance(cg.createNot(mLineFeed)));
        target = cg.createAssign(cg.ssa("sol"), cg.createAnd(cg.createVarIfAssign(target), sol));
    }
    else if (isa<End>(re)) {
        PabloAST * eol = mLineFeed;
        target = cg.createAssign(cg.ssa("eol"), cg.createAnd(cg.createVarIfAssign(target), eol));
    }

    return target;
}

inline Assign * RE_Compiler::process(Name * name, Assign * target, PabloBlock & cg) {
    PabloAST * marker = cg.createVar(target);
    if (name->getType() != Name::Type::FixedLength) {
        // Move the markers forward through any nonfinal UTF-8 bytes to the final position of each character.
        marker = cg.createAnd(marker, mInitial);
        marker = cg.createScanThru(marker, mNonFinal);
    }
    PabloAST * cc = nullptr;
    if (name->getType() == Name::Type::UnicodeCategory) {
        cc = cg.createCall(name->getName());
    }
    else {
        cc = cg.createCharClass(name->getName());
    }
    if (name->isNegated()) {
        cc = cg.createNot(cg.createOr(cg.createOr(cc, mLineFeed), mNonFinal));
    }
    return cg.createAssign(cg.ssa("marker"), cg.createAdvance(cg.createAnd(cc, marker)));
}

inline Assign * RE_Compiler::process(Seq * seq, Assign *target, PabloBlock & cg) {
    for (RE * re : *seq) {
        target = process(re, target, cg);
    }
    return target;
}

inline Assign * RE_Compiler::process(Alt * alt, Assign * target, PabloBlock & cg) {
    if (alt->empty()) {
        target = cg.createAssign(cg.ssa("fail"), cg.createAll(0)); // always fail (note: I'm not sure this ever occurs. How do I create a 0-element alternation?)
    }
    else {
        auto i = alt->begin();
        Assign * const base = target;
        target = process(*i, base, cg);
        while (++i != alt->end()) {
            Assign * other = process(*i, base, cg);
            target = cg.createAssign(cg.ssa("alt"), cg.createOr(cg.createVar(target), cg.createVar(other)));
        }
    }    
    return target;
}

inline Assign * RE_Compiler::process(Rep * rep, Assign * target, PabloBlock & cg) {
    if (rep->getUB() == Rep::UNBOUNDED_REP) {
        target = processUnboundedRep(rep->getRE(), rep->getLB(), target, cg);
    }
    else { // if (rep->getUB() != Rep::UNBOUNDED_REP)
        target = processBoundedRep(rep->getRE(), rep->getLB(), rep->getUB(), target, cg);
    }    
    return target;
}

inline Assign * RE_Compiler::processUnboundedRep(RE * repeated, int lb, Assign * target, PabloBlock & cg) {
    while (lb-- != 0) {
        target = process(repeated, target, cg);
    }

    if (isa<Name>(repeated)) {
        Name * rep_name = dyn_cast<Name>(repeated);

        PabloAST * cc;
        if (rep_name->getType() == Name::Type::UnicodeCategory) {
            cc = cg.createCall(rep_name->getName());
        }
        else {
            cc = cg.createCharClass(rep_name->getName());
        }

        if (rep_name->isNegated()) {
            cc = cg.createNot(cg.createOr(cg.createOr(cc, mLineFeed), mNonFinal));
        }

        PabloAST * unbounded = cg.createVar(target);
        if (rep_name->getType() == Name::Type::FixedLength) {
            unbounded = cg.createMatchStar(unbounded, cc);
        }
        else { // Name::Unicode and Name::UnicodeCategory
            unbounded = cg.createAnd(cg.createMatchStar(unbounded, cg.createOr(mNonFinal, cc)), mInitial);
        }
        target = cg.createAssign(cg.ssa("marker"), unbounded);
    }
    else if (isa<Any>(repeated)) {
        PabloAST * dot = cg.createNot(mLineFeed);
        PabloAST * unbounded = cg.createVar(target);
        unbounded = cg.createAnd(cg.createMatchStar(unbounded, cg.createOr(mNonFinal, dot)), mInitial);
        target = cg.createAssign(cg.ssa("marker"), unbounded);
    }

    else {

        Var * targetVar = cg.createVar(target);

        Assign * while_test = cg.createAssign(cg.ssa("while_test"), targetVar);
        Assign * while_accum = cg.createAssign(cg.ssa("while_accum"), targetVar);

        PabloBlock wt(cg);
        PabloAST * accum = cg.createVarIfAssign(process(repeated, while_test, wt));
        Var * var_while_test = cg.createVar(while_accum);
        wt.createAssign(while_test->getName(), wt.createAnd(accum, wt.createNot(var_while_test)));
        target = wt.createAssign(while_accum->getName(), wt.createOr(var_while_test, accum));
        cg.createWhile(cg.createVar(while_test), std::move(wt));
    }    
    return target;
}

inline Assign * RE_Compiler::processBoundedRep(RE * repeated, int lb, int ub, Assign * target, PabloBlock & cg) {
    ub -= lb;
    while(lb-- != 0) {
        target = process(repeated, target, cg);
    }
    while (ub-- != 0) {
        Assign * alt = process(repeated, target, cg);
        target = cg.createAssign(cg.ssa("alt"), cg.createOr(cg.createVar(target), cg.createVar(alt)));
    }
    return target;
}


bool RE_Compiler::hasUnicode(const RE * re) {
    bool found = false;
    if (re == nullptr) {
        throw std::runtime_error("Unexpected Null Value passed to RE Compiler!");
    }
    else if (isa<Any>(re)) {
        found = true;
    }
    else if (const Name * name = dyn_cast<const Name>(re)) {
        if ((name->getType() == Name::Type::UnicodeCategory) || (name->getType() == Name::Type::Unicode)) {
            found = true;
        }
    }
    else if (const Seq * re_seq = dyn_cast<const Seq>(re)) {
        for (auto i = re_seq->cbegin(); i != re_seq->cend(); ++i) {
            if (hasUnicode(*i)) {
                found = true;
                break;
            }
        }
    }
    else if (const Alt * re_alt = dyn_cast<const Alt>(re)) {
        for (auto i = re_alt->cbegin(); i != re_alt->cend(); ++i) {
            if (hasUnicode(*i)) {
                found = true;
                break;
            }
        }
    }
    else if (const Rep * rep = dyn_cast<const Rep>(re)) {
        found = hasUnicode(rep->getRE());
    }
    return found;
}

} // end of namespace re
