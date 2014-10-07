/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "re_compiler.h"
//Regular Expressions
#include <re/re_name.h>
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
#include <pablo/pe_pabloe.h>
#include <pablo/pe_scanthru.h>
#include <pablo/pe_sel.h>
#include <pablo/pe_var.h>
#include <pablo/pe_xor.h>
#include <pablo/ps_assign.h>
#include <pablo/ps_if.h>
#include <pablo/ps_while.h>

#include <assert.h>
#include <stdexcept>

using namespace pablo;

namespace re {

RE_Compiler::RE_Compiler(PabloBlock & baseCG, std::map<std::string, std::string> name_map)
: mCG(baseCG)
, m_name_map(name_map)
{

}

void RE_Compiler::compile(RE * re, PabloBlock & cg) {

    if (hasUnicode(re)) {
        //Set the 'internal.initial' bit stream for the utf-8 multi-byte encoding.
        std::string gs_initial = cg.ssa("initial");
        m_name_map.insert(make_pair("initial", gs_initial));
        PabloE * u8single = cg.createVar(m_name_map.find("UTF8-SingleByte")->second);
        PabloE * u8pfx2 = cg.createVar(m_name_map.find("UTF8-Prefix2")->second);
        PabloE * u8pfx3 = cg.createVar(m_name_map.find("UTF8-Prefix3")->second);
        PabloE * u8pfx4 = cg.createVar(m_name_map.find("UTF8-Prefix4")->second);
        PabloE * u8pfx = cg.createOr(cg.createOr(u8pfx2, u8pfx3), u8pfx4);
        cg.createAssign(gs_initial, cg.createOr(u8pfx, u8single));

        //Set the 'internal.nonfinal' bit stream for the utf-8 multi-byte encoding.
        std::string gs_nonfinal = cg.ssa("nonfinal");
        m_name_map.insert(make_pair("nonfinal", gs_nonfinal));
        //#define USE_IF_FOR_NONFINAL
        #ifdef USE_IF_FOR_NONFINAL
        cg.createAssign(gs_nonfinal, make_all(0));
        #endif
        PabloE * u8scope32 = cg.createAdvance(u8pfx3);
        PabloE * u8scope42 = cg.createAdvance(u8pfx4);
        PabloE * u8scope43 = cg.createAdvance(u8scope42);
        #ifdef USE_IF_FOR_NONFINAL
        CodeGenState it(cg);
        it.createAssign(gs_nonfinal, it.createOr(it.createOr(u8pfx, u8scope32), it.createOr(u8scope42, u8scope43)));
        cg.createIf(u8pfx, std::move(it));
        #else
        cg.createAssign(gs_nonfinal, cg.createOr(cg.createOr(u8pfx, u8scope32), cg.createOr(u8scope42, u8scope43)));
        #endif
    }

    PabloE * start_marker = cg.createAll(1);
    PabloE * result = process(re, start_marker, cg);

    //These three lines are specifically for grep.
    cg.createAssign(cg.ssa("marker"), cg.createAnd(cg.createMatchStar(cg.createVarIfAssign(result),
                    cg.createNot(cg.createVar(m_name_map.find("LineFeed")->second))), cg.createVar(m_name_map.find("LineFeed")->second)));
}


PabloE * RE_Compiler::process(RE * re, PabloE * target, PabloBlock & cg) {
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
    else if (isa<Start>(re)) {
        target = cg.createAnd(cg.createVarIfAssign(target), cg.createNot(cg.createAdvance(cg.createNot(cg.createCharClass(m_name_map.find("LineFeed")->second)))));
    }
    else if (isa<End>(re)) {
        target = cg.createAnd(cg.createVarIfAssign(target), cg.createCharClass(m_name_map.find("LineFeed")->second));
    }

    return target;
}

inline PabloE * RE_Compiler::process(Name * name, PabloE * target, PabloBlock & cg) {
    PabloE * markerExpr = cg.createVarIfAssign(target);
    if (name->getType() != Name::Type::FixedLength) {
        // Move the markers forward through any nonfinal UTF-8 bytes to the final position of each character.
        markerExpr = cg.createAnd(markerExpr, cg.createCharClass(m_name_map.find("initial")->second));
        markerExpr = cg.createScanThru(markerExpr, cg.createCharClass(m_name_map.find("nonfinal")->second));
    }
    PabloE * cc = nullptr;
    if (name->getType() == Name::Type::UnicodeCategory) {
        cc = cg.createCall(name->getName());
    }
    else {
        cc = cg.createCharClass(name->getName());
    }
    if (name->isNegated()) {
        cc = cg.createNot(cg.createOr(cg.createOr(cc, cg.createCharClass(m_name_map.find("LineFeed")->second)),
                                cg.createCharClass(m_name_map.find("nonfinal")->second)));
    }
    return cg.createAssign(cg.ssa("marker"), cg.createAdvance(cg.createAnd(cc, markerExpr)));
}

inline PabloE * RE_Compiler::process(Seq * seq, PabloE * target, PabloBlock & cg) {
    for (RE * re : *seq) {
        target = process(re, target, cg);
    }
    return target;
}

inline PabloE * RE_Compiler::process(Alt * alt, PabloE * target, PabloBlock & cg) {
    if (alt->empty()) {
        target = cg.createAll(0); // always fail (note: I'm not sure this ever occurs. How do I create a 0-element alternation?)
    }
    else {
        auto i = alt->begin();
        PabloE * const base = target;
        target = cg.createVarIfAssign(process(*i, target, cg));
        while (++i != alt->end()) {
            PabloE * other = cg.createVarIfAssign(process(*i, base, cg));
            target = cg.createOr(target, other);
        }
    }    
    return target;
}

inline PabloE * RE_Compiler::process(Rep * rep, PabloE * target, PabloBlock & cg) {
    if (rep->getUB() == Rep::UNBOUNDED_REP) {
        target = processUnboundedRep(rep->getRE(), rep->getLB(), target, cg);
    }
    else { // if (rep->getUB() != Rep::UNBOUNDED_REP)
        target = processBoundedRep(rep->getRE(), rep->getLB(), rep->getUB(), target, cg);
    }    
    return target;
}

inline PabloE * RE_Compiler::processUnboundedRep(RE * repeated, int lb, PabloE * target, PabloBlock & cg) {
    while (lb-- != 0) {
        target = process(repeated, target, cg);
    }

    target = cg.createVarIfAssign(target);

    if (isa<Name>(repeated)) {
        Name * rep_name = dyn_cast<Name>(repeated);

        PabloE * ccExpr;
        if (rep_name->getType() == Name::Type::UnicodeCategory) {
            ccExpr = cg.createCall(rep_name->getName());
        }
        else {
            ccExpr = cg.createCharClass(rep_name->getName());
        }

        if (rep_name->isNegated()) {
            ccExpr = cg.createNot(cg.createOr(cg.createOr(ccExpr, cg.createCharClass(m_name_map.find("LineFeed")->second)), cg.createCharClass(m_name_map.find("nonfinal")->second)));
        }

        if (rep_name->getType() == Name::Type::FixedLength) {
            target = cg.createMatchStar(target, ccExpr);
        }
        else { // Name::Unicode and Name::UnicodeCategory
            target = cg.createAnd(cg.createMatchStar(target, cg.createOr(cg.createCharClass(m_name_map.find("nonfinal")->second), ccExpr)), cg.createCharClass(m_name_map.find("initial")->second));
        }
    }
    else {

        Assign * while_test = cg.createAssign(cg.ssa("while_test"), target);
        Assign * while_accum = cg.createAssign(cg.ssa("while_accum"), target);

        PabloBlock wt(cg);
        PabloE * accum = cg.createVarIfAssign(process(repeated, while_test, wt));
        Var * var_while_test = cg.createVar(while_accum);
        wt.createAssign(while_test->getName(), wt.createAnd(accum, wt.createNot(var_while_test)));
        target = wt.createAssign(while_accum->getName(), wt.createOr(var_while_test, accum));
        cg.createWhile(cg.createVar(while_test), std::move(wt));
    }    
    return target;
}

inline PabloE * RE_Compiler::processBoundedRep(RE * repeated, int lb, int ub, PabloE * target, PabloBlock & cg) {
    ub -= lb;
    while(lb-- != 0) {
        target = process(repeated, target, cg);
    }
    while (ub-- != 0) {
        PabloE * alt = process(repeated, target, cg);
        target = cg.createOr(cg.createVarIfAssign(target), cg.createVarIfAssign(alt));
    }
    return target;
}


bool RE_Compiler::hasUnicode(const RE * re) {
    bool found = false;
    if (re == nullptr) {
        throw std::runtime_error("Unexpected Null Value passed to RE Compiler!");
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
