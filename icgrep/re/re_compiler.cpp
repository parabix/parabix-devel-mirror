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

RE_Compiler::RE_Compiler(CodeGenState & baseCG, std::map<std::string, std::string> name_map)
: mBaseCG(baseCG)
, m_name_map(name_map)
{

}

inline PabloE * makeVarIfAssign(PabloE * const input) {
    return isa<Assign>(input) ? makeVar(cast<Assign>(input)) : input;
}

void RE_Compiler::compile(RE * re) {

    if (hasUnicode(re)) {
        //Set the 'internal.initial' bit stream for the utf-8 multi-byte encoding.
        std::string gs_initial = mBaseCG.symgen("initial");
        m_name_map.insert(make_pair("initial", gs_initial));
        PabloE * u8single = makeVar(m_name_map.find("UTF8-SingleByte")->second);
        PabloE * u8pfx2 = makeVar(m_name_map.find("UTF8-Prefix2")->second);
        PabloE * u8pfx3 = makeVar(m_name_map.find("UTF8-Prefix3")->second);
        PabloE * u8pfx4 = makeVar(m_name_map.find("UTF8-Prefix4")->second);
        PabloE * u8pfx = makeOr(makeOr(u8pfx2, u8pfx3), u8pfx4);
        mBaseCG.push_back(makeAssign(gs_initial, makeOr(u8pfx, u8single)));

        //Set the 'internal.nonfinal' bit stream for the utf-8 multi-byte encoding.
        std::string gs_nonfinal = mBaseCG.symgen("nonfinal");
        m_name_map.insert(make_pair("nonfinal", gs_nonfinal));
        //#define USE_IF_FOR_NONFINAL
        #ifdef USE_IF_FOR_NONFINAL
        cg.push_back(make_assign(gs_nonfinal, make_all(0)));
        #endif
        PabloE * u8scope32 = makeAdvance(u8pfx3);
        PabloE * u8scope42 = makeAdvance(u8pfx4);
        PabloE * u8scope43 = makeAdvance(u8scope42);
        PabloE * assign_non_final = makeAssign(gs_nonfinal, makeOr(makeOr(u8pfx, u8scope32), makeOr(u8scope42, u8scope43)));
        #ifdef USE_IF_FOR_NONFINAL
        std::list<PabloE *> * if_body = new std::list<PabloE *> ();
        if_body->push_back(assign_non_final);
        cg.push_back(makeIf(u8pfx, *if_body));
        #else
        mBaseCG.push_back(assign_non_final);
        #endif
    }

    PabloE * start_marker = mBaseCG.createAll(1);
    mBaseCG.push_back(start_marker);
    PabloE * result = process(re, start_marker, mBaseCG);

    //These three lines are specifically for grep.
    Assign * final = makeAssign(mBaseCG.symgen("marker"), makeAnd(makeMatchStar(makeVarIfAssign(result),
        makeNot(makeVar(m_name_map.find("LineFeed")->second))), makeVar(m_name_map.find("LineFeed")->second)));
    mBaseCG.push_back(final);
}

PabloE * RE_Compiler::process(RE * re, PabloE * target, CodeGenState & cg) {
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
        target = makeAnd(makeVarIfAssign(target), makeNot(makeAdvance(makeNot(makeCharClass(m_name_map.find("LineFeed")->second)))));
        cg.push_back(target);
    }
    else if (isa<End>(re)) {
        target = makeAnd(makeVarIfAssign(target), makeCharClass(m_name_map.find("LineFeed")->second));
        cg.push_back(target);
    }

    return target;
}

inline PabloE * RE_Compiler::process(Name * name, PabloE * target, CodeGenState & cg) {
    PabloE * markerExpr = makeVarIfAssign(target);
    if (name->getType() != Name::Type::FixedLength) {
        // Move the markers forward through any nonfinal UTF-8 bytes to the final position of each character.
        markerExpr = makeAnd(markerExpr, makeCharClass(m_name_map.find("initial")->second));
        markerExpr = makeScanThru(markerExpr, makeCharClass(m_name_map.find("nonfinal")->second));
    }
    PabloE * cc = nullptr;
    if (name->getType() == Name::Type::UnicodeCategory) {
        cc = makeCall(name->getName());
    }
    else {
        cc = makeCharClass(name->getName());
    }
    if (name->isNegated()) {
        cc = makeNot(makeOr(makeOr(cc, makeCharClass(m_name_map.find("LineFeed")->second)),
                                makeCharClass(m_name_map.find("nonfinal")->second)));
    }
    target = makeAssign(cg.symgen("marker"), makeAdvance(makeAnd(cc, markerExpr)));
    cg.push_back(target);
    return target;
}

inline PabloE * RE_Compiler::process(Seq * seq, PabloE * target, CodeGenState & cg) {
    for (RE * re : *seq) {
        target = process(re, target, cg);
    }
    cg.push_back(target);
    return target;
}

inline PabloE * RE_Compiler::process(Alt * alt, PabloE * target, CodeGenState & cg) {
    if (alt->empty()) {
        target = cg.createAll(0); // always fail (note: I'm not sure this ever occurs. How do I create a 0-element alternation?)
    }
    else {
        auto i = alt->begin();
        PabloE * const base = target;
        target = makeVarIfAssign(process(*i, target, cg));
        while (++i != alt->end()) {
            PabloE * other = makeVarIfAssign(process(*i, base, cg));
            target = makeOr(target, other);
        }
        cg.push_back(target);
    }    
    return target;
}

inline PabloE * RE_Compiler::process(Rep * rep, PabloE * target, CodeGenState & cg) {
    if (rep->getUB() == Rep::UNBOUNDED_REP) {
        target = processUnboundedRep(rep->getRE(), rep->getLB(), target, cg);
    }
    else { // if (rep->getUB() != Rep::UNBOUNDED_REP)
        target = processBoundedRep(rep->getRE(), rep->getLB(), rep->getUB(), target, cg);
    }    
    return target;
}

inline PabloE * RE_Compiler::processUnboundedRep(RE * repeated, int lb, PabloE * target, CodeGenState & cg) {
    while (lb-- != 0) {
        target = process(repeated, target, cg);
    }
    if (isa<Name>(repeated)) {
        Name * rep_name = dyn_cast<Name>(repeated);

        PabloE * ccExpr;
        if (rep_name->getType() == Name::Type::UnicodeCategory) {
            ccExpr = makeCall(rep_name->getName());
        }
        else {
            ccExpr = makeCharClass(rep_name->getName());
        }

        if (rep_name->isNegated()) {
            ccExpr = makeNot(makeOr(makeOr(ccExpr, makeCharClass(m_name_map.find("LineFeed")->second)), makeCharClass(m_name_map.find("nonfinal")->second)));
        }

        if (rep_name->getType() == Name::Type::FixedLength) {
            target = makeMatchStar(makeVarIfAssign(target), ccExpr);
        }
        else { // Name::Unicode and Name::UnicodeCategory
            target = makeAnd(makeMatchStar(makeVarIfAssign(target), makeOr(makeCharClass(m_name_map.find("nonfinal")->second), ccExpr)), makeCharClass(m_name_map.find("initial")->second));
        }
    }
    else {

        Assign * while_test = makeAssign(cg.symgen("while_test"), makeVarIfAssign(target));
        Assign * while_accum = makeAssign(cg.symgen("while_accum"), makeVarIfAssign(target));

        CodeGenState wt(cg);

        PabloE * accum = makeVarIfAssign(process(repeated, while_test, wt));

        cg.push_back(while_test);
        cg.push_back(while_accum);

        Var * var_while_test = makeVar(while_accum);

        wt.push_back(makeAssign(while_test->getName(), makeAnd(accum, makeNot(var_while_test))));

        target = makeAssign(while_accum->getName(), makeOr(var_while_test, accum));

        wt.push_back(target);
        cg.push_back(makeWhile(makeVar(while_test), wt.expressions()));
    }    
    return target;
}

inline PabloE * RE_Compiler::processBoundedRep(RE * repeated, int lb, int ub, PabloE * target, CodeGenState & cg) {
    ub -= lb;
    while(lb-- != 0) {
        target = process(repeated, target, cg);
    }
    while (ub-- != 0) {
        PabloE * alt = process(repeated, target, cg);
        target = makeOr(makeVarIfAssign(target), makeVarIfAssign(alt));
    }
    cg.push_back(target);
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
