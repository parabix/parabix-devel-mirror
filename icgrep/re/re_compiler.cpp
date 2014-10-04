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

void RE_Compiler::compile(RE * re) {

    std::string gs_m0 = mBaseCG.symgen("start_marker");
    mBaseCG.push_back(makeAssign(gs_m0, makeAll(1)));

    if (hasUnicode(re)) {
        mBaseCG.newsym = gs_m0;
        //Set the 'internal.initial' bit stream for the utf-8 multi-byte encoding.
        std::string gs_initial = mBaseCG.symgen("internal.initial");
        m_name_map.insert(make_pair("internal.initial", gs_initial));
        PabloE * u8single = makeVar(m_name_map.find("UTF8-SingleByte")->second);
        PabloE * u8pfx2 = makeVar(m_name_map.find("UTF8-Prefix2")->second);
        PabloE * u8pfx3 = makeVar(m_name_map.find("UTF8-Prefix3")->second);
        PabloE * u8pfx4 = makeVar(m_name_map.find("UTF8-Prefix4")->second);
        PabloE * u8pfx = makeOr(makeOr(u8pfx2, u8pfx3), u8pfx4);
        mBaseCG.push_back(makeAssign(gs_initial, makeOr(u8pfx, u8single)));
        mBaseCG.newsym = gs_initial;

        //Set the 'internal.nonfinal' bit stream for the utf-8 multi-byte encoding.
        mBaseCG.newsym = gs_m0;
        std::string gs_nonfinal = mBaseCG.symgen("internal.nonfinal");
        m_name_map.insert(make_pair("internal.nonfinal", gs_nonfinal));
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
        cg.push_back(new If(u8pfx, *if_body));
        #else
        mBaseCG.push_back(assign_non_final);
        #endif
        mBaseCG.newsym = gs_nonfinal;
    }

    mBaseCG.newsym = gs_m0;
    process(re, mBaseCG);

    //These three lines are specifically for grep.
    std::string gs_retVal = mBaseCG.symgen("marker");
    mBaseCG.push_back(makeAssign(gs_retVal, makeAnd(makeMatchStar(makeVar(mBaseCG.newsym),
        makeNot(makeVar(m_name_map.find("LineFeed")->second))), makeVar(m_name_map.find("LineFeed")->second))));
    mBaseCG.newsym = gs_retVal;
}

void RE_Compiler::process(RE * re, CodeGenState & cg) {
    if (Name * name = dyn_cast<Name>(re)) {
        process(name, cg);
    }
    else if (Seq* seq = dyn_cast<Seq>(re)) {
        process(seq, cg);
    }
    else if (Alt * alt = dyn_cast<Alt>(re)) {
        process(alt, cg);
    }
    else if (Rep * rep = dyn_cast<Rep>(re)) {
        process(rep, cg);
    }
    else if (isa<Start>(re)) {
        std::string gs_retVal = cg.symgen("sol");
        cg.push_back(makeAssign(gs_retVal, makeAnd(makeVar(cg.newsym), makeNot(makeAdvance(makeNot(makeCharClass(m_name_map.find("LineFeed")->second)))))));
        cg.newsym = gs_retVal;
    }
    else if (isa<End>(re)) {
        std::string gs_retVal = cg.symgen("eol");
        cg.push_back(makeAssign(gs_retVal, makeAnd(makeVar(cg.newsym), makeCharClass(m_name_map.find("LineFeed")->second))));
        cg.newsym = gs_retVal;
    }
}

inline void RE_Compiler::process(Name * name, CodeGenState & cg) {
    std::string gs_retVal = cg.symgen("marker");
    PabloE * markerExpr = makeVar(cg.newsym);
    if (name->getType() != Name::Type::FixedLength) {
        // Move the markers forward through any nonfinal UTF-8 bytes to the final position of each character.
        markerExpr = makeAnd(markerExpr, makeCharClass(m_name_map.find("internal.initial")->second));
        markerExpr = new ScanThru(markerExpr, makeCharClass(m_name_map.find("internal.nonfinal")->second));
    }
    PabloE * ccExpr;
    if (name->getType() == Name::Type::UnicodeCategory) {
        ccExpr = makeCall(name->getName());
    }
    else {
        ccExpr = makeCharClass(name->getName());
    }
    if (name->isNegated()) {
        ccExpr = makeNot(makeOr(makeOr(ccExpr, makeCharClass(m_name_map.find("LineFeed")->second)),
                                makeCharClass(m_name_map.find("internal.nonfinal")->second)));
    }
    cg.push_back(makeAssign(gs_retVal, makeAdvance(makeAnd(ccExpr, markerExpr))));
    cg.newsym = gs_retVal;
}

inline void RE_Compiler::process(Seq * seq, CodeGenState & cg) {
    for (RE * re : *seq) {
        process(re, cg);
    }
}

inline void RE_Compiler::process(Alt * alt, CodeGenState & cg) {
    if (alt->empty()) {
        std::string gs_retVal = cg.symgen("always_fail_marker");
        cg.push_back(makeAssign(gs_retVal, makeAll(0)));
        cg.newsym = gs_retVal;
    }
    else {
        auto i = alt->begin();
        const std::string startsym = cg.newsym;
        process(*i, cg);
        while (++i != alt->end()) {
            std::string alt1 = cg.newsym;
            cg.newsym = startsym;
            process(*i, cg);
            std::string newsym = cg.symgen("alt");
            cg.push_back(makeAssign(newsym, makeOr(makeVar(alt1), makeVar(cg.newsym))));
            cg.newsym = newsym;
        }
    }
}

inline void RE_Compiler::process(Rep * rep, CodeGenState & cg) {
    if (rep->getUB() == Rep::UNBOUNDED_REP) {
        processUnboundedRep(rep->getRE(), rep->getLB(), cg);
    }
    else { // if (rep->getUB() != Rep::UNBOUNDED_REP)
        processBoundedRep(rep->getRE(), rep->getLB(), rep->getUB(), cg);
    }
}

inline void RE_Compiler::processUnboundedRep(RE * repeated, int lb, CodeGenState & cg) {
    while (lb-- != 0) {
        process(repeated, cg);
    }
    if (isa<Name>(repeated)) {
        Name * rep_name = dyn_cast<Name>(repeated);
        std::string gs_retVal = cg.symgen("marker");

        PabloE* ccExpr;
        if (rep_name->getType() == Name::Type::UnicodeCategory) {
            ccExpr = makeCall(rep_name->getName());
        }
        else {
            ccExpr = makeCharClass(rep_name->getName());
        }

        if (rep_name->isNegated()) {
            ccExpr = makeNot(makeOr(makeOr(ccExpr, makeCharClass(m_name_map.find("LineFeed")->second)), makeCharClass(m_name_map.find("internal.nonfinal")->second)));
        }
        if (rep_name->getType() == Name::Type::FixedLength) {
            cg.push_back(makeAssign(gs_retVal, makeMatchStar(makeVar(cg.newsym), ccExpr)));
        }
        else { // Name::Unicode and Name::UnicodeCategory
            cg.push_back(makeAssign(gs_retVal,
                makeAnd(makeMatchStar(makeVar(cg.newsym),
                        makeOr(makeCharClass(m_name_map.find("internal.nonfinal")->second), ccExpr)),
                               makeCharClass(m_name_map.find("internal.initial")->second))));
        }
        cg.newsym = gs_retVal;
    }
    else {
      std::string while_test = cg.symgen("while_test");
      std::string while_accum = cg.symgen("while_accum");

      CodeGenState wt(cg);

      wt.newsym = while_test;
      process(repeated, wt);

      cg.push_back(makeAssign(while_test, makeVar(cg.newsym)));
      cg.push_back(makeAssign(while_accum, makeVar(cg.newsym)));

      wt.push_back(makeAssign(while_test, makeAnd(makeVar(wt.newsym), makeNot(makeVar(while_accum)))));
      wt.push_back(makeAssign(while_accum, makeOr(makeVar(while_accum), makeVar(wt.newsym))));

      cg.push_back(makeWhile(makeVar(while_test), wt.expressions()));
      cg.newsym = while_accum;
    }
}

inline void RE_Compiler::processBoundedRep(RE * repeated, int lb, int ub, CodeGenState & cg) {
    ub -= lb;
    while(lb-- != 0) {
        process(repeated, cg);
    }
    if (ub > 0) {
         std::string oldsym = cg.newsym;
         process(repeated, cg);
         processBoundedRep(repeated, 0, ub - 1, cg);
         std::string altsym = cg.symgen("alt");
         cg.push_back(makeAssign(altsym, makeOr(makeVar(oldsym), makeVar(cg.newsym))));
         cg.newsym = altsym;
    }
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
