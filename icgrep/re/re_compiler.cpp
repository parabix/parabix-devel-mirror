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
#include <pablo/ps_pablos.h>
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
#include <pablo/ps_pablos.h>
#include <pablo/ps_assign.h>
#include <pablo/ps_if.h>
#include <pablo/ps_while.h>

#include <assert.h>
#include <stdexcept>

using namespace pablo;

namespace re {

RE_Compiler::RE_Compiler(std::map<std::string, std::string> name_map)
: m_name_map(name_map)
, symgen()
{

}

CodeGenState RE_Compiler::compile_subexpressions(const std::map<std::string, RE*>& re_map)
{
    CodeGenState cg_state;
    for (auto i =  re_map.rbegin(); i != re_map.rend(); ++i) {
        //This is specifically for the utf8 multibyte character classes.
        if (Seq * seq = dyn_cast<Seq>(i->second)) {
            if (seq->getType() == Seq::Type::Byte) {
                std::string gs_retVal = symgen.get("start_marker");
                cg_state.stmtsl.push_back(make_assign(gs_retVal, make_all(1)));
                for (auto j = seq->begin();; ) {
                    Name * name = dyn_cast<Name>(*j);
                    assert (name);
                    auto * cc_mask = make_and(make_var(gs_retVal), make_charclass(name->getName()));
                    if (++j != seq->end()) {
                        gs_retVal = symgen.get("marker");
                        cg_state.stmtsl.push_back(make_assign(gs_retVal, make_advance(cc_mask)));
                    }
                    else {
                        cg_state.stmtsl.push_back(make_assign(seq->getName(), cc_mask));
                        break;
                    }
                }
                cg_state.newsym = gs_retVal;
            }
        }
    }
    return cg_state;
}

CodeGenState RE_Compiler::compile(RE * re)
{
    CodeGenState cg_state;

    std::string gs_m0 = symgen.get("start_marker");
    cg_state.stmtsl.push_back(make_assign(gs_m0, make_all(1)));

    if (hasUnicode(re)) {
        cg_state.newsym = gs_m0;
        //Set the 'internal.initial' bit stream for the utf-8 multi-byte encoding.
        std::string gs_initial = symgen.get("internal.initial");
        m_name_map.insert(make_pair("internal.initial", gs_initial));
        PabloE * u8single = make_var(m_name_map.find("UTF8-SingleByte")->second);
        PabloE * u8pfx2 = make_var(m_name_map.find("UTF8-Prefix2")->second);
        PabloE * u8pfx3 = make_var(m_name_map.find("UTF8-Prefix3")->second);
        PabloE * u8pfx4 = make_var(m_name_map.find("UTF8-Prefix4")->second);
        PabloE * u8pfx = make_or(make_or(u8pfx2, u8pfx3), u8pfx4);
        cg_state.stmtsl.push_back(make_assign(gs_initial, make_or(u8pfx, u8single)));
        cg_state.newsym = gs_initial;

        //Set the 'internal.nonfinal' bit stream for the utf-8 multi-byte encoding.
        cg_state.newsym = gs_m0;
        std::string gs_nonfinal = symgen.get("internal.nonfinal");
        m_name_map.insert(make_pair("internal.nonfinal", gs_nonfinal));
        //#define USE_IF_FOR_NONFINAL
        #ifdef USE_IF_FOR_NONFINAL
        cg_state.stmtsl.push_back(make_assign(gs_nonfinal, make_all(0)));
        #endif
        PabloE * u8scope32 = make_advance(u8pfx3);
        PabloE * u8scope42 = make_advance(u8pfx4);
        PabloE * u8scope43 = make_advance(u8scope42);
        PabloE * assign_non_final = make_assign(gs_nonfinal, make_or(make_or(u8pfx, u8scope32), make_or(u8scope42, u8scope43)));
        #ifdef USE_IF_FOR_NONFINAL
        std::list<PabloE *> * if_body = new std::list<PabloE *> ();
        if_body->push_back(assign_non_final);
        cg_state.stmtsl.push_back(new If(u8pfx, *if_body));
        #else
        cg_state.stmtsl.push_back(assign_non_final);
        #endif
        cg_state.newsym = gs_nonfinal;
    }

    cg_state.newsym = gs_m0;
    compile(re, cg_state);

    //These three lines are specifically for grep.
    std::string gs_retVal = symgen.get("marker");
    cg_state.stmtsl.push_back(make_assign(gs_retVal, make_and(new MatchStar(make_var(cg_state.newsym),
        make_not(make_var(m_name_map.find("LineFeed")->second))), make_var(m_name_map.find("LineFeed")->second))));
    cg_state.newsym = gs_retVal;

    return cg_state;
}

void RE_Compiler::compile(RE * re, CodeGenState & cg_state) {
    if (Name * name = dyn_cast<Name>(re)) {
        compile(name, cg_state);
    }
    else if (Seq* seq = dyn_cast<Seq>(re)) {
        compile(seq, cg_state);
    }
    else if (Alt * alt = dyn_cast<Alt>(re)) {
        compile(alt, cg_state);
    }
    else if (Rep * rep = dyn_cast<Rep>(re)) {
        compile(rep, cg_state);
    }
    else if (isa<Start>(re)) {
        std::string gs_retVal = symgen.get("sol");
        cg_state.stmtsl.push_back(make_assign(gs_retVal, make_and(make_var(cg_state.newsym), make_not(make_advance(make_not(make_charclass(m_name_map.find("LineFeed")->second)))))));
        cg_state.newsym = gs_retVal;
    }
    else if (isa<End>(re)) {
        std::string gs_retVal = symgen.get("eol");
        cg_state.stmtsl.push_back(make_assign(gs_retVal, make_and(make_var(cg_state.newsym), make_charclass(m_name_map.find("LineFeed")->second))));
        cg_state.newsym = gs_retVal;
    }
}

inline void RE_Compiler::compile(Name * name, CodeGenState & cg_state) {
    std::string gs_retVal = symgen.get("marker");
    PabloE * markerExpr = make_var(cg_state.newsym);
    if (name->getType() != Name::Type::FixedLength) {
        // Move the markers forward through any nonfinal UTF-8 bytes to the final position of each character.
        markerExpr = make_and(markerExpr, make_charclass(m_name_map.find("internal.initial")->second));
        markerExpr = new ScanThru(markerExpr, make_charclass(m_name_map.find("internal.nonfinal")->second));
    }
    PabloE * ccExpr;
    if (name->getType() == Name::Type::UnicodeCategory) {
        ccExpr = make_call(name->getName());
    }
    else {
        ccExpr = make_charclass(name->getName());
    }
    if (name->isNegated()) {
        ccExpr = make_not(make_or(make_or(ccExpr, make_charclass(m_name_map.find("LineFeed")->second)),
                                make_charclass(m_name_map.find("internal.nonfinal")->second)));
    }
    cg_state.stmtsl.push_back(make_assign(gs_retVal, make_advance(make_and(ccExpr, markerExpr))));
    cg_state.newsym = gs_retVal;
}

inline void RE_Compiler::compile(Seq * seq, CodeGenState & cg_state) {
    for (RE * re : *seq) {
        compile(re, cg_state);
    }
}

inline void RE_Compiler::compile(Alt * alt, CodeGenState & cg_state) {
    if (alt->empty()) {
        std::string gs_retVal = symgen.get("always_fail_marker");
        cg_state.stmtsl.push_back(make_assign(gs_retVal, make_all(0)));
        cg_state.newsym = gs_retVal;
    }
    else {
        auto i = alt->begin();
        const std::string startsym = cg_state.newsym;
        compile(*i, cg_state);
        while (++i != alt->end()) {
            std::string oldsym = cg_state.newsym;
            cg_state.newsym = startsym;
            compile(*i, cg_state);
            std::string altsym = symgen.get("alt");
            cg_state.stmtsl.push_back(make_assign(altsym, make_or(make_var(oldsym), make_var(cg_state.newsym))));
            cg_state.newsym = altsym;
        }
    }
}

inline void RE_Compiler::compile(Rep * rep, CodeGenState & cg_state) {
    if (isa<Name>(rep->getRE()) && (rep->getLB() == 0) && (rep->getUB()== Rep::UNBOUNDED_REP)) {
        Name * rep_name = dyn_cast<Name>(rep->getRE());
        std::string gs_retVal = symgen.get("marker");

        PabloE* ccExpr;
        if (rep_name->getType() == Name::Type::UnicodeCategory) {
            ccExpr = make_call(rep_name->getName());
        }
        else {
            ccExpr = make_charclass(rep_name->getName());
        }

        if (rep_name->isNegated()) {
            ccExpr = make_not(make_or(make_or(ccExpr, make_charclass(m_name_map.find("LineFeed")->second)), make_charclass(m_name_map.find("internal.nonfinal")->second)));
        }
        if (rep_name->getType() == Name::Type::FixedLength) {
            cg_state.stmtsl.push_back(make_assign(gs_retVal, new MatchStar(make_var(cg_state.newsym), ccExpr)));
        }
        else { // Name::Unicode and Name::UnicodeCategory
            cg_state.stmtsl.push_back(make_assign(gs_retVal,
                make_and(new MatchStar(make_var(cg_state.newsym),
                        make_or(make_charclass(m_name_map.find("internal.nonfinal")->second), ccExpr)),
                               make_charclass(m_name_map.find("internal.initial")->second))));
        }
        cg_state.newsym = gs_retVal;
    }
    else if (rep->getUB() == Rep::UNBOUNDED_REP) {
        compileUnboundedRep(rep->getRE(), rep->getLB(), cg_state);
    }
    else { // if (rep->getUB() != Rep::UNBOUNDED_REP)
        compileBoundedRep(rep->getRE(), rep->getLB(), rep->getUB(), cg_state);
    }
}

inline void RE_Compiler::compileUnboundedRep(RE * repeated, int lb, CodeGenState & cg_state) {
    for (; lb; --lb) {
        compile(repeated, cg_state);
    }
    std::string while_test = symgen.get("while_test");
    std::string while_accum = symgen.get("while_accum");
    CodeGenState while_test_state;
    while_test_state.newsym = while_test;
    compile(repeated, while_test_state);
    cg_state.stmtsl.push_back(make_assign(while_test, make_var(cg_state.newsym)));
    cg_state.stmtsl.push_back(make_assign(while_accum, make_var(cg_state.newsym)));
    while_test_state.stmtsl.push_back(make_assign(while_test, make_and(make_var(while_test_state.newsym), make_not(make_var(while_accum)))));
    while_test_state.stmtsl.push_back(make_assign(while_accum, make_or(make_var(while_accum), make_var(while_test_state.newsym))));
    cg_state.stmtsl.push_back(new While(make_var(while_test), while_test_state.stmtsl));
    cg_state.newsym = while_accum;
}

inline void RE_Compiler::compileBoundedRep(RE * repeated, int lb, int ub, CodeGenState & cg_state) {
    ub -= lb;
    for (; lb; --lb) {
        compile(repeated, cg_state);
    }
    if (ub > 0) {
         std::string oldsym = cg_state.newsym;
         compile(repeated, cg_state);
         compileBoundedRep(repeated, 0, ub - 1, cg_state);
         std::string altsym = symgen.get("alt");
         cg_state.stmtsl.push_back(make_assign(altsym, make_or(make_var(oldsym), make_var(cg_state.newsym))));
         cg_state.newsym = altsym;
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
