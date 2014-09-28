/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "pbix_compiler.h"
//Regular Expressions
#include "re/re_name.h"
#include "re/re_start.h"
#include "re/re_end.h"
#include "re/re_seq.h"
#include "re/re_alt.h"
#include "re/re_rep.h"

//Pablo Expressions
#include "pe_pabloe.h"
#include "pe_sel.h"
#include "pe_advance.h"
#include "pe_all.h"
#include "pe_and.h"
#include "pe_charclass.h"
#include "pe_call.h"
#include "pe_matchstar.h"
#include "pe_scanthru.h"
#include "pe_not.h"
#include "pe_or.h"
#include "pe_var.h"
#include "pe_xor.h"

//Pablo Statements
#include "ps_pablos.h"
#include "ps_assign.h"
#include "ps_if.h"
#include "ps_while.h"

#include <assert.h>
#include <stdexcept>

using namespace re;

Pbix_Compiler::Pbix_Compiler(std::map<std::string, std::string> name_map)
{
    m_name_map = name_map;
    symgen = SymbolGenerator();
}

CodeGenState Pbix_Compiler::compile_subexpressions(const std::map<std::string, RE*>& re_map)
{
    CodeGenState cg_state;
    for (auto i =  re_map.rbegin(); i != re_map.rend(); ++i) {
        //This is specifically for the utf8 multibyte character classes.
        if (Seq * seq = dyn_cast<Seq>(i->second)) {
            if (seq->getType() == Seq::Type::Byte) {
                std::string gs_retVal = symgen.gensym("start_marker");
                cg_state.stmtsl.push_back(new Assign(gs_retVal, new All(1)));                
                for (auto j = seq->begin();; ) {
                    Name * name = dyn_cast<Name>(*j);
                    assert (name);
                    And * cc_mask = new And(new Var(gs_retVal), new CharClass(name->getName()));
                    if (++j != seq->end()) {
                        gs_retVal = symgen.gensym("marker");
                        cg_state.stmtsl.push_back(new Assign(gs_retVal, new Advance(cc_mask)));
                    }
                    else {
                        cg_state.stmtsl.push_back(new Assign(seq->getName(), cc_mask));
                        break;
                    }
                }
                cg_state.newsym = gs_retVal;
            }
        }
    }
    return cg_state;
}

CodeGenState Pbix_Compiler::compile(RE *re)
{   
    CodeGenState cg_state;

    std::string gs_m0 = symgen.gensym("start_marker");
    cg_state.stmtsl.push_back(new Assign(gs_m0, new All(1)));

    if (hasUnicode(re))
    {
        cg_state.newsym = gs_m0;
        //Set the 'internal.initial' bit stream for the utf-8 multi-byte encoding.
        std::string gs_initial = symgen.gensym("internal.initial");
        m_name_map.insert(make_pair("internal.initial", gs_initial));
        PabloE * u8single = new Var(m_name_map.find("UTF8-SingleByte")->second);
        PabloE * u8pfx2 = new Var(m_name_map.find("UTF8-Prefix2")->second);
        PabloE * u8pfx3 = new Var(m_name_map.find("UTF8-Prefix3")->second);
        PabloE * u8pfx4 = new Var(m_name_map.find("UTF8-Prefix4")->second);
        PabloE * u8pfx = new Or(new Or(u8pfx2, u8pfx3), u8pfx4);
        cg_state.stmtsl.push_back(new Assign(gs_initial, new Or(u8pfx, u8single)));
        cg_state.newsym = gs_initial;

        //Set the 'internal.nonfinal' bit stream for the utf-8 multi-byte encoding.
        cg_state.newsym = gs_m0;
        std::string gs_nonfinal = symgen.gensym("internal.nonfinal");
        m_name_map.insert(make_pair("internal.nonfinal", gs_nonfinal));
        //#define USE_IF_FOR_NONFINAL
        #ifdef USE_IF_FOR_NONFINAL
        cg_state.stmtsl.push_back(new Assign(gs_nonfinal, new All(0)));
        #endif
        PabloE * u8scope32 = new Advance(u8pfx3);
        PabloE * u8scope42 = new Advance(u8pfx4);
        PabloE * u8scope43 = new Advance(u8scope42);
        PabloS * assign_non_final = new Assign(gs_nonfinal, new Or(new Or(u8pfx, u8scope32), new Or(u8scope42, u8scope43)));
        #ifdef USE_IF_FOR_NONFINAL
        std::list<PabloS *> * if_body = new std::list<PabloS *> ();
        if_body->push_back(assign_non_final);
        cg_state.stmtsl.push_back(new If(u8pfx, *if_body));
        #else
        cg_state.stmtsl.push_back(assign_non_final);
        #endif
        cg_state.newsym = gs_nonfinal;
    }

    cg_state.newsym = gs_m0;
    cg_state = re2pablo_helper(re, cg_state);

    //These three lines are specifically for grep.
    std::string gs_retVal = symgen.gensym("marker");
    cg_state.stmtsl.push_back(new Assign(gs_retVal, new And(new MatchStar(new Var(cg_state.newsym),
        new Not(new Var(m_name_map.find("LineFeed")->second))), new Var(m_name_map.find("LineFeed")->second))));
    cg_state.newsym = gs_retVal;

    return cg_state;
}

CodeGenState Pbix_Compiler::re2pablo_helper(RE *re, CodeGenState cg_state)
{
    if (Name* name = dyn_cast<Name>(re))
    {
        std::string gs_retVal = symgen.gensym("marker");
        PabloE* markerExpr = new Var(cg_state.newsym);
        if (name->getType() != Name::Type::FixedLength) {
            // Move the markers forward through any nonfinal UTF-8 bytes to the final position of each character.
            markerExpr = new And(markerExpr, new CharClass(m_name_map.find("internal.initial")->second));
            markerExpr = new ScanThru(markerExpr, new CharClass(m_name_map.find("internal.nonfinal")->second));
        }       
        PabloE* ccExpr;
        if (name->getType() == Name::Type::UnicodeCategory)
        {
            ccExpr = new Call(name->getName());
        }
        else 
        {
            ccExpr = new CharClass(name->getName());
        }
        if (name->isNegated()) {
            ccExpr = new Not(new Or(new Or(ccExpr, new CharClass(m_name_map.find("LineFeed")->second)),
                                    new CharClass(m_name_map.find("internal.nonfinal")->second)));
        }
        cg_state.stmtsl.push_back(new Assign(gs_retVal, new Advance(new And(ccExpr, markerExpr))));
        cg_state.newsym = gs_retVal;
    }
    else if (isa<Start>(re))
    {
        std::string gs_retVal = symgen.gensym("start_of_line_marker");
        cg_state.stmtsl.push_back(new Assign(gs_retVal, new And(new Var(cg_state.newsym), new Not(new Advance(new Not(new CharClass(m_name_map.find("LineFeed")->second)))))));
        cg_state.newsym = gs_retVal;
    }
    else if (isa<End>(re))
    {
        std::string gs_retVal = symgen.gensym("end_of_line_marker");
        cg_state.stmtsl.push_back(new Assign(gs_retVal, new And(new Var(cg_state.newsym), new CharClass(m_name_map.find("LineFeed")->second))));
        cg_state.newsym = gs_retVal;
    }
    else if (Seq* seq = dyn_cast<Seq>(re))
    {
        if (!seq->empty())
        {
            cg_state = Seq_helper(seq, seq->begin(), cg_state);
        }
    }
    else if (Alt* alt = dyn_cast<Alt>(re))
    {
        if (alt->empty())
        {
            std::string gs_retVal = symgen.gensym("always_fail_marker");
            cg_state.stmtsl.push_back(new Assign(gs_retVal, new All(0)));
            cg_state.newsym = gs_retVal;
        }
        else
        {
            if (alt->size() == 1)
            {
                cg_state = re2pablo_helper(alt->back(), cg_state);
            }
            else
            {
                cg_state = Alt_helper(alt, alt->begin(), cg_state);
            }
        }

    }
    else if (Rep* rep = dyn_cast<Rep>(re))
    {
        if (isa<Name>(rep->getRE()) && (rep->getLB() == 0) && (rep->getUB()== Rep::UNBOUNDED_REP))
        {
            Name* rep_name = dyn_cast<Name>(rep->getRE());
            std::string gs_retVal = symgen.gensym("marker");

            PabloE* ccExpr;
            if (rep_name->getType() == Name::Type::UnicodeCategory)
            {
                ccExpr = new Call(rep_name->getName());
            }
            else 
            {
                ccExpr = new CharClass(rep_name->getName());
            }

            if (rep_name->isNegated()) {
                ccExpr = new Not(new Or(new Or(ccExpr, new CharClass(m_name_map.find("LineFeed")->second)),
                                        new CharClass(m_name_map.find("internal.nonfinal")->second)));
            }
            if (rep_name->getType() == Name::Type::FixedLength)
            {
                cg_state.stmtsl.push_back(new Assign(gs_retVal, new MatchStar(new Var(cg_state.newsym), ccExpr)));
            }
            else //Name::Unicode and Name::UnicodeCategory
            {
                cg_state.stmtsl.push_back(new Assign(gs_retVal,
                    new And(new MatchStar(new Var(cg_state.newsym), new Or(new CharClass(m_name_map.find("internal.nonfinal")->second),
                    ccExpr)), new CharClass(m_name_map.find("internal.initial")->second))));
            }

            cg_state.newsym = gs_retVal;
        }
        else if (rep->getUB() == Rep::UNBOUNDED_REP)
        {
            cg_state = UnboundedRep_helper(rep->getRE(), rep->getLB(), cg_state);
        }
        else if (rep->getUB() != Rep::UNBOUNDED_REP)
        {
            cg_state = BoundedRep_helper(rep->getRE(), rep->getLB(), rep->getUB(), cg_state);
        }
    }

    return cg_state;
}


CodeGenState Pbix_Compiler::Seq_helper(Vector *lst, const_iterator it, CodeGenState cg_state)
{
    if (it != lst->end())
    {
        cg_state = re2pablo_helper(*it, cg_state);
        cg_state = Seq_helper(lst, ++it, cg_state);
    }

    return cg_state;
}

CodeGenState Pbix_Compiler::Alt_helper(Vector* lst, const_iterator it, CodeGenState cg_state)
{
    CodeGenState t1_cg_state = re2pablo_helper(*it, cg_state);
    cg_state.stmtsl = t1_cg_state.stmtsl;
    ++it;
    if (it != lst->end())
    {
        CodeGenState t2_cg_state = Alt_helper(lst, it, cg_state);
        cg_state.stmtsl = t2_cg_state.stmtsl;
        std::string gs_retVal = symgen.gensym("alt_marker");
        cg_state.stmtsl.push_back(new Assign(gs_retVal, new Or(new Var(t1_cg_state.newsym), new Var(t2_cg_state.newsym))));
        cg_state.newsym = gs_retVal;
    }
    else
    {
        cg_state.newsym = t1_cg_state.newsym;
    }

    return cg_state;
}

CodeGenState Pbix_Compiler::UnboundedRep_helper(RE* repeated, int lb, CodeGenState cg_state) {
    if (lb == 0)
    {
         std::string while_test_gs_retVal = symgen.gensym("while_test");
         std::string while_accum_gs_retVal = symgen.gensym("while_accum");
         CodeGenState while_test_state;
         while_test_state.newsym = while_test_gs_retVal;
         CodeGenState t1_cg_state = re2pablo_helper(repeated, while_test_state);
         cg_state.stmtsl.push_back(new Assign(while_test_gs_retVal, new Var(cg_state.newsym)));
         cg_state.stmtsl.push_back(new Assign(while_accum_gs_retVal, new Var(cg_state.newsym)));
         std::list<PabloS*> stmtList;
         stmtList = t1_cg_state.stmtsl;
         stmtList.push_back(new Assign(while_test_gs_retVal, new And(new Var(t1_cg_state.newsym), new Not(new Var(while_accum_gs_retVal)))));
         stmtList.push_back(new Assign(while_accum_gs_retVal, new Or(new Var(while_accum_gs_retVal), new Var(t1_cg_state.newsym))));
         cg_state.stmtsl.push_back( new While(new Var(while_test_gs_retVal), stmtList));
         cg_state.newsym = while_accum_gs_retVal;
    }
    else //if (lb > 0)
    {
         CodeGenState t1_cg_state = re2pablo_helper(repeated, cg_state);
         cg_state = UnboundedRep_helper(repeated, lb -1, t1_cg_state);
    }
    return cg_state;
}


CodeGenState Pbix_Compiler::BoundedRep_helper(RE* repeated, int lb, int ub, CodeGenState cg_state) {
    if ((lb == 0) && (ub == 0))
    {
    //Just fall through...do nothing.
    }
    else if ((lb == 0) && (ub > 0))
    {
         CodeGenState t1_cg_state = re2pablo_helper(repeated, cg_state);
         CodeGenState t2_cg_state = BoundedRep_helper(repeated, 0, ub-1, t1_cg_state);
         std::string gs_retVal = symgen.gensym("alt_marker");
         cg_state.stmtsl = t2_cg_state.stmtsl;
         cg_state.stmtsl.push_back(new Assign(gs_retVal, new Or(new Var(cg_state.newsym), new Var(t2_cg_state.newsym))));
         cg_state.newsym = gs_retVal;
    }
    else //if ((lb > 0) && (ub > 0))
    {
         CodeGenState t1_cg_state = re2pablo_helper(repeated, cg_state);
         cg_state = BoundedRep_helper(repeated, lb-1, ub-1, t1_cg_state);
    }
    return cg_state;
}


bool Pbix_Compiler::hasUnicode(const RE * re) {
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
