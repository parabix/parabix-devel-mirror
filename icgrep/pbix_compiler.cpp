/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "pbix_compiler.h"
#include "printer_pablos.h"

Pbix_Compiler::Pbix_Compiler(std::map<std::string, std::string> name_map)
{
    m_name_map = name_map;
    symgen = SymbolGenerator();
}

CodeGenState Pbix_Compiler::compile_subexpressions(const std::map<std::string, RE*>& re_map)
{
    CodeGenState cg_state;

    for (auto it =  re_map.rbegin(); it != re_map.rend(); ++it)
    {
        //This is specifically for the utf8 multibyte character classes.
        if (Seq* seq = dynamic_cast<Seq*>(it->second))
        {
            if (seq->getType() == Seq::Byte)
            {
                std::string gs_retVal = symgen.gensym("start_marker");
                cg_state.stmtsl.push_back(new Assign(gs_retVal, new All(1)));
                cg_state.newsym = gs_retVal;

                auto endit = seq->end();
                --endit;

                for (auto it = seq->begin(); it != seq->end(); ++it)
                {
                    Name* name = dynamic_cast<Name*>(*it);
                    if (it != endit)
                    {
                        gs_retVal = symgen.gensym("marker");
                        cg_state.stmtsl.push_back(new Assign(gs_retVal, new Advance(new And(new Var(cg_state.newsym), new CharClass(name->getName())))));
                        cg_state.newsym = gs_retVal;
                    }
                    else
                    {
                        cg_state.stmtsl.push_back(new Assign(seq->getName(), new And(new Var(cg_state.newsym), new CharClass(name->getName()))));
                    }
                }
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

    if (unicode_re(re))
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
#endif
#ifndef USE_IF_FOR_NONFINAL
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
    if (Name* name = dynamic_cast<Name*>(re))
    {
        std::string gs_retVal = symgen.gensym("marker");
        PabloE* markerExpr = new Var(cg_state.newsym);
        if (name->getType() != Name::FixedLength) {
            // Move the markers forward through any nonfinal UTF-8 bytes to the final position of each character.
            markerExpr = new And(markerExpr, new CharClass(m_name_map.find("internal.initial")->second));
            markerExpr = new ScanThru(markerExpr, new CharClass(m_name_map.find("internal.nonfinal")->second));
        }       
        PabloE* ccExpr;
        if (name->getType() == Name::UnicodeCategory)
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

        //std::cout << "\n" << "(" << StatementPrinter::PrintStmts(cg_state) << ")" << "\n" << std::endl;
    }
    else if (dynamic_cast<Start*>(re))
    {
        std::string gs_retVal = symgen.gensym("start_of_line_marker");
        cg_state.stmtsl.push_back(new Assign(gs_retVal, new And(new Var(cg_state.newsym), new Not(new Advance(new Not(new CharClass(m_name_map.find("LineFeed")->second)))))));
        cg_state.newsym = gs_retVal;
    }
    else if (dynamic_cast<End*>(re))
    {
        std::string gs_retVal = symgen.gensym("end_of_line_marker");
        cg_state.stmtsl.push_back(new Assign(gs_retVal, new And(new Var(cg_state.newsym), new CharClass(m_name_map.find("LineFeed")->second))));
        cg_state.newsym = gs_retVal;
    }
    else if (Seq* seq = dynamic_cast<Seq*>(re))
    {
        if (!seq->empty())
        {
            cg_state = Seq_helper(seq, seq->begin(), cg_state);
        }
    }
    else if (Alt* alt = dynamic_cast<Alt*>(re))
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
    else if (Rep* rep = dynamic_cast<Rep*>(re))
    {
        if ((dynamic_cast<Name*>(rep->getRE()) != 0) && (rep->getLB() == 0) && (rep->getUB()== UNBOUNDED_REP))
        {
            Name* rep_name = dynamic_cast<Name*>(rep->getRE());
            std::string gs_retVal = symgen.gensym("marker");

            PabloE* ccExpr;
            if (rep_name->getType() == Name::UnicodeCategory)
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
            if (rep_name->getType() == Name::FixedLength)
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
        else if (rep->getUB() == UNBOUNDED_REP)
        {
            cg_state = UnboundedRep_helper(rep->getRE(), rep->getLB(), cg_state);
        }
        else if (rep->getUB() != UNBOUNDED_REP)
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
         //std::cout << "While, no lb." << std::endl;

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


bool Pbix_Compiler::unicode_re(RE *re)
{
    bool found = false;

    return unicode_re_helper(re, found);
}

bool Pbix_Compiler::unicode_re_helper(RE *re, bool found)
{
    if (!found)
    {
        if (Name* name = dynamic_cast<Name*>(re))
        {
            if ((name->getType() == Name::UnicodeCategory) || (name->getType() == Name::Unicode))
            {
                found = true;
            }
        }
        else if (Seq* re_seq = dynamic_cast<Seq*>(re))
        {
            for (auto it = re_seq->begin(); it != re_seq->end(); ++it)
            {
                found = unicode_re_helper(*it, found);
                if (found) break;
            }
        }
        else if (Alt* re_alt = dynamic_cast<Alt*>(re))
        {
            for (auto it = re_alt->begin(); it != re_alt->end(); ++it)
            {
                found = unicode_re_helper(*it, found);
                if (found) break;
            }
        }
        else if (Rep* rep = dynamic_cast<Rep*>(re))
        {
            found = unicode_re_helper(rep->getRE(), found);
        }
    }

    return found;
}
