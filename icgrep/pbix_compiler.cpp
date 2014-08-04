/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "pbix_compiler.h"

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

                std::list<RE*>::iterator endit;
                endit = seq->GetREList()->end();
                --endit;
                std::list<RE*>::iterator it;
                for (it = seq->GetREList()->begin(); it != seq->GetREList()->end(); ++it)
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
    std::string gs_retVal;
    CodeGenState cg_state;

    //Set the 'internal.initial' bit stream for the utf-8 multi-byte encoding.
    gs_retVal = symgen.gensym("start_marker");
    cg_state.stmtsl.push_back(new Assign(gs_retVal, new All(1)));
    cg_state.newsym = gs_retVal;

    std::string gs_retVal_m1 = symgen.gensym("marker");
    cg_state.stmtsl.push_back(new Assign(gs_retVal_m1, new And(new Var(m_name_map.find("UTF8-SingleByte")->second), new Var(cg_state.newsym))));

    std::string gs_retVal_m2 = symgen.gensym("marker");
    cg_state.stmtsl.push_back(new Assign(gs_retVal_m2, new And(new Var(m_name_map.find("UTF8-Prefix2")->second), new Var(cg_state.newsym))));

    std::string gs_retVal_m3 = symgen.gensym("marker");
    cg_state.stmtsl.push_back(new Assign(gs_retVal_m3, new And(new Var(m_name_map.find("UTF8-Prefix3")->second), new Var(cg_state.newsym))));

    std::string gs_retVal_m4 = symgen.gensym("marker");
    cg_state.stmtsl.push_back(new Assign(gs_retVal_m4, new And(new Var(m_name_map.find("UTF8-Prefix4")->second), new Var(cg_state.newsym))));

    std::string gs_retVal_m5 = symgen.gensym("marker");
    cg_state.stmtsl.push_back(new Assign(gs_retVal_m5, new Or(new Var(gs_retVal_m2), new Var(gs_retVal_m1))));

    std::string gs_retVal_m6 = symgen.gensym("marker");
    cg_state.stmtsl.push_back(new Assign(gs_retVal_m6, new Or(new Var(gs_retVal_m5), new Var(gs_retVal_m3))));

    gs_retVal = symgen.gensym("internal.initial");
    m_name_map.insert(make_pair("internal.initial", gs_retVal));
    cg_state.stmtsl.push_back(new Assign(gs_retVal, new Or(new Var(gs_retVal_m6), new Var(gs_retVal_m4))));
    cg_state.newsym = gs_retVal;

    //Set the 'internal.nonfinal' bit stream for the utf-8 multi-byte encoding.
    gs_retVal = symgen.gensym("start_marker");
    cg_state.stmtsl.push_back(new Assign(gs_retVal, new All(1)));
    cg_state.newsym = gs_retVal;

    gs_retVal_m1 = symgen.gensym("marker");
    cg_state.stmtsl.push_back(new Assign(gs_retVal_m1, new And(new Var(m_name_map.find("UTF8-Prefix2")->second), new Var(cg_state.newsym))));

    gs_retVal_m2 = symgen.gensym("marker");
    cg_state.stmtsl.push_back(new Assign(gs_retVal_m2, new And(new Var(m_name_map.find("UTF8-Prefix3")->second), new Var(cg_state.newsym))));

    gs_retVal_m3 = symgen.gensym("marker");
    cg_state.stmtsl.push_back(new Assign(gs_retVal_m3, new Advance(new Var(gs_retVal_m2))));

    gs_retVal_m4 = symgen.gensym("marker");
    cg_state.stmtsl.push_back(new Assign(gs_retVal_m4, new And(new Var(m_name_map.find("UTF8-Prefix4")->second), new Var(cg_state.newsym))));

    gs_retVal_m5 = symgen.gensym("marker");
    cg_state.stmtsl.push_back(new Assign(gs_retVal_m5, new Advance(new Var(gs_retVal_m4))));

    gs_retVal_m6 = symgen.gensym("marker");
    cg_state.stmtsl.push_back(new Assign(gs_retVal_m6, new Advance(new Var(gs_retVal_m5))));

    std::string gs_retVal_m7 = symgen.gensym("marker");
    cg_state.stmtsl.push_back(new Assign(gs_retVal_m7, new Or(new Var(gs_retVal_m2), new Var(gs_retVal_m1))));

    std::string gs_retVal_m8 = symgen.gensym("marker");
    cg_state.stmtsl.push_back(new Assign(gs_retVal_m8, new Or(new Var(gs_retVal_m7), new Var(gs_retVal_m3))));

    std::string gs_retVal_m9 = symgen.gensym("marker");
    cg_state.stmtsl.push_back(new Assign(gs_retVal_m9, new Or(new Var(gs_retVal_m8), new Var(gs_retVal_m4))));

    std::string gs_retVal_m10 = symgen.gensym("marker");
    cg_state.stmtsl.push_back(new Assign(gs_retVal_m10, new Or(new Var(gs_retVal_m9), new Var(gs_retVal_m5))));

    gs_retVal = symgen.gensym("internal.nonfinal");
    m_name_map.insert(make_pair("internal.nonfinal", gs_retVal));
    cg_state.stmtsl.push_back(new Assign(gs_retVal, new Or(new Var(gs_retVal_m10), new Var(gs_retVal_m6))));
    cg_state.newsym = gs_retVal;


    gs_retVal = symgen.gensym("start_marker");
    cg_state.stmtsl.push_back(new Assign(gs_retVal, new All(1)));
    cg_state.newsym = gs_retVal;
    cg_state = re2pablo_helper(re, cg_state);

    //These three lines are specifically for grep.
    gs_retVal = symgen.gensym("marker");
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

        if (name->getType() == Name::FixedLength)
        {
            cg_state.stmtsl.push_back(new Assign(gs_retVal, new Advance(new And(new Var(cg_state.newsym), new CharClass(name->getName())))));
        }
        else if (name->getType() == Name::UnicodeCategory)
        {
            cg_state.stmtsl.push_back(new Assign(gs_retVal, new Advance(new And(new Var(cg_state.newsym), new Call(name->getName())))));
        }
        else //Name::Unicode
        {
            cg_state.stmtsl.push_back(new Assign(gs_retVal, new Advance(new And(new CharClass(name->getName()), new ScanThru(new Var(cg_state.newsym), new CharClass(m_name_map.find("internal.nonfinal")->second))))));
        }
        cg_state.newsym = gs_retVal;

        //cout << "\n" << "(" << StatementPrinter::PrintStmts(cg_state) << ")" << "\n" << endl;
    }
    else if (Start* start = dynamic_cast<Start*>(re))
    {
        std::string gs_retVal = symgen.gensym("start_of_line_marker");
        cg_state.stmtsl.push_back(new Assign(gs_retVal, new And(new Var(cg_state.newsym), new Not(new Advance(new Not(new CharClass(m_name_map.find("LineFeed")->second)))))));
        cg_state.newsym = gs_retVal;
    }
    else if (End* end = dynamic_cast<End*>(re))
    {
        std::string gs_retVal = symgen.gensym("end_of_line_marker");
        cg_state.stmtsl.push_back(new Assign(gs_retVal, new And(new Var(cg_state.newsym), new CharClass(m_name_map.find("LineFeed")->second))));
        cg_state.newsym = gs_retVal;
    }
    else if (Seq* seq = dynamic_cast<Seq*>(re))
    {
        std::list<RE*>::iterator it = seq->GetREList()->begin();
        if (it != seq->GetREList()->end())
        {
            cg_state = Seq_helper(seq->GetREList(), it, cg_state);
        }
    //cout << "\n" << "Seq => (" << StatementPrinter::PrintStmts(cg_state) << ")" << "\n" << endl;
    }
    else if (Alt* alt = dynamic_cast<Alt*>(re))
    {
        if (alt->GetREList() == 0)
        {

            std::string gs_retVal = symgen.gensym("always_fail_marker");
            cg_state.stmtsl.push_back(new Assign(gs_retVal, new All(0)));
            cg_state.newsym = gs_retVal;
        }
        else
        {
            if (alt->GetREList()->size() == 1)
            {
                cg_state = re2pablo_helper(alt->GetREList()->front(), cg_state);
            }
            else
            {
                std::list<RE*>::iterator it = alt->GetREList()->begin();
                cg_state = Alt_helper(alt->GetREList(), it, cg_state);
            }
        }
    //cout << "\n" << "Alt => (" << StatementPrinter::PrintStmts(cg_state) << ")" << "\n" << endl;
    }
    else if (Rep* rep = dynamic_cast<Rep*>(re))
    {
        if ((dynamic_cast<Name*>(rep->getRE()) != 0) && (rep->getLB() == 0) && (rep->getUB()== unboundedRep))
        {
            Name* rep_name = dynamic_cast<Name*>(rep->getRE());
            std::string gs_retVal = symgen.gensym("marker");

            if (rep_name->getType() == Name::FixedLength)
            {
                cg_state.stmtsl.push_back(new Assign(gs_retVal, new MatchStar(new Var(cg_state.newsym), new CharClass(rep_name->getName()))));
            }
            else if (rep_name->getType() == Name::UnicodeCategory)
            {
                // TODO:  ?? not too sure....
            }
            else //Name::unicode
            {
                std::string t_retVal = symgen.gensym("t");
                std::string u_retVal = symgen.gensym("u");
                std::string v_retVal = symgen.gensym("v");
                std::string new_cur_retVal = symgen.gensym("new_cur");

                cg_state.stmtsl.push_back(new Assign(t_retVal, new Or(new CharClass(m_name_map.find("internal.nonfinal")->second), new CharClass(rep_name->getName()))));
                cg_state.stmtsl.push_back(new Assign(u_retVal, new MatchStar(new Var(cg_state.newsym), new Var(t_retVal))));
                cg_state.stmtsl.push_back(new Assign(v_retVal, new And(new Var(u_retVal), new CharClass(m_name_map.find("internal.initial")->second))));
                cg_state.stmtsl.push_back(new Assign(new_cur_retVal, new And(new Var(u_retVal), new Not(new Var(t_retVal)))));

                cg_state.stmtsl.push_back(new Assign(gs_retVal, new Or(new Var(v_retVal), new Var(new_cur_retVal))));
            }

            cg_state.newsym = gs_retVal;
        }
        else if (rep->getUB() == unboundedRep)
        {
            if (rep->getLB() == 0)
            {
                //std::cout << "While, no lb." << std::endl;

                std::string while_test_gs_retVal = symgen.gensym("while_test");
                std::string while_accum_gs_retVal = symgen.gensym("while_accum");
                CodeGenState while_test_state;
                while_test_state.newsym = while_test_gs_retVal;
                CodeGenState t1_cg_state = re2pablo_helper(rep->getRE(), while_test_state);
                cg_state.stmtsl.push_back(new Assign(while_test_gs_retVal, new Var(cg_state.newsym)));
                cg_state.stmtsl.push_back(new Assign(while_accum_gs_retVal, new Var(cg_state.newsym)));
                std::list<PabloS*> stmtList;
                stmtList = t1_cg_state.stmtsl;
                stmtList.push_back(new Assign(while_test_gs_retVal, new And(new Var(t1_cg_state.newsym), new Not(new Var(while_accum_gs_retVal)))));
                stmtList.push_back(new Assign(while_accum_gs_retVal, new Or(new Var(while_accum_gs_retVal), new Var(t1_cg_state.newsym))));
                cg_state.stmtsl.push_back( new While(new Var(while_test_gs_retVal), stmtList));
                cg_state.newsym = while_accum_gs_retVal;
            }
            else //if (rep->getLB() > 1)
            {
                CodeGenState t1_cg_state = re2pablo_helper(rep->getRE(), cg_state);
                rep->setLB(rep->getLB() - 1);
                cg_state = re2pablo_helper(rep, t1_cg_state);
            }
        }
        else if (rep->getUB() != unboundedRep)
        {
            if ((rep->getLB() == 0) && (rep->getUB() == 0))
            {
                //Just fall through...do nothing.
            }
            else if ((rep->getLB() == 0) && (rep->getUB() > 0))
            {
                CodeGenState t1_cg_state = re2pablo_helper(rep->getRE(), cg_state);
                rep->setUB(rep->getUB() - 1);
                CodeGenState t2_cg_state = re2pablo_helper(re, t1_cg_state);
                std::string gs_retVal = symgen.gensym("alt_marker");
                cg_state.stmtsl = t2_cg_state.stmtsl;
                cg_state.stmtsl.push_back(new Assign(gs_retVal, new Or(new Var(cg_state.newsym), new Var(t2_cg_state.newsym))));
                cg_state.newsym = gs_retVal;
            }
            else //if ((rep->getLB() > 0) && (rep->getUB() > 0))
            {
                CodeGenState t1_cg_state = re2pablo_helper(rep->getRE(), cg_state);
                rep->setLB(rep->getLB() - 1);
                rep->setUB(rep->getUB() - 1);
                cg_state = re2pablo_helper(rep, t1_cg_state);
            }
        }
    }

    return cg_state;
}

CodeGenState Pbix_Compiler::Seq_helper(std::list<RE*>* lst, std::list<RE*>::const_iterator it, CodeGenState cg_state)
{
    if (it != lst->end())
    {
        cg_state = re2pablo_helper(*it, cg_state);
        cg_state = Seq_helper(lst, ++it, cg_state);
    }

    return cg_state;
}

CodeGenState Pbix_Compiler::Alt_helper(std::list<RE*>* lst, std::list<RE*>::const_iterator it, CodeGenState cg_state)
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

