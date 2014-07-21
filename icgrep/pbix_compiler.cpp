/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "pbix_compiler.h"

Pbix_Compiler::Pbix_Compiler(){
  symgen = SymbolGenerator();
}

CodeGenState Pbix_Compiler::compile(RE *re)
{
    std::string gs_retVal;
    gs_retVal = symgen.gensym("start_marker");

    CodeGenState cg_state;
    cg_state.stmtsl.push_back(new Assign(gs_retVal, new All(1)));
    cg_state.newsym = gs_retVal;

    cg_state = re2pablo_helper(re, cg_state);

    //These three lines are specifically for grep.
    gs_retVal = symgen.gensym("marker");
    cg_state.stmtsl.push_back(new Assign(gs_retVal, new And(new MatchStar(new Var(cg_state.newsym), new Not(new Var("lex.cclf"))), new Var("lex.cclf"))));
    cg_state.newsym = gs_retVal;

    return cg_state;
}

CodeGenState Pbix_Compiler::re2pablo_helper(RE *re, CodeGenState cg_state)
{
    if (CC* cc = dynamic_cast<CC*>(re))
    {
        std::string gs_retVal = symgen.gensym("marker");
        cg_state.stmtsl.push_back(new Assign(gs_retVal, new Advance(new And(new Var(cg_state.newsym), new CharClass(cc->getName())))));
        cg_state.newsym = gs_retVal;

        //cout << "\n" << "(" << StatementPrinter::PrintStmts(cg_state) << ")" << "\n" << endl;
    }
    else if (Start* start = dynamic_cast<Start*>(re))
    {
        std::string gs_retVal = symgen.gensym("start_of_line_marker");
        cg_state.stmtsl.push_back(new Assign(gs_retVal, new And(new Var(cg_state.newsym), new Not(new Advance(new Not(new CharClass("lex.cclf")))))));
        cg_state.newsym = gs_retVal;
    }
    else if (End* end = dynamic_cast<End*>(re))
    {
        std::string gs_retVal = symgen.gensym("end_of_line_marker");
        cg_state.stmtsl.push_back(new Assign(gs_retVal, new And(new Var(cg_state.newsym), new CharClass("lex.cclf"))));
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
        if ((dynamic_cast<CC*>(rep->getRE()) != 0) && (rep->getLB() == 0) && (dynamic_cast<Unbounded*>(rep->getUB())!= 0))
        {
            //std::cout << "Matchstar!" << std::endl;
            CC* rep_cc = dynamic_cast<CC*>(rep->getRE());
            std::string gs_retVal = symgen.gensym("marker");
            cg_state.stmtsl.push_back(new Assign(gs_retVal, new MatchStar(new Var(cg_state.newsym), new CharClass(rep_cc->getName()))));
            cg_state.newsym = gs_retVal;
        }
        else if (dynamic_cast<Unbounded*>(rep->getUB()) != 0)
        {
            if (rep->getLB() == 0)
            {
                //std::cout << "While, no lb." << std::endl; //THIS IS THE ONE THAT ISN'T WORKING.

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
        else if (dynamic_cast<UpperBound*>(rep->getUB()) != 0)
        {
            UpperBound* ub = dynamic_cast<UpperBound*>(rep->getUB());
            if ((rep->getLB() == 0) && (ub->getUB() == 0))
            {
                //Just fall through...do nothing.
            }
            else if ((rep->getLB() == 0) && (ub->getUB() > 0))
            {
                CodeGenState t1_cg_state = re2pablo_helper(rep->getRE(), cg_state);
                ub->setUB(ub->getUB() - 1);
                CodeGenState t2_cg_state = re2pablo_helper(re, t1_cg_state);
                std::string gs_retVal = symgen.gensym("alt_marker");
                cg_state.stmtsl = t2_cg_state.stmtsl;
                cg_state.stmtsl.push_back(new Assign(gs_retVal, new Or(new Var(cg_state.newsym), new Var(t2_cg_state.newsym))));
                cg_state.newsym = gs_retVal;
            }
            else //if ((rep->getLB() > 0) && (ub->getUB() > 0))
            {
                CodeGenState t1_cg_state = re2pablo_helper(rep->getRE(), cg_state);
                rep->setLB(rep->getLB() - 1);
                ub->setUB(ub->getUB() - 1);
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

