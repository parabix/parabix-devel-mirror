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
    CodeGenState cg_state;

    std::string gs_m0 = symgen.gensym("start_marker");
    cg_state.stmtsl.push_back(new Assign(gs_m0, new All(1)));

    if (unicode_re(re))
    {
        cg_state.newsym = gs_m0;
        //Set the 'internal.initial' bit stream for the utf-8 multi-byte encoding.
        std::string gs_initial = symgen.gensym("internal.initial");
        m_name_map.insert(make_pair("internal.initial", gs_initial));
        cg_state.stmtsl.push_back(new Assign(gs_initial, new Or(new Or( new Or( new And(new Var(m_name_map.find("UTF8-Prefix2")->second),
            new Var(cg_state.newsym)),  new And(new Var(m_name_map.find("UTF8-SingleByte")->second), new Var(cg_state.newsym))),
            new And(new Var(m_name_map.find("UTF8-Prefix3")->second), new Var(cg_state.newsym))),
            new And(new Var(m_name_map.find("UTF8-Prefix4")->second), new Var(cg_state.newsym)))));
        cg_state.newsym = gs_initial;

        //Set the 'internal.nonfinal' bit stream for the utf-8 multi-byte encoding.
        cg_state.newsym = gs_m0;
        std::string gs_nonfinal = symgen.gensym("internal.nonfinal");
        m_name_map.insert(make_pair("internal.nonfinal", gs_nonfinal));
        cg_state.stmtsl.push_back(new Assign(gs_nonfinal, new Or(new Or(new Or(new Or(new Or( new And(new Var(m_name_map.find("UTF8-Prefix3")->second),
            new Var(cg_state.newsym)),  new And(new Var(m_name_map.find("UTF8-Prefix2")->second), new Var(cg_state.newsym))),
            new Advance( new And(new Var(m_name_map.find("UTF8-Prefix3")->second), new Var(cg_state.newsym)))),
            new And(new Var(m_name_map.find("UTF8-Prefix4")->second), new Var(cg_state.newsym))), new Advance(
            new And(new Var(m_name_map.find("UTF8-Prefix4")->second), new Var(cg_state.newsym)))), new Advance(
            new Advance( new And(new Var(m_name_map.find("UTF8-Prefix4")->second), new Var(cg_state.newsym)))))));
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
            else //Name::Unicode and Name::UnicodeCategory
            {
                cg_state.stmtsl.push_back(new Assign(gs_retVal,
                    new And(new MatchStar(new Var(cg_state.newsym), new Or(new CharClass(m_name_map.find("internal.nonfinal")->second),
                    new CharClass(rep_name->getName()))), new CharClass(m_name_map.find("internal.initial")->second))));
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
            std::list<RE*>::iterator it;
            for (it = re_seq->GetREList()->begin(); it != re_seq->GetREList()->end(); ++it)
            {
                found = unicode_re_helper(*it, found);
                if (found) break;
            }
        }
        else if (Alt* re_alt = dynamic_cast<Alt*>(re))
        {
            std::list<RE*>::iterator it;
            for (it = re_alt->GetREList()->begin(); it != re_alt->GetREList()->end(); ++it)
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
