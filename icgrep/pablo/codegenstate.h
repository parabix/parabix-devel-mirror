/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PS_PABLOS_H
#define PS_PABLOS_H

#include <pablo/pe_pabloe.h>
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
#include <unordered_map>
#include <vector>
#include <string>
#include <pablo/symbol_generator.h>

namespace pablo {

struct CodeGenState {

    CodeGenState(SymbolGenerator & symgen)
    : mSymbolGenerator(symgen)
    , mPredecessor(nullptr)
    {

    }

    CodeGenState(CodeGenState & cg)
    : mSymbolGenerator(cg.mSymbolGenerator)
    , mPredecessor(&cg)
    {

    }


//    PabloE * createAll(const bool value);
//    PabloE * createAdvance(PabloE * expr);
//    PabloE * createNot(PabloE * expr);
//    PabloE * createCall(const std::string name);
//    PabloE * createVar(const std::string name);
//    PabloE * createCharClass(const std::string name);
//    PabloE * createAssign(const std::string name, PabloE * expr);
//    PabloE * createAnd(PabloE * expr1, PabloE * expr2);
//    PabloE * createOr(PabloE * expr1, PabloE * expr2);
//    PabloE * createXor(PabloE * expr1, PabloE * expr2);
//    PabloE * createMatchStar(PabloE * expr1, PabloE * expr2);
//    PabloE * createScanThru(PabloE * from, PabloE * thru);



//    struct ExpressionTable {
//        inline PabloE * find(PabloE * inst) {
//            switch(inst->getClassTypeId()) {
//                // UNARY
//                case All:       // bool
//                case Advance:   // pe
//                case Not:       // pe

//                case Call:      // string
//                case Var:       // string
//                case CharClass: // string
//                // BINARY
//                case Assign:    // string, pe

//                case And:       // pe, pe
//                case Or:        // pe, pe
//                case Xor:       // pe, pe
//                case MatchStar: // pe, pe
//                case ScanThru:  // pe, pe

//                case If:        // pe, pe list
//                case While:     // pe, pe list
//                // TERNARY
//                case Sel:       // pe, pe, pe
//            }


//            switch (inst.getNumOperands()) {
//                case 1: return _unary.find(&inst, inst.getType(), inst.getOperand(0));
//                case 2: return _binary.find(&inst, inst.getOperand(0), inst.getOperand(1));
//                case 3: return _ternary.find(&inst, inst.getOperand(0), inst.getOperand(1), inst.getOperand(2));
//                default: return nullptr;
//            }
//        }
//    private:
//        template<typename... Args>
//        struct ExpressionMap {
//            typedef std::tuple<PabloE::ClassTypeId, Args...> Key;
//            inline Instruction * find(Instruction * inst, Args... args) {
//                auto key = std::make_tuple(inst->getOpcode(), args...);
//                auto itr = _map.find(key);
//                if (itr == _map.end()) {
//                    _map.insert(std::make_pair(std::move(key), inst));
//                    return nullptr;
//                }
//                return itr->second;
//            }
//        private:
//            std::unordered_map<Key, PabloE *>                           _map;
//        };
//        ExpressionMap<const PabloE *>                                   _unary;
//        ExpressionMap<const PabloE *, const PabloE *>                   _binary;
//        ExpressionMap<const PabloE *, const PabloE *, const PabloE *>   _ternary;
//    };


    inline void push_back(PabloE * expr) {
        mExpressions.push_back(expr);
    }

    inline std::string symgen(std::string prefix) {
        return mSymbolGenerator.get(prefix);
    }

    inline const std::list<PabloE *> & expressions() const {
        return mExpressions;
    }

    std::string newsym;
private:
    std::list<PabloE *>     mExpressions;
    SymbolGenerator &       mSymbolGenerator;
    CodeGenState * const    mPredecessor;
};

}

#endif // PS_PABLOS_H
