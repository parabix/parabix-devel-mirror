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

namespace pablo {

struct CodeGenState{

//    struct ExpressionTable {
//        inline PabloE * find(PabloE * inst) {
//            switch(inst->getClassTypeId()) {
//                // UNARY
//                case All:
//                case Advance:
//                case Call:
//                case Not:
//                case Var:
//                case Assign:
//                case CharClass: // ***
//                // BINARY
//                case And:
//                case Or:
//                case Xor:

//                case MatchStar:
//                case ScanThru:
//                case If:
//                case While:
//                // TERNARY
//                case Sel:
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
//            std::map<Key, PabloE *>                                     _map;
//        };
//        ExpressionMap<const PabloE *>                                   _unary;
//        ExpressionMap<const PabloE *, const PabloE *>                   _binary;
//        ExpressionMap<const PabloE *, const PabloE *, const PabloE *>   _ternary;
//    };

    std::list<PabloE *>  stmtsl;
    std::string          newsym;
};

}

#endif // PS_PABLOS_H
