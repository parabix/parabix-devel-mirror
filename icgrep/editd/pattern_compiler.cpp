/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "pattern_compiler.h"
//Regular Expressions
#include <re/re_name.h>
#include <re/re_any.h>
#include <re/re_start.h>
#include <re/re_end.h>
#include <re/re_alt.h>
#include <re/re_cc.h>
#include <re/re_seq.h>
#include <re/re_rep.h>
#include <re/re_diff.h>
#include <re/re_intersect.h>
#include <re/re_assertion.h>
#include <re/re_analysis.h>
#include <pablo/codegenstate.h>
#include <pablo/function.h>

#include <assert.h>
#include <stdexcept>
#include <iostream>

#include "llvm/Support/CommandLine.h"

using namespace pablo;

namespace re {


Pattern_Compiler::Pattern_Compiler(pablo::PabloFunction & function)
: mFunction(function)
{

}


inline std::string make_e(int i, int j) {return ("e_"+std::to_string(i)+"_"+std::to_string(j));}


std::vector<Assign *> optimizer(std::string patt, std::vector<pablo::Var *> basisBits, std::vector<std::vector<Assign*>>  & e, int i, PabloAST * cond, PabloBuilder & pb, int dist, int stepSize){
    PabloBuilder it = PabloBuilder::Create(pb);
    std::vector<Assign *> var_list;
    for(int n = 0; n < stepSize; n++){
        // PabloAST * name = cast<Name>(seq->at(i))->getCompiled();
        int pi = (patt[i] >> 1) & 0x3;
        PabloAST * name = basisBits[pi];        
        
        e[i][0] = it.createAssign(make_e(i,0), it.createAnd(it.createAdvance(e[i-1][0],1), name));
        for(int j = 1; j <= dist; j++){  
            auto tmp1 = it.createAssign("tmp1", it.createAnd(it.createAdvance(e[i-1][j],1), name));
            auto tmp2 = it.createAssign("tmp2", it.createAnd(it.createAdvance(e[i-1][j-1],1), it.createNot(name)));
            auto tmp3 = it.createAssign("tmp3", it.createOr(it.createAdvance(e[i][j-1],1), e[i-1][j-1]));
            e[i][j] = it.createAssign(make_e(i,j), it.createOr(it.createOr(tmp1,tmp2),tmp3));
        }
        
        i++;
        if(i >= patt.length()) break;
    } 
    i--;  

    if(i < patt.length()-1){ 
        var_list = optimizer(patt, basisBits, e, i+1, e[i][dist], it, dist, stepSize);
    }

    for(int j = 0; j <= dist; j++){ 
        var_list.push_back(e[i][j]);
    }

    std::vector<Assign *> new_var_list = var_list;
    pb.createIf(cond, std::move(var_list), it);
    return new_var_list;
}

void Pattern_Compiler::compile(std::vector<std::string> patts, PabloBuilder & pb, std::vector<pablo::Var *> basisBits, int dist, int optPosition, int stepSize) {
    std::vector<std::vector<Assign*>> e(patts[0].length()*2, std::vector<Assign*>(dist+1));
    std::vector<PabloAST*> E(dist+1);
    int i = 0;
    int pi = 0;

    for(int d=0; d<=dist; d++){
        E[d] = pb.createZeroes();
    }

    for(int r=0; r< patts.size(); r++){
        std::string patt = patts[r];
        pi = (patt[0] >> 1) & 0x3;
        PabloAST * name = basisBits[pi];

        e[0][0] = pb.createAssign(make_e(0, 0), name);
        for(int j = 1; j <= dist; j++){
          e[0][j] = pb.createAssign(make_e(0, j), pb.createOnes());
        }

        i++;
        while (i < patt.length()) {
            pi = (patt[i] >> 1) & 0x3;
            name = basisBits[pi];

            e[i][0] = pb.createAssign(make_e(i,0), pb.createAnd(pb.createAdvance(e[i-1][0],1), name));
            for(int j = 1; j <= dist; j++){     
                auto tmp1 = pb.createAssign("tmp1", pb.createAnd(pb.createAdvance(e[i-1][j],1), name));
                auto tmp2 = pb.createAssign("tmp2", pb.createAnd(pb.createAdvance(e[i-1][j-1],1), pb.createNot(name)));
                auto tmp3 = pb.createAssign("tmp3", pb.createOr(pb.createAdvance(e[i][j-1],1), e[i-1][j-1]));
                e[i][j] = pb.createAssign(make_e(i,j), pb.createOr(pb.createOr(tmp1,tmp2),tmp3));
            }
            
            i++;
            if(i >= optPosition) break;
        }

        //Optimize from optPosition
        if(i >= optPosition){
            optimizer(patt, basisBits, e, i, e[i-1][dist], pb, dist, stepSize);
        }

        E[0] = pb.createOr(E[0], e[patt.length()-1][0]);
        for(int d=1; d<=dist; d++){
            E[d] = pb.createOr(E[d], pb.createAnd(e[patt.length()-1][d], pb.createNot(e[patt.length()-1][d-1])));
        }

        i = 0;
    }
    for(int d=0; d<=dist; d++){
        mFunction.setResult(d, pb.createAssign("E" + std::to_string(d), E[d]));
    }
}

}//end of re namespace
