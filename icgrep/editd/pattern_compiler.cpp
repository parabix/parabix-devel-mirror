/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "pattern_compiler.h"
#include <pablo/builder.hpp>
#include <pablo/pe_zeroes.h>
#include <pablo/pe_ones.h>

using namespace pablo;

namespace re {


Pattern_Compiler::Pattern_Compiler(PabloKernel & kernel)
: mKernel(kernel)
{

}


inline std::string make_e(int i, int j) {return ("e_"+std::to_string(i)+"_"+std::to_string(j));}


void optimizer(const std::string & patt, PabloAST * basisBits[], std::vector<std::vector<PabloAST *>> & e, unsigned i, PabloAST * cond, PabloBuilder & main, PabloBuilder & pb, int dist, int stepSize){
    PabloBuilder it = PabloBuilder::Create(pb);

    Zeroes * zeroes = pb.createZeroes();

    for(int n = 0; n < stepSize; n++){

        PabloAST * name = basisBits[(patt[i] >> 1) & 0x3];
        
        e[i][0] = it.createAnd(it.createAdvance(e[i - 1][0], 1), name, make_e(i, 0));
        for(int j = 1; j <= dist; j++){
            auto tmp1 = it.createAnd(it.createAdvance(e[i-1][j],1), name);
            auto tmp2 = it.createAnd(it.createAdvance(e[i-1][j-1],1), it.createNot(name));
            auto tmp3 = it.createOr(it.createAdvance(e[i][j-1],1), e[i-1][j-1]);
            e[i][j] = it.createOr(it.createOr(tmp1, tmp2), tmp3, make_e(i, j));
        }
        
        i++;
        if (i >= patt.length()) break;
    } 

    if (i < patt.length()) {
        optimizer(patt, basisBits, e, i, e[i - 1][dist], main, it, dist, stepSize);
    } else {
        const auto i = patt.length() - 1;
        for(int j = 0; j <= dist; j++){
            auto eij = main.createVar("m" + std::to_string(j), zeroes);
            it.createAssign(eij, e[i][j]);
            e[i][j] = eij;
        }
    }

    pb.createIf(cond, it);
}

void Pattern_Compiler::compile(const std::vector<std::string> & patts, PabloBuilder & pb, PabloAST * basisBits[], int dist, unsigned optPosition, int stepSize) {
    std::vector<std::vector<PabloAST *>> e(patts[0].length() * 2, std::vector<PabloAST *>(dist+1));
    std::vector<PabloAST*> E(dist + 1);

    for(int d=0; d<=dist; d++){
        E[d] = pb.createZeroes();
    }

    for(unsigned r = 0; r < patts.size(); r++){
        const std::string & patt = patts[r];

        PabloAST * name = basisBits[(patt[0] >> 1) & 0x3];

        e[0][0] = name;
        for(int j = 1; j <= dist; j++){
          e[0][j] = pb.createOnes();
        }

        unsigned i = 1;
        while (i < patt.length() && i < optPosition) {

            name = basisBits[(patt[i] >> 1) & 0x3];

            e[i][0] = pb.createAnd(pb.createAdvance(e[i-1][0], 1), name, make_e(i, 0));
            for(int j = 1; j <= dist; j++){     
                auto tmp1 = pb.createAnd(pb.createAdvance(e[i-1][j],1), name);
                auto tmp2 = pb.createAnd(pb.createAdvance(e[i-1][j-1],1), pb.createNot(name));
                auto tmp3 = pb.createOr(pb.createAdvance(e[i][j-1],1), e[i-1][j-1]);
                e[i][j] = pb.createOr(pb.createOr(tmp1,tmp2),tmp3, make_e(i, j));
            }
            
            i++;
        }

        //Optimize from optPosition
        if (i >= optPosition) {
            optimizer(patt, basisBits, e, i, e[i-1][dist], pb, pb, dist, stepSize);
        }

        E[0] = pb.createOr(E[0], e[patt.length() - 1][0]);
        for(int d=1; d<=dist; d++){
            E[d] = pb.createOr(E[d], pb.createAnd(e[patt.length()-1][d], pb.createNot(e[patt.length()-1][d-1])));
        }

    }


    Var * output = mKernel.getOutputStreamVar("E");
    for (int d = 0; d <= dist; d++) {
        pb.createAssign(pb.createExtract(output, d), E[d]);
    }
}

}//end of re namespace
