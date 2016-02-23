#ifndef SCANMATCHGEN_H
#define SCANMATCHGEN_H
/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */


#include <IDISA/idisa_builder.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/Module.h>
        
void generateScanBitBlockRoutine(Module * m, IDISA::IDISA_Builder * iBuilder, int segBitWidth, Type * kernelStuctType, bool isNameExpression);

void generateScanMatch(Module * m, IDISA::IDISA_Builder * iBuilder, int segBitWidth, KernelBuilder * kBuilder, bool isNameExpression);

#endif // SCANMATCHGEN_H
