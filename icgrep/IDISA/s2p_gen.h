/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */


#ifndef S2P_GEN_H
#define S2P_GEN_H

#include <stdio.h>
#include <IDISA/idisa_builder.h>
#include <llvm/IR/Module.h>

void gen_s2p_function(Module * m, IDISA::IDISA_Builder * iBuilder);

#endif

