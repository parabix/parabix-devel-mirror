/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */


#ifndef S2P_KERNEL_H
#define S2P_KERNEL_H

#include <stdio.h>
#include <IDISA/idisa_builder.h>
#include <llvm/IR/Module.h>

void generateS2PKernel(Module * m, IDISA::IDISA_Builder * iBuilder, KernelBuilder * kBuilder);

#endif

