/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#ifndef IDISA_TARGET_H
#define IDISA_TARGET_H

#include <llvm/IR/Module.h>
#include <llvm/IR/Type.h>
#include <IDISA/idisa_builder.h>

IDISA::IDISA_Builder * GetIDISA_Builder(Module * m);

#endif
