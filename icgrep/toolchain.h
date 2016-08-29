/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef TOOLCHAIN_H
#define TOOLCHAIN_H

#include <llvm/IR/Module.h>
#include <llvm/ExecutionEngine/ExecutionEngine.h>
#include <llvm/Support/CommandLine.h>

namespace codegen {
const llvm::cl::OptionCategory * codegen_flags();

// Command Parameters
extern char OptLevel;  // set from command line
extern int BlockSize;  // set from command line
extern int SegmentSize;  // set from command line
extern int BufferSegments;

}

bool AVX2_available();

llvm::ExecutionEngine * JIT_to_ExecutionEngine (llvm::Module * m);

void ApplyObjectCache(llvm::ExecutionEngine * e);

#endif
