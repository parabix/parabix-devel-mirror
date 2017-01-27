/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "evenodd.h"
#include <IR_Gen/idisa_builder.h>  // for IDISA_Builder
#include <llvm/IR/Constant.h>      // for Constant
#include <llvm/IR/Module.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/Module.h>
#include <llvm/ExecutionEngine/ExecutionEngine.h>
#include "llvm/Linker/Linker.h"
#include <llvm/Support/CommandLine.h>
#include <llvm/Support/raw_ostream.h>
namespace llvm { class BasicBlock; }
namespace llvm { class Function; }
namespace llvm { class Value; }

using namespace llvm;

namespace kernel {

    
void EvenOddKernel::generateDoBlockLogic(Value * self, Value * blockNo) const {
    Value * even = iBuilder->simd_fill(64, iBuilder->getInt64(0x5555555555555555));
    Value * odd = iBuilder->bitCast(iBuilder->simd_fill(8, iBuilder->getInt8(0xAA)));
    Value * evenBitsPtr = getStream(self, "even_odd", blockNo, iBuilder->getInt32(0));
    iBuilder->CreateBlockAlignedStore(even, evenBitsPtr);
    Value * oddBitsPtr = getStream(self, "even_odd", blockNo, iBuilder->getInt32(1));
    iBuilder->CreateBlockAlignedStore(odd, oddBitsPtr);
}

void EvenOddKernel::generateDoBlockMethod() const {
    auto savePoint = iBuilder->saveIP();

    Function * doBlockFunction = iBuilder->getModule()->getFunction(mKernelName + doBlock_suffix);
    
    iBuilder->SetInsertPoint(BasicBlock::Create(iBuilder->getContext(), "entry", doBlockFunction));
    
    Value * self = getParameter(doBlockFunction, "self");
    Value * blockNo = getScalarField(self, blockNoScalar);
    
    generateDoBlockLogic(self, blockNo);

    iBuilder->CreateRetVoid();
    iBuilder->restoreIP(savePoint);
}

void EvenOddKernel::generateFinalBlockMethod() const {
    auto savePoint = iBuilder->saveIP();
    Module * m = iBuilder->getModule();
    Function * finalBlockFunction = m->getFunction(mKernelName + finalBlock_suffix);
    iBuilder->SetInsertPoint(BasicBlock::Create(iBuilder->getContext(), "fb_entry", finalBlockFunction, 0));
    
    Value * self = getParameter(finalBlockFunction, "self");
    Value * remainingBytes = getParameter(finalBlockFunction, "remainingBytes");
    Value * blockNo = getScalarField(self, blockNoScalar);
    generateDoBlockLogic(self, blockNo);
    
    iBuilder->CreateRetVoid();
    iBuilder->restoreIP(savePoint);
}
    


EvenOddKernel::EvenOddKernel(IDISA::IDISA_Builder * builder)
: KernelBuilder(builder, "EvenOdd", {Binding{builder->getStreamSetTy(8, 1), "BasisBits"}}, {Binding{builder->getStreamSetTy(2, 1), "even_odd"}}, {}, {}, {}) {
    setNoTerminateAttribute(true);
    setDoBlockUpdatesProducedItemCountsAttribute(false);

}

}
