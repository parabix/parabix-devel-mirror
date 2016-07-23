/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#include <kernels/stdout_kernel.h>
#include <kernels/kernel.h>
#include <IDISA/idisa_builder.h>
#include <llvm/IR/TypeBuilder.h>

namespace kernel {

static Function * create_write(Module * const mod) {
    Function * write = mod->getFunction("write");
    if (write == nullptr) {
        FunctionType *write_type =
        TypeBuilder<long(int, char *, long), false>::get(mod->getContext());
        write = cast<Function>(mod->getOrInsertFunction("write", write_type,
                                                         AttributeSet().addAttribute(mod->getContext(), 2U, Attribute::NoAlias)));
    }
    return write;
}

//  Override the default void type for DoBlock functions. 
void stdOutKernel::prepareKernel() {
    setDoBlockReturnType(mStreamType);
    KernelBuilder::prepareKernel();
}


void stdOutKernel::generateDoBlockMethod() {
    IDISA::IDISA_Builder::InsertPoint savePoint = iBuilder->saveIP();
    Module * m = iBuilder->getModule();
    Function * writefn = create_write(m);
    Function * doBlockFunction = m->getFunction(mKernelName + doBlock_suffix);
    Type * i8PtrTy = iBuilder->getInt8PtrTy();

    iBuilder->SetInsertPoint(BasicBlock::Create(iBuilder->getContext(), "entry", doBlockFunction, 0));

    Value * self = getParameter(doBlockFunction, "self");
    Value * blockNo = getScalarField(self, blockNoScalar);
    Value * inputStreamBlock = getStreamSetBlockPtr(self, "inputStreamSet", blockNo);

    Value * bufferPtr = getScalarField(self, "bufferPtr");
    Value * bufferFinalBlockPtr = getScalarField(self, "bufferFinalBlockPtr");
    //iBuilder->CallPrintInt("bufferPtr", iBuilder->CreatePtrToInt(bufferPtr, iBuilder->getInt64Ty()));
    //iBuilder->CallPrintInt("bufferFinalBlockPtr", iBuilder->CreatePtrToInt(bufferFinalBlockPtr, iBuilder->getInt64Ty()));
    
    
    BasicBlock * flushBlock = BasicBlock::Create(iBuilder->getContext(), "flush", doBlockFunction, 0);
    BasicBlock * exitBlock = BasicBlock::Create(iBuilder->getContext(), "exit", doBlockFunction, 0);
    Value * inFinal = iBuilder->CreateICmpUGT(bufferPtr, bufferFinalBlockPtr);
    iBuilder->CreateCondBr(inFinal, flushBlock, exitBlock);
    
    iBuilder->SetInsertPoint(flushBlock);
    Value * basePtr = getScalarField(self, "bufferBasePtr");
    //iBuilder->CallPrintInt("bufferBasePtr", iBuilder->CreatePtrToInt(basePtr, iBuilder->getInt64Ty()));
    Value * baseAddress = iBuilder->CreatePtrToInt(basePtr, iBuilder->getInt64Ty());
    Value * pointerAddress = iBuilder->CreatePtrToInt(bufferPtr, iBuilder->getInt64Ty());
    Value * bytesToFlush = iBuilder->CreateSub(pointerAddress, baseAddress);
    
    iBuilder->CreateCall(writefn, std::vector<Value *>({iBuilder->getInt32(1), iBuilder->CreateBitCast(basePtr, i8PtrTy), bytesToFlush}));
    // Buffer is flushed, return the buffer base pointer for subsequent output to the buffer.
    iBuilder->CreateRet(basePtr);

    iBuilder->SetInsertPoint(exitBlock);
    iBuilder->CreateRet(bufferPtr);
    iBuilder->restoreIP(savePoint);
}

void stdOutKernel::generateFinalBlockMethod() {
    IDISA::IDISA_Builder::InsertPoint savePoint = iBuilder->saveIP();
    Module * m = iBuilder->getModule();
    Function * writefn = create_write(m);
    Function * finalBlockFunction = m->getFunction(mKernelName + finalBlock_suffix);
    Type * i8PtrTy = iBuilder->getInt8PtrTy();
    
    iBuilder->SetInsertPoint(BasicBlock::Create(iBuilder->getContext(), "fb_flush", finalBlockFunction, 0));
    Value * self = getParameter(finalBlockFunction, "self");
    Value * bufferPtr = getParameter(finalBlockFunction, "bufferPtr");
    Value * basePtr = getScalarField(self, "bufferBasePtr");
    // Flush the output.
    Value * baseAddress = iBuilder->CreatePtrToInt(basePtr, iBuilder->getInt64Ty());
    Value * pointerAddress = iBuilder->CreatePtrToInt(bufferPtr, iBuilder->getInt64Ty());
    Value * bytesToFlush = iBuilder->CreateSub(pointerAddress, baseAddress);
    
    iBuilder->CreateCall(writefn, std::vector<Value *>({iBuilder->getInt32(1), iBuilder->CreateBitCast(basePtr, i8PtrTy), bytesToFlush}));
    // Buffer is flushed, return the buffer base pointer for subsequent output to the buffer.
    iBuilder->CreateRet(basePtr);
    iBuilder->restoreIP(savePoint);
}
}
