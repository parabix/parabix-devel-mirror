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
    

void generateStdOutKernel(Module * m, IDISA::IDISA_Builder * iBuilder, KernelBuilder * kBuilder, unsigned fw) {
    LLVMContext & ctxt = m->getContext();

    Type * i32 = iBuilder->getIntNTy(32);
    Type * i64 = iBuilder->getIntNTy(64);
    
    // Insert this declaration in the module (if necessary):  declare i64 @write(i32, i8*, i64) 
    Function * writefn = create_write(m);
    kBuilder->addInputStream(fw, "byte_pack");
    // No output streams.
    kBuilder->addInternalState(i64, "RemainingBytes");

    Function * function = kBuilder->prepareFunction();
    
    BasicBlock * full_block_write = BasicBlock::Create(ctxt, "full_block_write", function, 0);
    BasicBlock * final_block_write = BasicBlock::Create(ctxt, "final_block_write", function, 0);
    BasicBlock * exit_block = BasicBlock::Create(ctxt, "exit_stdout", function, 0);
    Value * bytes = iBuilder->CreateLoad(kBuilder->getInternalState("RemainingBytes"));

    Value * input = iBuilder->CreateBitCast(kBuilder->getInputStream(0), iBuilder->getInt8PtrTy());
    Value * blockSize = ConstantInt::get(i64, iBuilder->getBitBlockWidth());

    Value * fullblock_cond = iBuilder->CreateICmpULT(bytes, blockSize);
    iBuilder->CreateCondBr(fullblock_cond, final_block_write, full_block_write);
    
    iBuilder->SetInsertPoint(full_block_write);
    Value * outputBytes = ConstantInt::get(i64, iBuilder->getBitBlockWidth() * fw/8);
    iBuilder->CreateCall(writefn, std::vector<Value *>({ConstantInt::get(i32, 1), input, outputBytes}));
    Value * remain = iBuilder->CreateSub(bytes, blockSize);
    kBuilder->setInternalState("RemainingBytes", remain);
    iBuilder->CreatePtrToInt(remain, iBuilder->getInt64Ty());
    iBuilder->CreateBr(exit_block);
    
    iBuilder->SetInsertPoint(final_block_write);
    outputBytes = iBuilder->CreateMul(bytes, ConstantInt::get(i64, fw/8));
    iBuilder->CreateCall(writefn, std::vector<Value *>({ConstantInt::get(i32, 1), input, outputBytes}));
    kBuilder->setInternalState("RemainingBytes", ConstantInt::getNullValue(i64));
    iBuilder->CreateBr(exit_block);

    
    iBuilder->SetInsertPoint(exit_block);
    
    kBuilder->finalize();
}


    
}
