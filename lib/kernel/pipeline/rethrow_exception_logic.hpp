#ifndef RETHROW_EXCEPTION_LOGIC_HPP
#define RETHROW_EXCEPTION_LOGIC_HPP

#include "pipeline_compiler.hpp"

namespace kernel {

void PipelineCompiler::initializeExceptionRethrow(BuilderRef b) {

    KernelBuilder * b;

    const auto ip = saveIP();

    BasicBlock * const current = GetInsertBlock();
    Function * const f = current->getParent();

    f->setPersonalityFn(getDefaultPersonalityFunction());
    f->addFnAttr(Attribute::UWTable);

    LLVMContext & C = getContext();

    BasicBlock * const handleCatch = BasicBlock::Create(C, "__catch", f);
    BasicBlock * const handleRethrow = BasicBlock::Create(C, "__rethrow", f);
    BasicBlock * const handleResume = BasicBlock::Create(C, "__resume", f);
    BasicBlock * const handleExit = BasicBlock::Create(C, "__exit", f);
    BasicBlock * const handleUnreachable = BasicBlock::Create(C, "__unreachable", f);

    PointerType * const int8PtrTy = getInt8PtrTy();
    IntegerType * const int32Ty = getInt32Ty();
    StructType * const caughtResultType = StructType::get(C, { int8PtrTy, int32Ty });
    Constant * const catchAny = ConstantPointerNull::get(int8PtrTy);

    ConstantInt * handle = getSize((uintptr_t)(handleCatch));
    ConstantInt * stdErr = getInt32(STDERR_FILENO);

    std::string tmp = f->getName();

    SetInsertPoint(handleCatch);
    LandingPadInst * const caughtResult = CreateLandingPad(caughtResultType, 1);
    caughtResult->addClause(catchAny);
    Value * exception = CreateExtractValue(caughtResult, 0);
    CallPrintInt(tmp + "_catch", handle);
    CreateFSync(stdErr);
    CallInst * beginCatch = CreateCall(getBeginCatch(), {exception});
    beginCatch->setTailCall(true);
    beginCatch->addAttribute(-1, Attribute::NoUnwind);
    InvokeInst * const rethrowInst = CreateInvoke(getRethrow(), handleUnreachable, handleRethrow);
    rethrowInst->addAttribute(-1, Attribute::NoReturn);

    SetInsertPoint(handleRethrow);
    LandingPadInst * const caughtResult2 = CreateLandingPad(caughtResultType, 0);
    caughtResult2->setCleanup(true);
    CallPrintInt(tmp + "_rethrow", handle);
    CreateFSync(stdErr);
    CreateInvoke(getEndCatch(), handleResume, handleExit);

    SetInsertPoint(handleResume);
    CallPrintInt(tmp + "_resume", handle);
    CreateFSync(stdErr);
    CreateResume(caughtResult);

    SetInsertPoint(handleExit);
    LandingPadInst * const caughtResult3 = CreateLandingPad(caughtResultType, 1);
    caughtResult3->addClause(catchAny);
    CallPrintInt(tmp + "_exit", handle);
    CreateFSync(stdErr);
    // should call std::terminate
    CreateBr(handleUnreachable);

    SetInsertPoint(handleUnreachable);
    CallPrintInt(tmp + "_resume", handle);
    CreateFSync(stdErr);
    CreateUnreachable();

    restoreIP(ip);




}

Value * PipelineCompiler::invokeKernelOrRethrow(BuilderRef b, Value * const function, ArrayRef<Value *> args) {

    PointerType * const int8PtrTy = b->getInt8PtrTy();
    IntegerType * const int32Ty = b->getInt32Ty();
    StructType * const caughtResultType = StructType::get(b->getContext(), { int8PtrTy, int32Ty });
    Constant * const catchAny = ConstantPointerNull::get(int8PtrTy);

    BasicBlock * const invokeCatch = b->CreateBasicBlock("", mKernelTerminationCheck);
    BasicBlock * const catchRethrow = b->CreateBasicBlock("", mKernelTerminationCheck);
    BasicBlock * const invokeOk = b->CreateBasicBlock("", mKernelTerminationCheck);
    mTerminatedExplicitly = b->CreateInvoke(function, invokeOk, invokeCatch, args);
    b->SetInsertPoint(invokeCatch);
    LandingPadInst * const caughtResult = b->CreateLandingPad(caughtResultType, 1);
    caughtResult->addClause(catchAny);
    Value * const exception = b->CreateExtractValue(caughtResult, 0);
    CallInst * const beginCatch = b->CreateCall(b->getBeginCatch(), {exception});
    beginCatch->setTailCall(true);
    beginCatch->addAttribute(-1, Attribute::NoUnwind);

    InvokeInst * const rethrowInst = b->CreateInvoke(b->getRethrow(), mExceptionUnreachable, catchRethrow);
    rethrowInst->addAttribute(-1, Attribute::NoReturn);

    b->SetInsertPoint(catchRethrow);
    LandingPadInst * const caughtResult2 = b->CreateLandingPad(caughtResultType, 0);
    caughtResult2->setCleanup(true);



    b->CreateInvoke(b->getEndCatch(), handleResume, mExceptionUnreachable);


    b->SetInsertPoint(invokeOk);


}



}

#endif // RETHROW_EXCEPTION_LOGIC_HPP
