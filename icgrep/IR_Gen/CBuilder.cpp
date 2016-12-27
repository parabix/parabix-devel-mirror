/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "CBuilder.h"
#include <llvm/IR/IRBuilder.h>
#include <llvm/IR/Constants.h>
#include <llvm/IR/Intrinsics.h>
#include <llvm/IR/Function.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/IR/TypeBuilder.h>

    
Function * CBuilder::GetPrintf() {
    Function * printf = mMod->getFunction("printf");
    if (printf == nullptr) {
        printf = cast<Function>(mMod->getOrInsertFunction("printf"
                                , FunctionType::get(getInt32Ty(), {getInt8PtrTy()}, true)
                                , AttributeSet().addAttribute(getContext(), 1U, Attribute::NoAlias)));

    }
    return printf;
}

void CBuilder::CallPrintInt(const std::string & name, Value * const value) {
    Constant * printRegister = mMod->getFunction("PrintInt");
    if (LLVM_UNLIKELY(printRegister == nullptr)) {
        FunctionType *FT = FunctionType::get(getVoidTy(), { PointerType::get(getInt8Ty(), 0), getSizeTy() }, false);
        Function * function = Function::Create(FT, Function::InternalLinkage, "PrintInt", mMod);
        auto arg = function->arg_begin();
        std::string out = "%-40s = %" PRIx64 "\n";
        BasicBlock * entry = BasicBlock::Create(mMod->getContext(), "entry", function);
        IRBuilder<> builder(entry);
        std::vector<Value *> args;
        args.push_back(CreateGlobalStringPtr(out.c_str()));
        Value * const name = &*(arg++);
        name->setName("name");
        args.push_back(name);
        Value * value = &*arg;
        value->setName("value");
        args.push_back(value);
        builder.CreateCall(GetPrintf(), args);
        builder.CreateRetVoid();

        printRegister = function;
    }
    Value * num = nullptr;
    if (value->getType()->isPointerTy()) {
        num = CreatePtrToInt(value, getSizeTy());
    } else {
        num = CreateZExtOrBitCast(value, getSizeTy());
    }
    assert (num->getType()->isIntegerTy());
    CreateCall(printRegister, {CreateGlobalStringPtr(name.c_str()), num});
}

Value * CBuilder::CreateMalloc(Type * type, Value * size) {
    DataLayout DL(getModule());
    Type * const intTy = getIntPtrTy(DL);
    Type * const voidPtrTy = getVoidPtrTy();
    Function * malloc = cast<Function>(getModule()->getOrInsertFunction("malloc", voidPtrTy, intTy, nullptr));
    malloc->setDoesNotAlias(0);
    const auto width = ConstantExpr::getSizeOf(type);
    if (!width->isOneValue()) {
        if (isa<Constant>(size)) {
            size = ConstantExpr::getMul(cast<Constant>(size), width);
        } else {
            size = CreateMul(size, width);
        }
    }
    size = CreateTruncOrBitCast(size, intTy);
    CallInst * ci = CreateCall(malloc, {size});
    ci->setTailCall();
    ci->setCallingConv(malloc->getCallingConv());
    return CreateBitOrPointerCast(ci, type->getPointerTo());
}

Value * CBuilder::CreateAlignedMalloc(Type * type, Value * size, const unsigned alignment) {
    assert ((alignment & (alignment - 1)) == 0); // is power of 2
    DataLayout DL(getModule());
    IntegerType * const intTy = getIntPtrTy(DL);
    const auto byteWidth = (intTy->getBitWidth() / 8);
    const auto offset = ConstantInt::get(intTy, alignment + byteWidth - 1);
    const auto width = ConstantExpr::getSizeOf(type);
    if (!width->isOneValue()) {
        if (isa<Constant>(size)) {
            size = ConstantExpr::getMul(cast<Constant>(size), width);
        } else {
            size = CreateMul(size, width);
        }
    }
    if (isa<Constant>(size)) {
        size = ConstantExpr::getAdd(cast<Constant>(size), offset);
    } else {
        size = CreateAdd(size, offset);
    }
    size = CreateTruncOrBitCast(size, intTy);
    Value * unaligned = CreateMalloc(getInt8Ty(), size);
    Value * aligned = CreateBitOrPointerCast(unaligned, intTy);
    aligned = CreateAnd(CreateAdd(aligned, offset), ConstantExpr::getNot(ConstantInt::get(intTy, alignment - 1)));
    Value * ptr = CreateBitOrPointerCast(CreateSub(aligned, ConstantInt::get(intTy, byteWidth)), intTy->getPointerTo());
    CreateAlignedStore(CreateBitOrPointerCast(unaligned, intTy), ptr, byteWidth);
    return CreateBitOrPointerCast(aligned, type->getPointerTo());
}

void CBuilder::CreateFree(Value * ptr) {
    PointerType * const voidPtrTy = getVoidPtrTy();
    Function * const free = cast<Function>(getModule()->getOrInsertFunction("free", getVoidTy(), voidPtrTy, nullptr));
    CallInst * const ci = CreateCall(free, {CreateBitOrPointerCast(ptr, voidPtrTy)});
    ci->setTailCall();
    ci->setCallingConv(free->getCallingConv());
}

void CBuilder::CreateAlignedFree(Value * ptr) {
    DataLayout DL(getModule());
    IntegerType * const intTy = getIntPtrTy(DL);
    const auto byteWidth = (intTy->getBitWidth() / 8);
    ptr = CreateBitOrPointerCast(ptr, intTy);
    ptr = CreateSub(ptr, ConstantInt::get(intTy, byteWidth));
    ptr = CreateBitOrPointerCast(ptr, getInt8PtrTy());
    CreateFree(CreateAlignedLoad(ptr, byteWidth));
}

Value * CBuilder::CreateRealloc(Value * ptr, Value * size) {
    assert (ptr->getType()->isPointerTy());
    DataLayout DL(getModule());
    IntegerType * const intTy = getIntPtrTy(DL);
    PointerType * const voidPtrTy = getVoidPtrTy();
    Function * realloc = cast<Function>(getModule()->getOrInsertFunction("realloc", voidPtrTy, voidPtrTy, intTy, nullptr));
    realloc->setDoesNotAlias(0);
    Type * const type = ptr->getType();
    // calculate our new size parameter
    size = CreateMul(size, ConstantExpr::getSizeOf(type->getPointerElementType()));
    size = CreateTruncOrBitCast(size, intTy);
    // call realloc with the pointer and adjusted size
    CallInst * ci = CreateCall(realloc, {ptr, size});
    ci->setTailCall();
    ci->setCallingConv(realloc->getCallingConv());
    return CreateBitOrPointerCast(ci, type);
}

Value * CBuilder::CreateAlignedRealloc(Value * ptr, Value * size, const unsigned alignment) {
    assert ((alignment & (alignment - 1)) == 0); // is power of 2
    assert (ptr->getType()->isPointerTy());
    DataLayout DL(getModule());
    IntegerType * const intTy = getIntPtrTy(DL);
    PointerType * const bpTy = getInt8PtrTy();
    Type * const type = ptr->getType();
    // calculate our new size parameter
    const auto byteWidth = (intTy->getBitWidth() / 8);
    const auto offset = ConstantInt::get(intTy, alignment + byteWidth - 1);
    const auto width = ConstantExpr::getSizeOf(type);
    if (!width->isOneValue()) {
        if (isa<Constant>(size)) {
            size = ConstantExpr::getMul(cast<Constant>(size), width);
        } else {
            size = CreateMul(size, width);
        }
    }
    if (isa<Constant>(size)) {
        size = ConstantExpr::getAdd(cast<Constant>(size), offset);
    } else {
        size = CreateAdd(size, offset);
    }
    size = CreateTruncOrBitCast(size, intTy);
    // calculate the offset containing the unaligned pointer address
    ptr = CreateBitOrPointerCast(ptr, bpTy);
    ptr = CreateSub(ptr, ConstantInt::get(intTy, byteWidth));
    ptr = CreateBitOrPointerCast(ptr, intTy->getPointerTo());
    // load the unaligned pointer as an uint8 *
    ptr = CreateAlignedLoad(ptr, byteWidth);
    ptr = CreateBitOrPointerCast(ptr, bpTy);
    // call realloc with the unaligned pointer and adjusted size
    Value * unaligned = CreateRealloc(ptr, size);
    Value * aligned = CreateBitOrPointerCast(unaligned, intTy);
    aligned = CreateAnd(CreateAdd(aligned, offset), ConstantExpr::getNot(ConstantInt::get(intTy, alignment - 1)));
    Value * prefix = CreateBitOrPointerCast(CreateSub(aligned, ConstantInt::get(intTy, byteWidth)), intTy->getPointerTo());
    CreateAlignedStore(CreateBitOrPointerCast(unaligned, intTy), prefix, byteWidth);
    return CreateBitOrPointerCast(aligned, type);
}

void CBuilder::CreateMemZero(Value * ptr, Value * size, const unsigned alignment) {
    assert (ptr->getType()->isPointerTy() && size->getType()->isIntegerTy());
    Type * const type = ptr->getType();
    const auto width = ConstantExpr::getSizeOf(type->getPointerElementType());
    if (isa<Constant>(size)) {
        size = ConstantExpr::getMul(cast<Constant>(size), width);
    } else {
        size = CreateMul(size, width);
    }
    CreateMemSet(CreateBitOrPointerCast(ptr, getInt8PtrTy()), getInt8(0), size, alignment);
}

PointerType * CBuilder::getVoidPtrTy() const {
    return TypeBuilder<void *, false>::get(getContext());
}


LoadInst * CBuilder::CreateAtomicLoadAcquire(Value * ptr) {
    unsigned alignment = dyn_cast<PointerType>(ptr->getType())->getElementType()->getPrimitiveSizeInBits()/8;
    LoadInst * inst = CreateAlignedLoad(ptr, alignment);
    inst->setOrdering(AtomicOrdering::Acquire);
    return inst;
    
}
StoreInst * CBuilder::CreateAtomicStoreRelease(Value * val, Value * ptr) {
    unsigned alignment = dyn_cast<PointerType>(ptr->getType())->getElementType()->getPrimitiveSizeInBits()/8;
    StoreInst * inst = CreateAlignedStore(val, ptr, alignment);
    inst->setOrdering(AtomicOrdering::Release);
    return inst;
}
