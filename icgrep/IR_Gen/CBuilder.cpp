/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "CBuilder.h"
#include <llvm/IR/Module.h>
#include <llvm/IR/Constants.h>
#include <llvm/IR/Intrinsics.h>
#include <llvm/IR/TypeBuilder.h>
#include <llvm/IR/MDBuilder.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <toolchain.h>

#include <llvm/Support/raw_ostream.h>

using namespace llvm;

Value * CBuilder::CreateOpenCall(Value * filename, Value * oflag, Value * mode) {
    Function * openFn = mMod->getFunction("open");
    if (openFn == nullptr) {
        IntegerType * int32Ty = getInt32Ty();
        PointerType * int8PtrTy = getInt8PtrTy();
        openFn = cast<Function>(mMod->getOrInsertFunction("open",
                                                         int32Ty, int8PtrTy, int32Ty, int32Ty, nullptr));
    }
    return CreateCall(openFn, {filename, oflag, mode});
}

// ssize_t write(int fildes, const void *buf, size_t nbyte);
Value * CBuilder::CreateWriteCall(Value * fildes, Value * buf, Value * nbyte) {
    Function * write = mMod->getFunction("write");
    if (write == nullptr) {
        IntegerType * sizeTy = getSizeTy();
        IntegerType * int32Ty = getInt32Ty();
        PointerType * int8PtrTy = getInt8PtrTy();
        write = cast<Function>(mMod->getOrInsertFunction("write",
                                                        AttributeSet().addAttribute(mMod->getContext(), 2U, Attribute::NoAlias),
                                                        sizeTy, int32Ty, int8PtrTy, sizeTy, nullptr));
    }
    return CreateCall(write, {fildes, buf, nbyte});
}

Value * CBuilder::CreateReadCall(Value * fildes, Value * buf, Value * nbyte) {
    Function * readFn = mMod->getFunction("read");
    if (readFn == nullptr) {
        IntegerType * sizeTy = getSizeTy();
        IntegerType * int32Ty = getInt32Ty();
        PointerType * int8PtrTy = getInt8PtrTy();
        readFn = cast<Function>(mMod->getOrInsertFunction("read",
                                                         AttributeSet().addAttribute(mMod->getContext(), 2U, Attribute::NoAlias),
                                                         sizeTy, int32Ty, int8PtrTy, sizeTy, nullptr));
    }
    return CreateCall(readFn, {fildes, buf, nbyte});
}

Value * CBuilder::CreateCloseCall(Value * fildes) {
    Function * closeFn = mMod->getFunction("close");
    if (closeFn == nullptr) {
        IntegerType * int32Ty = getInt32Ty();
        FunctionType * fty = FunctionType::get(int32Ty, {int32Ty}, true);
        closeFn = Function::Create(fty, Function::ExternalLinkage, "close", mMod);
    }
    return CreateCall(closeFn, {fildes});
}

Function * CBuilder::GetPrintf() {
    Function * printf = mMod->getFunction("printf");
    if (printf == nullptr) {
        FunctionType * fty = FunctionType::get(getInt32Ty(), {getInt8PtrTy()}, true);
        printf = Function::Create(fty, Function::ExternalLinkage, "printf", mMod);
        printf->addAttribute(1, Attribute::NoAlias);
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

Value * CBuilder::CreateMalloc(Value * size) {
    Module * const m = getModule();
    DataLayout DL(m);
    IntegerType * const intTy = getIntPtrTy(DL);
    if (size->getType() != intTy) {
        if (isa<Constant>(size)) {
            size = ConstantExpr::getIntegerCast(cast<Constant>(size), intTy, false);
        } else {
            size = CreateZExtOrTrunc(size, intTy);
        }
    }    
    PointerType * const voidPtrTy = getVoidPtrTy();
    Function * malloc = m->getFunction("malloc");
    if (malloc == nullptr) {
        FunctionType * fty = FunctionType::get(voidPtrTy, {intTy}, false);
        malloc = Function::Create(fty, Function::ExternalLinkage, "malloc", mMod);
        malloc->setCallingConv(CallingConv::C);
        malloc->setDoesNotAlias(0);
    }
    assert (size->getType() == intTy);
    CallInst * ci = CreateCall(malloc, size); assert (ci);
    ci->setTailCall();
    ci->setCallingConv(malloc->getCallingConv());
    Value * ptr = CreatePointerCast(ci, voidPtrTy); assert (ptr);
    CreateAssert(ptr, "FATAL ERROR: out of memory");
    return ptr;
}

Value * CBuilder::CreateAlignedMalloc(Value * size, const unsigned alignment) {
    if (LLVM_UNLIKELY((alignment & (alignment - 1)) != 0)) {
        report_fatal_error("CreateAlignedMalloc: alignment must be a power of 2");
    }
    DataLayout DL(mMod);
    IntegerType * const intTy = getIntPtrTy(DL);
    Function * aligned_malloc = mMod->getFunction("aligned_malloc" + std::to_string(alignment));
    if (LLVM_UNLIKELY(aligned_malloc == nullptr)) {
        const auto ip = saveIP();
        PointerType * const voidPtrTy = getVoidPtrTy();
        FunctionType * fty = FunctionType::get(voidPtrTy, {intTy}, false);
        aligned_malloc = Function::Create(fty, Function::InternalLinkage, "aligned_malloc" + std::to_string(alignment), mMod);
        aligned_malloc->setCallingConv(CallingConv::C);
        aligned_malloc->setDoesNotAlias(0);
        aligned_malloc->addFnAttr(Attribute::AlwaysInline);
        Value * size = &*aligned_malloc->arg_begin();
        SetInsertPoint(BasicBlock::Create(mMod->getContext(), "entry", aligned_malloc));
        const auto byteWidth = (intTy->getBitWidth() / 8);
        Constant * const offset = ConstantInt::get(intTy, alignment + byteWidth - 1);
        size = CreateAdd(size, offset);
        Value * unaligned = CreatePtrToInt(CreateMalloc(size), intTy);
        Value * aligned = CreateAnd(CreateAdd(unaligned, offset), ConstantExpr::getNot(ConstantInt::get(intTy, alignment - 1)));
        Value * prefix = CreateIntToPtr(CreateSub(aligned, ConstantInt::get(intTy, byteWidth)), intTy->getPointerTo());
        assert (unaligned->getType() == prefix->getType()->getPointerElementType());
        CreateAlignedStore(unaligned, prefix, byteWidth);
        CreateRet(CreateIntToPtr(aligned, voidPtrTy));
        restoreIP(ip);
    }
    return CreateCall(aligned_malloc, {CreateZExtOrTrunc(size, intTy)});
}

Value * CBuilder::CreateAnonymousMMap(Value * size) {
    DataLayout DL(mMod);
    PointerType * const voidPtrTy = getVoidPtrTy();
    IntegerType * const intTy = getIntPtrTy(DL);
    IntegerType * const sizeTy = getSizeTy();
    Type * const offTy = TypeBuilder<off_t, false>::get(getContext());
    Function * fMMap = mMod->getFunction("mmap");
    if (LLVM_UNLIKELY(fMMap == nullptr)) {
        FunctionType * fty = FunctionType::get(voidPtrTy, {voidPtrTy, sizeTy, intTy, intTy, intTy, offTy}, false);
        fMMap = Function::Create(fty, Function::ExternalLinkage, "mmap", mMod);
    }
    size = CreateZExtOrTrunc(size, sizeTy);
    ConstantInt * const prot =  ConstantInt::get(intTy, PROT_READ | PROT_WRITE);
    ConstantInt * const flags =  ConstantInt::get(intTy, MAP_PRIVATE | MAP_ANONYMOUS);
    ConstantInt * const fd =  ConstantInt::get(intTy, -1);
    Constant * const offset = ConstantInt::get(offTy, 0); // getCacheAlignment()
    Value * const ptr = CreateCall(fMMap, {Constant::getNullValue(voidPtrTy), size, prot, flags, fd, offset});
    CreateAssert(CreateICmpNE(CreatePtrToInt(ptr, getSizeTy()), getSize((size_t)MAP_FAILED)), "CreateAnonymousMMap: mmap failed to allocate memory");
    return ptr;
}

Value * CBuilder::CreateFileSourceMMap(Value * const fd, Value * size) {
    DataLayout DL(mMod);
    PointerType * const voidPtrTy = getVoidPtrTy();
    IntegerType * const intTy = getIntPtrTy(DL);
    IntegerType * const sizeTy = getSizeTy();
    Type * const offTy = TypeBuilder<off_t, false>::get(getContext());
    Function * fMMap = mMod->getFunction("mmap");
    if (LLVM_UNLIKELY(fMMap == nullptr)) {
        FunctionType * fty = FunctionType::get(voidPtrTy, {voidPtrTy, sizeTy, intTy, intTy, intTy, offTy}, false);
        fMMap = Function::Create(fty, Function::ExternalLinkage, "mmap", mMod);
    }
    size = CreateZExtOrTrunc(size, sizeTy);
    ConstantInt * const prot =  ConstantInt::get(intTy, PROT_READ);
    ConstantInt * const flags =  ConstantInt::get(intTy, MAP_PRIVATE);
    Constant * const offset = ConstantInt::get(offTy, 0); // getCacheAlignment()
    Value * const ptr = CreateCall(fMMap, {Constant::getNullValue(voidPtrTy), size, prot, flags, fd, offset});
    CreateAssert(CreateICmpNE(CreatePtrToInt(ptr, getSizeTy()), getSize((size_t)MAP_FAILED)), "CreateFileSourceMMap: mmap failed to allocate memory");
    return ptr;
}

Value * CBuilder::CreateMRemap(Value * addr, Value * oldSize, Value * newSize, const bool mayMove) {
    DataLayout DL(mMod);
    PointerType * const voidPtrTy = getVoidPtrTy();
    IntegerType * const intTy = getIntPtrTy(DL);
    IntegerType * const sizeTy = getSizeTy();
    Function * fMRemap = mMod->getFunction("mremap");
    if (LLVM_UNLIKELY(fMRemap == nullptr)) {
        FunctionType * fty = FunctionType::get(voidPtrTy, {voidPtrTy, sizeTy, sizeTy, intTy}, false);
        fMRemap = Function::Create(fty, Function::ExternalLinkage, "mremap", mMod);
    }    
    addr = CreatePointerCast(addr, voidPtrTy);
    CreateAssert(addr, "CreateMRemap: addr cannot be null");
    oldSize = CreateZExtOrTrunc(oldSize, sizeTy);
    newSize = CreateZExtOrTrunc(newSize, sizeTy);
    ConstantInt * const flags = ConstantInt::get(intTy, mayMove ? MREMAP_MAYMOVE : 0);
    Value * ptr = CreateCall(fMRemap, {addr, oldSize, newSize, flags});
    CreateAssert(addr, "CreateMRemap: mremap failed to allocate memory");
    return ptr;
}

Value * CBuilder::CreateMUnmap(Value * addr, Value * size) {
    DataLayout DL(mMod);
    IntegerType * const sizeTy = getSizeTy();
    PointerType * const voidPtrTy = getVoidPtrTy();
    Function * fMUnmap = mMod->getFunction("munmap");
    if (LLVM_UNLIKELY(fMUnmap == nullptr)) {
        IntegerType * const intTy = getIntPtrTy(DL);
        FunctionType * fty = FunctionType::get(intTy, {voidPtrTy, sizeTy}, false);
        fMUnmap = Function::Create(fty, Function::ExternalLinkage, "munmap", mMod);
    }
    addr = CreatePointerCast(addr, voidPtrTy);
    CreateAssert(addr, "CreateMUnmap: addr cannot be null");
    size = CreateZExtOrTrunc(size, sizeTy);
    return CreateCall(fMUnmap, {addr, size});
}

void CBuilder::CreateFree(Value * const ptr) {
    assert (ptr->getType()->isPointerTy());
    Module * const m = getModule();
    PointerType * const voidPtrTy = getVoidPtrTy();
    Function * free = m->getFunction("free");
    if (free == nullptr) {
        FunctionType * fty = FunctionType::get(getVoidTy(), {voidPtrTy}, false);
        free = Function::Create(fty, Function::ExternalLinkage, "free", mMod);
        free->setCallingConv(CallingConv::C);
    }
    CallInst * const ci = CreateCall(free, CreatePointerCast(ptr, voidPtrTy));
    ci->setTailCall();
    ci->setCallingConv(free->getCallingConv());
}

void CBuilder::CreateAlignedFree(Value * const ptr, const bool testForNullAddress) {
    // WARNING: this will segfault if the value of the ptr at runtime is null but testForNullAddress was not set
    PointerType * type = cast<PointerType>(ptr->getType());
    BasicBlock * exit = nullptr;
    if (testForNullAddress) {
        LLVMContext & C = getContext();
        BasicBlock * bb = GetInsertBlock();
        Function * f = bb->getParent();
        exit = BasicBlock::Create(C, "", f, bb);
        BasicBlock * entry = BasicBlock::Create(C, "", f, exit);
        Value * cond = CreateICmpEQ(ptr, ConstantPointerNull::get(type));
        CreateCondBr(cond, exit, entry);
        SetInsertPoint(entry);
    }
    DataLayout DL(getModule());
    IntegerType * const intTy = getIntPtrTy(DL);
    const auto byteWidth = (intTy->getBitWidth() / 8);
    Value * prefix = CreatePtrToInt(ptr, intTy);
    prefix = CreateSub(prefix, ConstantInt::get(intTy, byteWidth));
    prefix = CreateIntToPtr(prefix, intTy->getPointerTo());
    prefix = CreateIntToPtr(CreateAlignedLoad(prefix, byteWidth), type);
    CreateFree(prefix);
    if (testForNullAddress) {
        CreateBr(exit);
        SetInsertPoint(exit);
    }
}

Value * CBuilder::CreateRealloc(Value * ptr, Value * size) {
    DataLayout DL(getModule());
    IntegerType * const intTy = getIntPtrTy(DL);
    PointerType * type = cast<PointerType>(ptr->getType());
    if (size->getType() != intTy) {
        if (isa<Constant>(size)) {
            size = ConstantExpr::getIntegerCast(cast<Constant>(size), intTy, false);
        } else {
            size = CreateZExtOrTrunc(size, intTy);
        }
    }
    Module * const m = getModule();
    Function * realloc = m->getFunction("realloc");
    if (realloc == nullptr) {
        PointerType * const voidPtrTy = getVoidPtrTy();
        FunctionType * fty = FunctionType::get(voidPtrTy, {voidPtrTy, intTy}, false);
        realloc = Function::Create(fty, Function::ExternalLinkage, "realloc", mMod);
        realloc->setCallingConv(CallingConv::C);
        realloc->setDoesNotAlias(1);
    }
    assert (size->getType() == intTy);
    CallInst * ci = CreateCall(realloc, {ptr, size});
    ci->setTailCall();
    ci->setCallingConv(realloc->getCallingConv());
    return CreateBitOrPointerCast(ci, type);
}

PointerType * CBuilder::getVoidPtrTy() const {
    return TypeBuilder<void *, false>::get(getContext());
}

LoadInst * CBuilder::CreateAtomicLoadAcquire(Value * ptr) {
    const auto alignment = ptr->getType()->getPointerElementType()->getPrimitiveSizeInBits() / 8;
    LoadInst * inst = CreateAlignedLoad(ptr, alignment);
    inst->setOrdering(AtomicOrdering::Acquire);
    return inst;
    
}

StoreInst * CBuilder::CreateAtomicStoreRelease(Value * val, Value * ptr) {
    const auto alignment = ptr->getType()->getPointerElementType()->getPrimitiveSizeInBits() / 8;
    StoreInst * inst = CreateAlignedStore(val, ptr, alignment);
    inst->setOrdering(AtomicOrdering::Release);
    return inst;
}

PointerType * CBuilder::getFILEptrTy() {
    if (mFILEtype == nullptr) {
        mFILEtype = StructType::create(getContext(), "struct._IO_FILE");
    }
    return mFILEtype->getPointerTo();
}

Value * CBuilder::CreateFOpenCall(Value * filename, Value * mode) {
    Function * fOpenFunc = mMod->getFunction("fopen");
    if (fOpenFunc == nullptr) {
        FunctionType * fty = FunctionType::get(getFILEptrTy(), {getInt8Ty()->getPointerTo(), getInt8Ty()->getPointerTo()}, false);
        fOpenFunc = Function::Create(fty, Function::ExternalLinkage, "fopen", mMod);
        fOpenFunc->setCallingConv(CallingConv::C);
    }
    return CreateCall(fOpenFunc, {filename, mode});
}

Value * CBuilder::CreateFReadCall(Value * ptr, Value * size, Value * nitems, Value * stream) {
    Function * fReadFunc = mMod->getFunction("fread");
    if (fReadFunc == nullptr) {
        FunctionType * fty = FunctionType::get(getSizeTy(), {getVoidPtrTy(), getSizeTy(), getSizeTy(), getFILEptrTy()}, false);
        fReadFunc = Function::Create(fty, Function::ExternalLinkage, "fread", mMod);
        fReadFunc->setCallingConv(CallingConv::C);
    }
    return CreateCall(fReadFunc, {ptr, size, nitems, stream});
}

Value * CBuilder::CreateFWriteCall(Value * ptr, Value * size, Value * nitems, Value * stream) {
    Function * fWriteFunc = mMod->getFunction("fwrite");
    if (fWriteFunc == nullptr) {
        FunctionType * fty = FunctionType::get(getSizeTy(), {getVoidPtrTy(), getSizeTy(), getSizeTy(), getFILEptrTy()}, false);
        fWriteFunc = Function::Create(fty, Function::ExternalLinkage, "fwrite", mMod);
        fWriteFunc->setCallingConv(CallingConv::C);
    }
    return CreateCall(fWriteFunc, {ptr, size, nitems, stream});
}

Value * CBuilder::CreateFCloseCall(Value * stream) {
    Function * fCloseFunc = mMod->getFunction("fclose");
    if (fCloseFunc == nullptr) {
        FunctionType * fty = FunctionType::get(getInt32Ty(), {getFILEptrTy()}, false);
        fCloseFunc = Function::Create(fty, Function::ExternalLinkage, "fclose", mMod);
        fCloseFunc->setCallingConv(CallingConv::C);
    }
    return CreateCall(fCloseFunc, {stream});
}

Value * CBuilder::CreatePThreadCreateCall(Value * thread, Value * attr, Function * start_routine, Value * arg) {
    Function * pthreadCreateFunc = mMod->getFunction("pthread_create");
    if (pthreadCreateFunc == nullptr) {
        Type * pthreadTy = getSizeTy();
        FunctionType * funVoidPtrVoidTy = FunctionType::get(getVoidTy(), {getVoidPtrTy()}, false);
        FunctionType * fty = FunctionType::get(getInt32Ty(), {pthreadTy->getPointerTo(), getVoidPtrTy(), funVoidPtrVoidTy->getPointerTo(), getVoidPtrTy()}, false);
        pthreadCreateFunc = Function::Create(fty, Function::ExternalLinkage, "pthread_create", mMod);
        pthreadCreateFunc->setCallingConv(CallingConv::C);
    }
    return CreateCall(pthreadCreateFunc, {thread, attr, start_routine, arg});
}

Value * CBuilder::CreatePThreadExitCall(Value * value_ptr) {
    Function * pthreadExitFunc = mMod->getFunction("pthread_exit");
    if (pthreadExitFunc == nullptr) {
        FunctionType * fty = FunctionType::get(getVoidTy(), {getVoidPtrTy()}, false);
        pthreadExitFunc = Function::Create(fty, Function::ExternalLinkage, "pthread_exit", mMod);
        pthreadExitFunc->addFnAttr(Attribute::NoReturn);
        pthreadExitFunc->setCallingConv(CallingConv::C);
    }
    CallInst * exitThread = CreateCall(pthreadExitFunc, {value_ptr});
    exitThread->setDoesNotReturn();
    return exitThread;
}

Value * CBuilder::CreatePThreadJoinCall(Value * thread, Value * value_ptr){
    Function * pthreadJoinFunc = mMod->getFunction("pthread_join");
    if (pthreadJoinFunc == nullptr) {
        Type * pthreadTy = getSizeTy();
        FunctionType * fty = FunctionType::get(getInt32Ty(), {pthreadTy, getVoidPtrTy()->getPointerTo()}, false);
        pthreadJoinFunc = Function::Create(fty, Function::ExternalLinkage, "pthread_join", mMod);
        pthreadJoinFunc->setCallingConv(CallingConv::C);
    }
    return CreateCall(pthreadJoinFunc, {thread, value_ptr});
}

void CBuilder::CreateAssert(Value * const assertion, StringRef failureMessage) {
    if (codegen::EnableAsserts) {
        Function * function = mMod->getFunction("__assert");
        if (LLVM_UNLIKELY(function == nullptr)) {
            auto ip = saveIP();
            FunctionType * fty = FunctionType::get(getVoidTy(), { getInt1Ty(), getInt8PtrTy(), getSizeTy() }, false);
            function = Function::Create(fty, Function::PrivateLinkage, "__assert", mMod);
            function->setDoesNotThrow();
            function->setDoesNotAlias(2);
            BasicBlock * const entry = BasicBlock::Create(getContext(), "", function);
            BasicBlock * const failure = BasicBlock::Create(getContext(), "", function);
            BasicBlock * const success = BasicBlock::Create(getContext(), "", function);
            auto arg = function->arg_begin();
            arg->setName("assertion");
            Value * e = &*arg++;
            arg->setName("msg");
            Value * msg = &*arg++;
            arg->setName("sz");
            Value * sz = &*arg;
            SetInsertPoint(entry);
            CreateCondBr(e, failure, success);
            SetInsertPoint(failure);
            Value * len = CreateAdd(sz, getSize(21));
            ConstantInt * _11 = getSize(11);
            Value * bytes = CreatePointerCast(CreateMalloc(len), getInt8PtrTy());
            CreateMemCpy(bytes, CreateGlobalStringPtr("Assertion `"), _11, 1);
            CreateMemCpy(CreateGEP(bytes, _11), msg, sz, 1);
            CreateMemCpy(CreateGEP(bytes, CreateAdd(sz, _11)), CreateGlobalStringPtr("' failed.\n"), getSize(10), 1);
            CreateWriteCall(getInt32(2), bytes, len);
            CreateExit(-1);
            CreateBr(success); // necessary to satisfy the LLVM verifier. this is not actually executed.
            SetInsertPoint(success);
            CreateRetVoid();
            restoreIP(ip);
        }
        CreateCall(function, {CreateICmpEQ(assertion, Constant::getNullValue(assertion->getType())), CreateGlobalStringPtr(failureMessage), getSize(failureMessage.size())});
    }
}

void CBuilder::CreateExit(const int exitCode) {
    Function * exit = mMod->getFunction("exit");
    if (LLVM_UNLIKELY(exit == nullptr)) {
        FunctionType * fty = FunctionType::get(getVoidTy(), {getInt32Ty()}, false);
        exit = Function::Create(fty, Function::ExternalLinkage, "exit", mMod);
        exit->setDoesNotReturn();
        exit->setDoesNotThrow();
    }
    CreateCall(exit, getInt32(exitCode));
}

BranchInst * CBuilder::CreateLikelyCondBr(Value * Cond, BasicBlock * True, BasicBlock * False, const int probability) {
    MDBuilder mdb(getContext());
    if (probability < 0 || probability > 100) {
        report_fatal_error("branch weight probability must be in [0,100]");
    }
    return CreateCondBr(Cond, True, False, mdb.createBranchWeights(probability, 100 - probability));
}

Value * CBuilder::CreateCeilLog2(Value * value) {
    IntegerType * ty = cast<IntegerType>(value->getType());
    CreateAssert(value, "CreateCeilLog2: value cannot be zero");
    Value * m = CreateCall(Intrinsic::getDeclaration(mMod, Intrinsic::ctlz, ty), {value, ConstantInt::getFalse(getContext())});
    Value * isPowOf2 = CreateICmpEQ(CreateAnd(value, CreateSub(value, ConstantInt::get(ty, 1))), ConstantInt::getNullValue(ty));
    m = CreateSub(ConstantInt::get(m->getType(), ty->getBitWidth() - 1), m);
    return CreateSelect(isPowOf2, m, CreateAdd(m, ConstantInt::get(m->getType(), 1)));
}

CBuilder::CBuilder(Module * const m, const unsigned GeneralRegisterWidthInBits, const bool SupportsIndirectBr, const unsigned CacheLineAlignmentInBytes)
: IRBuilder<>(m->getContext())
, mMod(m)
, mCacheLineAlignment(CacheLineAlignmentInBytes)
, mSizeType(getIntNTy(GeneralRegisterWidthInBits))
, mFILEtype(nullptr)
, mSupportsIndirectBr(SupportsIndirectBr) {
}
