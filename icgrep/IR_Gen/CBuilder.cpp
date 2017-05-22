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
#include <llvm/Support/raw_ostream.h>
#include <toolchain/toolchain.h>
#include <toolchain/driver.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <unistd.h>

using namespace llvm;

Value * CBuilder::CreateOpenCall(Value * filename, Value * oflag, Value * mode) {
    Module * const m = getModule();
    Function * openFn = m->getFunction("open");
    if (openFn == nullptr) {
        IntegerType * int32Ty = getInt32Ty();
        PointerType * int8PtrTy = getInt8PtrTy();
        openFn = cast<Function>(m->getOrInsertFunction("open",
                                                         int32Ty, int8PtrTy, int32Ty, int32Ty, nullptr));
    }
    return CreateCall(openFn, {filename, oflag, mode});
}

// ssize_t write(int fildes, const void *buf, size_t nbyte);
Value * CBuilder::CreateWriteCall(Value * fileDescriptor, Value * buf, Value * nbyte) {
    PointerType * voidPtrTy = getVoidPtrTy();
    Module * const m = getModule();
    Function * write = m->getFunction("write");
    if (write == nullptr) {
        IntegerType * sizeTy = getSizeTy();
        IntegerType * int32Ty = getInt32Ty();
        write = cast<Function>(m->getOrInsertFunction("write",
                                                        AttributeSet().addAttribute(getContext(), 2U, Attribute::NoAlias),
                                                        sizeTy, int32Ty, voidPtrTy, sizeTy, nullptr));
    }
    buf = CreatePointerCast(buf, voidPtrTy);
    return CreateCall(write, {fileDescriptor, buf, nbyte});
}

Value * CBuilder::CreateReadCall(Value * fileDescriptor, Value * buf, Value * nbyte) {
    PointerType * voidPtrTy = getVoidPtrTy();
    Module * const m = getModule();
    Function * readFn = m->getFunction("read");
    if (readFn == nullptr) {
        IntegerType * sizeTy = getSizeTy();
        IntegerType * int32Ty = getInt32Ty();
        readFn = cast<Function>(m->getOrInsertFunction("read",
                                                         AttributeSet().addAttribute(getContext(), 2U, Attribute::NoAlias),
                                                         sizeTy, int32Ty, voidPtrTy, sizeTy, nullptr));
    }
    buf = CreatePointerCast(buf, voidPtrTy);
    return CreateCall(readFn, {fileDescriptor, buf, nbyte});
}

Value * CBuilder::CreateCloseCall(Value * fileDescriptor) {
    Module * const m = getModule();
    Function * closeFn = m->getFunction("close");
    if (closeFn == nullptr) {
        IntegerType * int32Ty = getInt32Ty();
        FunctionType * fty = FunctionType::get(int32Ty, {int32Ty}, true);
        closeFn = Function::Create(fty, Function::ExternalLinkage, "close", m);
    }
    return CreateCall(closeFn, fileDescriptor);
}


Value * CBuilder::CreateUnlinkCall(Value * path) {
    Module * const m = getModule();
    Function * unlinkFunc = m->getFunction("unlink");
    if (unlinkFunc == nullptr) {
        FunctionType * fty = FunctionType::get(getInt32Ty(), {getInt8PtrTy()}, false);
        unlinkFunc = Function::Create(fty, Function::ExternalLinkage, "unlink", m);
        unlinkFunc->setCallingConv(CallingConv::C);
    }
    return CreateCall(unlinkFunc, path);
}

Value * CBuilder::CreateMkstempCall(Value * ftemplate) {
    Module * const m = getModule();
    Function * mkstempFn = m->getFunction("mkstemp");
    if (mkstempFn == nullptr) {
        mkstempFn = cast<Function>(m->getOrInsertFunction("mkstemp", getInt32Ty(), getInt8PtrTy(), nullptr));
    }
    return CreateCall(mkstempFn, ftemplate);
}


Value * CBuilder::CreateStrlenCall(Value * str) {
    Module * const m = getModule();
    Function * strlenFn = m->getFunction("strlen");
    if (strlenFn == nullptr) {
        strlenFn = cast<Function>(m->getOrInsertFunction("strlen", getSizeTy(), getInt8PtrTy(), nullptr));
    }
    return CreateCall(strlenFn, str);
}


Function * CBuilder::GetPrintf() {
    Module * const m = getModule();
    Function * printf = m->getFunction("printf");
    if (printf == nullptr) {
        FunctionType * fty = FunctionType::get(getInt32Ty(), {getInt8PtrTy()}, true);
        printf = Function::Create(fty, Function::ExternalLinkage, "printf", m);
        printf->addAttribute(1, Attribute::NoAlias);
    }
    return printf;
}

Function * CBuilder::GetDprintf() {
    Module * const m = getModule();
    Function * dprintf = m->getFunction("dprintf");
    if (dprintf == nullptr) {
        FunctionType * fty = FunctionType::get(getInt32Ty(), {getInt32Ty(), getInt8PtrTy()}, true);
        dprintf = Function::Create(fty, Function::ExternalLinkage, "dprintf", m);
    }
    return dprintf;
}

void CBuilder::CallPrintInt(const std::string & name, Value * const value) {
    Module * const m = getModule();
    Constant * printRegister = m->getFunction("PrintInt");
    IntegerType * int64Ty = getInt64Ty();
    if (LLVM_UNLIKELY(printRegister == nullptr)) {
        FunctionType *FT = FunctionType::get(getVoidTy(), { getInt8PtrTy(), int64Ty }, false);
        Function * function = Function::Create(FT, Function::InternalLinkage, "PrintInt", m);
        auto arg = function->arg_begin();
        std::string out = "%-40s = %" PRIx64 "\n";
        BasicBlock * entry = BasicBlock::Create(getContext(), "entry", function);
        IRBuilder<> builder(entry);
        std::vector<Value *> args;
        args.push_back(GetString(out.c_str()));
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
        num = CreatePtrToInt(value, int64Ty);
    } else {
        num = CreateZExtOrBitCast(value, int64Ty);
    }
    assert (num->getType()->isIntegerTy());
    CreateCall(printRegister, {GetString(name.c_str()), num});
}

void CBuilder::CallPrintIntToStderr(const std::string & name, Value * const value) {
    Module * const m = getModule();
    Constant * printRegister = m->getFunction("PrintIntToStderr");
    if (LLVM_UNLIKELY(printRegister == nullptr)) {
        FunctionType *FT = FunctionType::get(getVoidTy(), { PointerType::get(getInt8Ty(), 0), getSizeTy() }, false);
        Function * function = Function::Create(FT, Function::InternalLinkage, "PrintIntToStderr", m);
        auto arg = function->arg_begin();
        std::string out = "%-40s = %" PRIx64 "\n";
        BasicBlock * entry = BasicBlock::Create(getContext(), "entry", function);
        IRBuilder<> builder(entry);
        std::vector<Value *> args;
        args.push_back(getInt32(2));    // fd 2 (stderr)
        args.push_back(GetString(out.c_str()));
        Value * const name = &*(arg++);
        name->setName("name");
        args.push_back(name);
        Value * value = &*arg;
        value->setName("value");
        args.push_back(value);
        builder.CreateCall(GetDprintf(), args);
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
    CreateCall(printRegister, {GetString(name.c_str()), num});
}

void CBuilder::CallPrintMsgToStderr(const std::string & message) {
    Module * const m = getModule();
    Constant * printMsg = m->getFunction("PrintMsgToStderr");
    if (LLVM_UNLIKELY(printMsg == nullptr)) {
        FunctionType *FT = FunctionType::get(getVoidTy(), { PointerType::get(getInt8Ty(), 0) }, false);
        Function * function = Function::Create(FT, Function::InternalLinkage, "PrintMsgToStderr", m);
        auto arg = function->arg_begin();
        std::string out = "%s\n";
        BasicBlock * entry = BasicBlock::Create(getContext(), "entry", function);
        IRBuilder<> builder(entry);
        std::vector<Value *> args;
        args.push_back(getInt32(2));    // fd 2 (stderr)
        args.push_back(GetString(out.c_str()));
        Value * const msg = &*(arg++);
        msg->setName("msg");
        args.push_back(msg);
        builder.CreateCall(GetDprintf(), args);
        builder.CreateRetVoid();

        printMsg = function;
    }
    CreateCall(printMsg, {GetString(message.c_str())});
}

Value * CBuilder::CreateMalloc(Value * size) {
    Module * const m = getModule();
    IntegerType * const sizeTy = getSizeTy();    
    Function * f = m->getFunction("malloc");
    if (f == nullptr) {
        // f = LinkFunction("malloc", &malloc);
        PointerType * const voidPtrTy = getVoidPtrTy();
        FunctionType * fty = FunctionType::get(voidPtrTy, {sizeTy}, false);
        f = Function::Create(fty, Function::ExternalLinkage, "malloc", m);
        f->setCallingConv(CallingConv::C);
        f->setDoesNotAlias(0);
    }
    size = CreateZExtOrTrunc(size, sizeTy);
    CallInst * const ptr = CreateCall(f, size);
    ptr->setTailCall();
    CreateAssert(ptr, "CreateMalloc: returned null pointer");
    return ptr;
}

Value * CBuilder::CreateAlignedMalloc(Value * size, const unsigned alignment) {
    if (LLVM_UNLIKELY((alignment & (alignment - 1)) != 0)) {
        report_fatal_error("CreateAlignedMalloc: alignment must be a power of 2");
    }
    Module * const m = getModule();
    IntegerType * const sizeTy = getSizeTy();
    PointerType * const voidPtrTy = getVoidPtrTy();
    #ifdef STDLIB_HAS_ALIGNED_ALLOC
    Function * f = m->getFunction("aligned_alloc");
    if (LLVM_UNLIKELY(f == nullptr)) {
        FunctionType * const fty = FunctionType::get(voidPtrTy, {sizeTy, sizeTy}, false);
        f = Function::Create(fty, Function::ExternalLinkage, "aligned_alloc", m);
        f->setCallingConv(CallingConv::C);
        f->setDoesNotAlias(0);
    }
    #else
    Function * f = m->getFunction("posix_memalign");
    if (LLVM_UNLIKELY(f == nullptr)) {
        FunctionType * const fty = FunctionType::get(getInt32Ty(), {voidPtrTy->getPointerTo(), sizeTy, sizeTy}, false);
        f = Function::Create(fty, Function::ExternalLinkage, "posix_memalign", m);
        f->setCallingConv(CallingConv::C);
        f->setDoesNotAlias(0);
    }
    #endif
    size = CreateZExtOrTrunc(size, sizeTy);
    ConstantInt * const align = ConstantInt::get(sizeTy, alignment);
    if (codegen::EnableAsserts) {
        CreateAssert(CreateICmpEQ(CreateURem(size, align), ConstantInt::get(sizeTy, 0)),
                     "CreateAlignedMalloc: size must be an integral multiple of alignment.");
    }
    #ifdef STDLIB_HAS_ALIGNED_ALLOC
    Value * const ptr = CreateCall(f, {align, size});
    #else
    Value * ptr = CreateAlloca(voidPtrTy);
    Value * success = CreateCall(f, {ptr, align, size});
    if (codegen::EnableAsserts) {
        CreateAssert(CreateICmpEQ(success, getInt32(0)), "CreateAlignedMalloc: posix_memalign reported bad allocation");
    }
    ptr = CreateLoad(ptr);
    #endif
    CreateAssert(ptr, "CreateAlignedMalloc: returned null pointer.");
    return ptr;
}

Value * CBuilder::CreateAnonymousMMap(Value * size) {
    PointerType * const voidPtrTy = getVoidPtrTy();
    IntegerType * const intTy = getInt32Ty();
    IntegerType * const sizeTy = getSizeTy();
    size = CreateZExtOrTrunc(size, sizeTy);
    ConstantInt * const prot =  ConstantInt::get(intTy, PROT_READ | PROT_WRITE);
    ConstantInt * const flags =  ConstantInt::get(intTy, MAP_PRIVATE | MAP_ANON);
    ConstantInt * const fd =  ConstantInt::get(intTy, -1);
    Constant * const offset = ConstantInt::get(sizeTy, 0);
    return CreateMMap(ConstantPointerNull::getNullValue(voidPtrTy), size, prot, flags, fd, offset);
}

Value * CBuilder::CreateFileSourceMMap(Value * fd, Value * size) {
    PointerType * const voidPtrTy = getVoidPtrTy();
    IntegerType * const intTy = getInt32Ty();
    fd = CreateZExtOrTrunc(fd, intTy);
    IntegerType * const sizeTy = getSizeTy();
    size = CreateZExtOrTrunc(size, sizeTy);
    ConstantInt * const prot =  ConstantInt::get(intTy, PROT_READ);
    ConstantInt * const flags =  ConstantInt::get(intTy, MAP_PRIVATE);
    Constant * const offset = ConstantInt::get(sizeTy, 0);       
    return CreateMMap(ConstantPointerNull::getNullValue(voidPtrTy), size, prot, flags, fd, offset);
}

Value * CBuilder::CreateMMap(Value * const addr, Value * size, Value * const prot, Value * const flags, Value * const fd, Value * const offset) {
    Module * const m = getModule();
    Function * fMMap = m->getFunction("mmap");
    if (LLVM_UNLIKELY(fMMap == nullptr)) {
        PointerType * const voidPtrTy = getVoidPtrTy();
        IntegerType * const intTy = getInt32Ty();
        IntegerType * const sizeTy = getSizeTy();
        FunctionType * fty = FunctionType::get(voidPtrTy, {voidPtrTy, sizeTy, intTy, intTy, intTy, sizeTy}, false);
        fMMap = Function::Create(fty, Function::ExternalLinkage, "mmap", m);
    }
    Value * ptr = CreateCall(fMMap, {addr, size, prot, flags, fd, offset});
    if (codegen::EnableAsserts) {
        CreateAssert(CheckMMapSuccess(ptr), "CreateMMap: mmap failed to allocate memory");
    }
    return ptr;
}

/**
 * @brief CBuilder::CreateMAdvise
 * @param addr
 * @param length
 * @param advice
 *
 * Note: this funcition can fail if a kernel resource was temporarily unavailable. Test if this is more than a simple hint and handle accordingly.
 *
 *  ADVICE_NORMAL
 *      No special treatment. This is the default.
 *  ADVICE_RANDOM
 *      Expect page references in random order. (Hence, read ahead may be less useful than normally.)
 *  ADVICE_SEQUENTIAL
 *      Expect page references in sequential order. (Hence, pages in the given range can be aggressively read ahead, and may be freed
 *      soon after they are accessed.)
 *  ADVICE_WILLNEED
 *      Expect access in the near future. (Hence, it might be a good idea to read some pages ahead.)
 *  ADVICE_DONTNEED
 *      Do not expect access in the near future. (For the time being, the application is finished with the given range, so the kernel
 *      can free resources associated with it.) Subsequent accesses of pages in this range will succeed, but will result either in
 *      reloading of the memory contents from the underlying mapped file (see mmap(2)) or zero-fill-on-demand pages for mappings
 *      without an underlying file.
 *
 * @return Value indicating success (0) or failure (-1).
 */
Value * CBuilder::CreateMAdvise(Value * addr, Value * length, Advice advice) {
    Triple T(mTriple);
    Value * result = nullptr;
    if (T.isOSLinux() || T.isOSDarwin()) {
        Module * const m = getModule();
        IntegerType * const intTy = getInt32Ty();
        IntegerType * const sizeTy = getSizeTy();
        PointerType * const voidPtrTy = getVoidPtrTy();
        Function * MAdviseFunc = m->getFunction("madvise");
        if (LLVM_UNLIKELY(MAdviseFunc == nullptr)) {
            FunctionType * fty = FunctionType::get(intTy, {voidPtrTy, sizeTy, intTy}, false);
            MAdviseFunc = Function::Create(fty, Function::ExternalLinkage, "madvise", m);
        }
        addr = CreatePointerCast(addr, voidPtrTy);
        length = CreateZExtOrTrunc(length, sizeTy);
        int madv_flag = 0;
        switch (advice) {
            case Advice::ADVICE_NORMAL:
                madv_flag = MADV_NORMAL; break;
            case Advice::ADVICE_RANDOM:
                madv_flag = MADV_RANDOM; break;
            case Advice::ADVICE_SEQUENTIAL:
                madv_flag = MADV_SEQUENTIAL; break;
            case Advice::ADVICE_WILLNEED:
                madv_flag = MADV_WILLNEED; break;
            case Advice::ADVICE_DONTNEED:
                madv_flag = MADV_DONTNEED; break;
        }
        result = CreateCall(MAdviseFunc, {addr, length, ConstantInt::get(intTy, madv_flag)});
    }
    return result;
}

Value * CBuilder::CheckMMapSuccess(Value * const addr) {
    Module * const m = getModule();
    DataLayout DL(m);
    IntegerType * const intTy = getIntPtrTy(DL);
    return CreateICmpNE(CreatePtrToInt(addr, intTy), ConstantInt::getAllOnesValue(intTy)); // MAP_FAILED = -1
}

#ifndef MREMAP_MAYMOVE
#define MREMAP_MAYMOVE	1
#endif

Value * CBuilder::CreateMRemap(Value * addr, Value * oldSize, Value * newSize) {
    Triple T(mTriple);
    Value * ptr = nullptr;
    if (T.isOSLinux()) {
        Module * const m = getModule();
        DataLayout DL(m);
        PointerType * const voidPtrTy = getVoidPtrTy();
        IntegerType * const sizeTy = getSizeTy();
        IntegerType * const intTy = getIntPtrTy(DL);
        Function * fMRemap = m->getFunction("mremap");
        if (LLVM_UNLIKELY(fMRemap == nullptr)) {
            FunctionType * fty = FunctionType::get(voidPtrTy, {voidPtrTy, sizeTy, sizeTy, intTy}, false);
            fMRemap = Function::Create(fty, Function::ExternalLinkage, "mremap", m);
        }
        addr = CreatePointerCast(addr, voidPtrTy);
        oldSize = CreateZExtOrTrunc(oldSize, sizeTy);
        newSize = CreateZExtOrTrunc(newSize, sizeTy);
        ConstantInt * const flags = ConstantInt::get(intTy, MREMAP_MAYMOVE);
        ptr = CreateCall(fMRemap, {addr, oldSize, newSize, flags});
        if (codegen::EnableAsserts) {
            CreateAssert(CheckMMapSuccess(ptr), "CreateMRemap: mremap failed to allocate memory");
        }
    } else { // no OS mremap support
        ptr = CreateAnonymousMMap(newSize);
        CreateMemCpy(ptr, addr, oldSize, getpagesize());
        CreateMUnmap(addr, oldSize);
    }
    return ptr;
}

Value * CBuilder::CreateMUnmap(Value * addr, Value * len) {
    IntegerType * const sizeTy = getSizeTy();
    PointerType * const voidPtrTy = getVoidPtrTy();
    Module * const m = getModule();
    Function * munmapFunc = m->getFunction("munmap");
    if (LLVM_UNLIKELY(munmapFunc == nullptr)) {
        FunctionType * const fty = FunctionType::get(sizeTy, {voidPtrTy, sizeTy}, false);
        munmapFunc = Function::Create(fty, Function::ExternalLinkage, "munmap", m);
    }
    len = CreateZExtOrTrunc(len, sizeTy);
    if (codegen::EnableAsserts) {
        DataLayout DL(getModule());
        IntegerType * const intPtrTy = getIntPtrTy(DL);
        CreateAssert(len, "CreateMUnmap: length cannot be 0");
        Value * const addrValue = CreatePtrToInt(addr, intPtrTy);
        Value * const pageOffset = CreateURem(addrValue, ConstantInt::get(intPtrTy, getpagesize()));
        CreateAssert(CreateICmpEQ(pageOffset, ConstantInt::getNullValue(intPtrTy)), "CreateMUnmap: addr must be a multiple of the page size");
        Value * const boundCheck = CreateICmpULT(addrValue, CreateSub(ConstantInt::getAllOnesValue(intPtrTy), CreateZExtOrTrunc(len, intPtrTy)));
        CreateAssert(boundCheck, "CreateMUnmap: addresses in [addr, addr+len) are outside the valid address space range");
    }
    addr = CreatePointerCast(addr, voidPtrTy);
    return CreateCall(munmapFunc, {addr, len});
}

void CBuilder::CreateFree(Value * ptr) {
    assert (ptr->getType()->isPointerTy());
    Module * const m = getModule();
    Type * const voidPtrTy =  getVoidPtrTy();
    Function * f = m->getFunction("free");
    if (f == nullptr) {
        FunctionType * fty = FunctionType::get(getVoidTy(), {voidPtrTy}, false);
        f = Function::Create(fty, Function::ExternalLinkage, "free", m);
        f->setCallingConv(CallingConv::C);
        // f = LinkFunction("free", &std::free);
    }
    ptr = CreatePointerCast(ptr, voidPtrTy);
    CreateCall(f, ptr)->setTailCall();
}

Value * CBuilder::CreateRealloc(Value * const ptr, Value * size) {
    Module * const m = getModule();
    IntegerType * const sizeTy = getSizeTy();
    PointerType * const voidPtrTy = getVoidPtrTy();
    Function * f = m->getFunction("realloc");
    if (f == nullptr) {
        FunctionType * fty = FunctionType::get(voidPtrTy, {voidPtrTy, sizeTy}, false);
        f = Function::Create(fty, Function::ExternalLinkage, "realloc", m);
        f->setCallingConv(CallingConv::C);
        f->setDoesNotAlias(0);
        f->setDoesNotAlias(1);
        // f = LinkFunction("realloc", &realloc);
    }
    Value * const addr = CreatePointerCast(ptr, voidPtrTy);
    size = CreateZExtOrTrunc(size, sizeTy);
    CallInst * const ci = CreateCall(f, {addr, size});
    ci->setTailCall();
    return CreatePointerCast(ci, ptr->getType());
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
    Module * const m = getModule();
    Function * fOpenFunc = m->getFunction("fopen");
    if (fOpenFunc == nullptr) {
        FunctionType * fty = FunctionType::get(getFILEptrTy(), {getInt8Ty()->getPointerTo(), getInt8Ty()->getPointerTo()}, false);
        fOpenFunc = Function::Create(fty, Function::ExternalLinkage, "fopen", m);
        fOpenFunc->setCallingConv(CallingConv::C);
    }
    return CreateCall(fOpenFunc, {filename, mode});
}

Value * CBuilder::CreateFReadCall(Value * ptr, Value * size, Value * nitems, Value * stream) {
    Module * const m = getModule();
    Function * fReadFunc = m->getFunction("fread");
    PointerType * const voidPtrTy = getVoidPtrTy();
    if (fReadFunc == nullptr) {
        IntegerType * const sizeTy = getSizeTy();
        FunctionType * fty = FunctionType::get(sizeTy, {voidPtrTy, sizeTy, sizeTy, getFILEptrTy()}, false);
        fReadFunc = Function::Create(fty, Function::ExternalLinkage, "fread", m);
        fReadFunc->setCallingConv(CallingConv::C);
    }
    ptr = CreatePointerCast(ptr, voidPtrTy);
    return CreateCall(fReadFunc, {ptr, size, nitems, stream});
}

Value * CBuilder::CreateFWriteCall(Value * ptr, Value * size, Value * nitems, Value * stream) {
    Module * const m = getModule();
    Function * fWriteFunc = m->getFunction("fwrite");
    PointerType * const voidPtrTy = getVoidPtrTy();
    if (fWriteFunc == nullptr) {
        IntegerType * const sizeTy = getSizeTy();
        FunctionType * fty = FunctionType::get(sizeTy, {voidPtrTy, sizeTy, sizeTy, getFILEptrTy()}, false);
        fWriteFunc = Function::Create(fty, Function::ExternalLinkage, "fwrite", m);
        fWriteFunc->setCallingConv(CallingConv::C);
    }
    ptr = CreatePointerCast(ptr, voidPtrTy);
    return CreateCall(fWriteFunc, {ptr, size, nitems, stream});
}

Value * CBuilder::CreateFCloseCall(Value * stream) {
    Module * const m = getModule();
    Function * fCloseFunc = m->getFunction("fclose");
    if (fCloseFunc == nullptr) {
        FunctionType * fty = FunctionType::get(getInt32Ty(), {getFILEptrTy()}, false);
        fCloseFunc = Function::Create(fty, Function::ExternalLinkage, "fclose", m);
        fCloseFunc->setCallingConv(CallingConv::C);
    }
    return CreateCall(fCloseFunc, {stream});
}

Value * CBuilder::CreateRenameCall(Value * oldName, Value * newName) {
    Module * const m = getModule();
    Function * renameFunc = m->getFunction("rename");
    if (renameFunc == nullptr) {
        FunctionType * fty = FunctionType::get(getInt32Ty(), {getInt8PtrTy(), getInt8PtrTy()}, false);
        renameFunc = Function::Create(fty, Function::ExternalLinkage, "rename", m);
        renameFunc->setCallingConv(CallingConv::C);
    }
    return CreateCall(renameFunc, {oldName, newName});
}

Value * CBuilder::CreateRemoveCall(Value * path) {
    Module * const m = getModule();
    Function * removeFunc = m->getFunction("remove");
    if (removeFunc == nullptr) {
        FunctionType * fty = FunctionType::get(getInt32Ty(), {getInt8PtrTy()}, false);
        removeFunc = Function::Create(fty, Function::ExternalLinkage, "remove", m);
        removeFunc->setCallingConv(CallingConv::C);
    }
    return CreateCall(removeFunc, {path});
}

Value * CBuilder::CreatePThreadCreateCall(Value * thread, Value * attr, Function * start_routine, Value * arg) {
    Module * const m = getModule();
    Type * const voidPtrTy = getVoidPtrTy();
    Function * pthreadCreateFunc = m->getFunction("pthread_create");
    if (pthreadCreateFunc == nullptr) {
        Type * pthreadTy = getSizeTy();
        FunctionType * funVoidPtrVoidTy = FunctionType::get(getVoidTy(), {getVoidPtrTy()}, false);
        FunctionType * fty = FunctionType::get(getInt32Ty(), {pthreadTy->getPointerTo(), voidPtrTy, funVoidPtrVoidTy->getPointerTo(), voidPtrTy}, false);
        pthreadCreateFunc = Function::Create(fty, Function::ExternalLinkage, "pthread_create", m);
        pthreadCreateFunc->setCallingConv(CallingConv::C);
    }
    return CreateCall(pthreadCreateFunc, {thread, attr, start_routine, CreatePointerCast(arg, voidPtrTy)});
}

Value * CBuilder::CreatePThreadExitCall(Value * value_ptr) {
    Module * const m = getModule();
    Function * pthreadExitFunc = m->getFunction("pthread_exit");
    if (pthreadExitFunc == nullptr) {
        FunctionType * fty = FunctionType::get(getVoidTy(), {getVoidPtrTy()}, false);
        pthreadExitFunc = Function::Create(fty, Function::ExternalLinkage, "pthread_exit", m);
        pthreadExitFunc->addFnAttr(Attribute::NoReturn);
        pthreadExitFunc->setCallingConv(CallingConv::C);
    }
    CallInst * exitThread = CreateCall(pthreadExitFunc, {value_ptr});
    exitThread->setDoesNotReturn();
    return exitThread;
}

Value * CBuilder::CreatePThreadJoinCall(Value * thread, Value * value_ptr){
    Module * const m = getModule();
    Function * pthreadJoinFunc = m->getFunction("pthread_join");
    if (pthreadJoinFunc == nullptr) {
        Type * pthreadTy = getSizeTy();
        FunctionType * fty = FunctionType::get(getInt32Ty(), {pthreadTy, getVoidPtrTy()->getPointerTo()}, false);
        pthreadJoinFunc = Function::Create(fty, Function::ExternalLinkage, "pthread_join", m);
        pthreadJoinFunc->setCallingConv(CallingConv::C);
    }
    return CreateCall(pthreadJoinFunc, {thread, value_ptr});
}

void CBuilder::CreateAssert(Value * const assertion, StringRef failureMessage) {    
    if (codegen::EnableAsserts) {
        Module * const m = getModule();
        Function * function = m->getFunction("__assert");
        if (LLVM_UNLIKELY(function == nullptr)) {
            auto ip = saveIP();
            FunctionType * fty = FunctionType::get(getVoidTy(), { getInt1Ty(), getInt8PtrTy(), getSizeTy() }, false);
            function = Function::Create(fty, Function::PrivateLinkage, "__assert", m);
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
            CreateMemCpy(bytes, GetString("Assertion `"), _11, 1);
            CreateMemCpy(CreateGEP(bytes, _11), msg, sz, 1);
            CreateMemCpy(CreateGEP(bytes, CreateAdd(sz, _11)), GetString("' failed.\n"), getSize(10), 1);
            CreateWriteCall(getInt32(2), bytes, len);


            CreateExit(-1);
            CreateBr(success); // necessary to satisfy the LLVM verifier. this is not actually executed.
            SetInsertPoint(success);
            CreateRetVoid();
            restoreIP(ip);
        }
        CreateCall(function, {CreateICmpEQ(assertion, Constant::getNullValue(assertion->getType())), GetString(failureMessage), getSize(failureMessage.size())});
    }
}

void CBuilder::CreateExit(const int exitCode) {
    Module * const m = getModule();
    Function * exit = m->getFunction("exit");
    if (LLVM_UNLIKELY(exit == nullptr)) {
        FunctionType * fty = FunctionType::get(getVoidTy(), {getInt32Ty()}, false);
        exit = Function::Create(fty, Function::ExternalLinkage, "exit", m);
        exit->setDoesNotReturn();
        exit->setDoesNotThrow();
    }
    CreateCall(exit, getInt32(exitCode));
}

llvm::BasicBlock * CBuilder::CreateBasicBlock(std::string && name) {
    return BasicBlock::Create(getContext(), name, GetInsertBlock()->getParent());
}

BranchInst * CBuilder::CreateLikelyCondBr(Value * Cond, BasicBlock * True, BasicBlock * False, const int probability) {
    MDBuilder mdb(getContext());
    if (probability < 0 || probability > 100) {
        report_fatal_error("branch weight probability must be in [0,100]");
    }
    return CreateCondBr(Cond, True, False, mdb.createBranchWeights(probability, 100 - probability));
}

Value * CBuilder::CreatePopcount(Value * bits) {
    Value * ctpopFunc = Intrinsic::getDeclaration(getModule(), Intrinsic::ctpop, bits->getType());
    return CreateCall(ctpopFunc, bits);
}

Value * CBuilder::CreateCountForwardZeroes(Value * value) {
    Value * cttzFunc = Intrinsic::getDeclaration(getModule(), Intrinsic::cttz, value->getType());
    return CreateCall(cttzFunc, {value, ConstantInt::getFalse(getContext())});
}

Value * CBuilder::CreateCountReverseZeroes(Value * value) {
    Value * ctlzFunc = Intrinsic::getDeclaration(getModule(), Intrinsic::ctlz, value->getType());
    return CreateCall(ctlzFunc, {value, ConstantInt::getFalse(getContext())});
}

Value * CBuilder::CreateResetLowestBit(Value * bits) {
    return CreateAnd(bits, CreateSub(bits, ConstantInt::get(bits->getType(), 1)));
}

Value * CBuilder::CreateIsolateLowestBit(Value * bits) {
    return CreateAnd(bits, CreateNeg(bits));
}

Value * CBuilder::CreateMaskToLowestBitInclusive(Value * bits) {
    return CreateXor(bits, CreateSub(bits, ConstantInt::get(bits->getType(), 1)));
}

Value * CBuilder::CreateMaskToLowestBitExclusive(Value * bits) {
    return CreateAnd(CreateSub(bits, ConstantInt::get(bits->getType(), 1)), CreateNot(bits));
}

Value * CBuilder::CreateExtractBitField(llvm::Value * bits, Value * start, Value * length) {
    Constant * One = ConstantInt::get(bits->getType(), 1);
    return CreateAnd(CreateLShr(bits, start), CreateSub(CreateShl(One, length), One));
}

Value * CBuilder::CreateCeilLog2(Value * value) {
    IntegerType * ty = cast<IntegerType>(value->getType());
    CreateAssert(value, "CreateCeilLog2: value cannot be zero");
    Value * m = CreateCountReverseZeroes(CreateSub(value, ConstantInt::get(ty, 1)));
    return CreateSub(ConstantInt::get(m->getType(), ty->getBitWidth() - 1), m);
}

Value * CBuilder::GetString(StringRef Str) {
    Module * const m = getModule();
    Value * ptr = m->getGlobalVariable(Str, true);
    if (ptr == nullptr) {
        ptr = CreateGlobalString(Str, Str);
    }
    Value * zero = getInt32(0);
    return CreateInBoundsGEP(ptr, { zero, zero });
}

Value * CBuilder::CreateReadCycleCounter() {
    Module * const m = getModule();
    Value * cycleCountFunc = Intrinsic::getDeclaration(m, Intrinsic::readcyclecounter);
    return CreateCall(cycleCountFunc, std::vector<Value *>({}));
}

Function * CBuilder::LinkFunction(llvm::StringRef name, FunctionType * type, void * functionPtr) const {
    assert (mDriver);
    return mDriver->addLinkFunction(getModule(), name, type, functionPtr);
}

CBuilder::CBuilder(llvm::LLVMContext & C, const unsigned GeneralRegisterWidthInBits)
: IRBuilder<>(C)
, mCacheLineAlignment(64)
, mSizeType(getIntNTy(GeneralRegisterWidthInBits))
, mFILEtype(nullptr)
, mDriver(nullptr) {

}
