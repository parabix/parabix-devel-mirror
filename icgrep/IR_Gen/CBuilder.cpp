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
#include <kernels/toolchain.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <errno.h>
#include <llvm/ADT/Triple.h>

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


Value * CBuilder::CreateUnlinkCall(Value * path) {
    Function * unlinkFunc = mMod->getFunction("unlink");
    if (unlinkFunc == nullptr) {
        FunctionType * fty = FunctionType::get(getInt32Ty(), {getInt8PtrTy()}, false);
        unlinkFunc = Function::Create(fty, Function::ExternalLinkage, "unlink", mMod);
        unlinkFunc->setCallingConv(CallingConv::C);
    }
    return CreateCall(unlinkFunc, {path});
}

Value * CBuilder::CreateMkstempCall(Value * ftemplate) {
    Function * mkstempFn = mMod->getFunction("mkstemp");
    if (mkstempFn == nullptr) {
        mkstempFn = cast<Function>(mMod->getOrInsertFunction("mkstemp", getInt32Ty(), getInt8PtrTy(), nullptr));
    }
    return CreateCall(mkstempFn, {ftemplate});
}


Value * CBuilder::CreateStrlenCall(Value * str) {
    Function * strlenFn = mMod->getFunction("strlen");
    if (strlenFn == nullptr) {
        strlenFn = cast<Function>(mMod->getOrInsertFunction("strlen", getSizeTy(), getInt8PtrTy(), nullptr));
    }
    return CreateCall(strlenFn, {str});
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
    IntegerType * int64Ty = getInt64Ty();
    if (LLVM_UNLIKELY(printRegister == nullptr)) {
        FunctionType *FT = FunctionType::get(getVoidTy(), { getInt8PtrTy(), int64Ty }, false);
        Function * function = Function::Create(FT, Function::InternalLinkage, "PrintInt", mMod);
        auto arg = function->arg_begin();
        std::string out = "%-40s = %" PRIx64 "\n";
        BasicBlock * entry = BasicBlock::Create(mMod->getContext(), "entry", function);
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

Value * CBuilder::CreateFileSourceMMap(Value * const fd, Value * size) {
    PointerType * const voidPtrTy = getVoidPtrTy();
    IntegerType * const intTy = getInt32Ty();
    IntegerType * const sizeTy = getSizeTy();
    size = CreateZExtOrTrunc(size, sizeTy);
    ConstantInt * const prot =  ConstantInt::get(intTy, PROT_READ);
    ConstantInt * const flags =  ConstantInt::get(intTy, MAP_PRIVATE);
    Constant * const offset = ConstantInt::get(sizeTy, 0);
    return CreateMMap(ConstantPointerNull::getNullValue(voidPtrTy), size, prot, flags, fd, offset);
}

Value * CBuilder::CreateMMap(Value * const addr, Value * size, Value * const prot, Value * const flags, Value * const fd, Value * const offset) {
    Function * fMMap = mMod->getFunction("mmap");
    if (LLVM_UNLIKELY(fMMap == nullptr)) {
        PointerType * const voidPtrTy = getVoidPtrTy();
        IntegerType * const intTy = getInt32Ty();
        IntegerType * const sizeTy = getSizeTy();
        FunctionType * fty = FunctionType::get(voidPtrTy, {voidPtrTy, sizeTy, intTy, intTy, intTy, sizeTy}, false);
        fMMap = Function::Create(fty, Function::ExternalLinkage, "mmap", mMod);
    }
    Value * ptr = CreateCall(fMMap, {addr, size, prot, flags, fd, offset});
    if (codegen::EnableAsserts) {
        CreateAssert(CheckMMapSuccess(ptr), "CreateMMap: mmap failed to allocate memory");
    }
    return ptr;
}

/*
    MADV_NORMAL
        No special treatment. This is the default.
    MADV_RANDOM
        Expect page references in random order. (Hence, read ahead may be less useful than normally.)
    MADV_SEQUENTIAL
        Expect page references in sequential order. (Hence, pages in the given range can be aggressively read ahead, and may be freed
        soon after they are accessed.)
    MADV_WILLNEED
        Expect access in the near future. (Hence, it might be a good idea to read some pages ahead.)
    MADV_DONTNEED
        Do not expect access in the near future. (For the time being, the application is finished with the given range, so the kernel
        can free resources associated with it.) Subsequent accesses of pages in this range will succeed, but will result either in
        reloading of the memory contents from the underlying mapped file (see mmap(2)) or zero-fill-on-demand pages for mappings
        without an underlying file.
*/

Value * CBuilder::CreateMMapAdvise(Value * addr, Value * length, std::initializer_list<MADV> advice) {
    Triple T(mMod->getTargetTriple());
    Value * result = nullptr;
    if (T.isOSLinux()) {
        DataLayout DL(mMod);
        IntegerType * const intTy = getIntPtrTy(DL);
        IntegerType * const sizeTy = getSizeTy();
        PointerType * const voidPtrTy = getVoidPtrTy();
        Function * MAdviseFunc = mMod->getFunction("madvise");
        if (LLVM_UNLIKELY(MAdviseFunc == nullptr)) {
            FunctionType * fty = FunctionType::get(intTy, {voidPtrTy, sizeTy, intTy}, false);
            MAdviseFunc = Function::Create(fty, Function::ExternalLinkage, "madvise", mMod);
        }
        addr = CreatePointerCast(addr, voidPtrTy);
        length = CreateZExtOrTrunc(length, sizeTy);
        int adviceFlags = 0;
        for (const MADV adv : advice) {
            switch (adv) {
                case MADV::NORMAL: adviceFlags |= MADV_NORMAL; break;
                case MADV::RANDOM: adviceFlags |= MADV_RANDOM; break;
                case MADV::SEQUENTIAL: adviceFlags |= MADV_SEQUENTIAL; break;
                case MADV::DONTNEED: adviceFlags |= MADV_DONTNEED; break;
                case MADV::WILLNEED: adviceFlags |= MADV_WILLNEED; break;
//                case MADV::REMOVE: adviceFlags |= MADV_REMOVE; break;
//                case MADV::DONTFORK: adviceFlags |= MADV_DONTFORK; break;
//                case MADV::DOFORK: adviceFlags |= MADV_DOFORK; break;
//                case MADV::HWPOISON: adviceFlags |= MADV_HWPOISON; break;
//                case MADV::MERGEABLE: adviceFlags |= MADV_MERGEABLE; break;
//                case MADV::UNMERGEABLE: adviceFlags |= MADV_UNMERGEABLE; break;
//                case MADV::HUGEPAGE: adviceFlags |= MADV_HUGEPAGE; break;
//                case MADV::NOHUGEPAGE: adviceFlags |= MADV_NOHUGEPAGE; break;
//                case MADV::DONTDUMP: adviceFlags |= MADV_DONTDUMP; break;
//                case MADV::DODUMP: adviceFlags |= MADV_DODUMP; break;
            }
        }
        result = CreateCall(MAdviseFunc, {addr, length, ConstantInt::get(intTy, adviceFlags)});
        if (codegen::EnableAsserts) {
            CreateAssert(CreateICmpEQ(result, ConstantInt::getNullValue(result->getType())), "CreateMMapAdvise: failed");
        }
    }
    return result;
}

Value * CBuilder::CheckMMapSuccess(Value * const addr) {
    DataLayout DL(mMod);
    IntegerType * const intTy = getIntPtrTy(DL);
    return CreateICmpNE(CreatePtrToInt(addr, intTy), ConstantInt::getAllOnesValue(intTy)); // MAP_FAILED = -1
}

#ifndef MREMAP_MAYMOVE
#define MREMAP_MAYMOVE	1
#endif

Value * CBuilder::CreateMRemap(Value * addr, Value * oldSize, Value * newSize) {
    Triple T(mMod->getTargetTriple());
    Value * ptr = nullptr;
    if (T.isOSLinux()) {
        DataLayout DL(mMod);
        PointerType * const voidPtrTy = getVoidPtrTy();
        IntegerType * const sizeTy = getSizeTy();
        IntegerType * const intTy = getIntPtrTy(DL);
        Function * fMRemap = mMod->getFunction("mremap");
        if (LLVM_UNLIKELY(fMRemap == nullptr)) {
            FunctionType * fty = FunctionType::get(voidPtrTy, {voidPtrTy, sizeTy, sizeTy, intTy}, false);
            fMRemap = Function::Create(fty, Function::ExternalLinkage, "mremap", mMod);
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

Value * CBuilder::CreateMUnmap(Value * addr, Value * size) {
    IntegerType * const sizeTy = getSizeTy();
    PointerType * const voidPtrTy = getVoidPtrTy();
    Function * fMUnmap = mMod->getFunction("munmap");
    if (LLVM_UNLIKELY(fMUnmap == nullptr)) {
        DataLayout DL(mMod);
        IntegerType * const intTy = getIntPtrTy(DL);
        FunctionType * fty = FunctionType::get(intTy, {voidPtrTy, sizeTy}, false);
        fMUnmap = Function::Create(fty, Function::ExternalLinkage, "munmap", mMod);
    }
    if (codegen::EnableAsserts) {
        Value * const pageOffset = CreateURem(CreatePtrToInt(addr, sizeTy), getSize(getpagesize()));
        CreateAssert(CreateICmpEQ(pageOffset, getSize(0)), "CreateMUnmap: addr must be a multiple of the page size");
    }
    addr = CreatePointerCast(addr, voidPtrTy);
    size = CreateZExtOrTrunc(size, sizeTy);
    CallInst * result = CreateCall(fMUnmap, {addr, size});
    if (codegen::EnableAsserts) {
        CreateAssert(CreateICmpEQ(result, ConstantInt::getNullValue(result->getType())), "CreateMUnmap: failed");
    }
    return result;
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
    return TypeBuilder<void *, true>::get(getContext());
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
    PointerType * const voidPtrTy = getVoidPtrTy();
    if (fReadFunc == nullptr) {
        IntegerType * const sizeTy = getSizeTy();
        FunctionType * fty = FunctionType::get(sizeTy, {voidPtrTy, sizeTy, sizeTy, getFILEptrTy()}, false);
        fReadFunc = Function::Create(fty, Function::ExternalLinkage, "fread", mMod);
        fReadFunc->setCallingConv(CallingConv::C);
    }
    ptr = CreatePointerCast(ptr, voidPtrTy);
    return CreateCall(fReadFunc, {ptr, size, nitems, stream});
}

Value * CBuilder::CreateFWriteCall(Value * ptr, Value * size, Value * nitems, Value * stream) {
    Function * fWriteFunc = mMod->getFunction("fwrite");
    PointerType * const voidPtrTy = getVoidPtrTy();
    if (fWriteFunc == nullptr) {
        IntegerType * const sizeTy = getSizeTy();
        FunctionType * fty = FunctionType::get(sizeTy, {voidPtrTy, sizeTy, sizeTy, getFILEptrTy()}, false);
        fWriteFunc = Function::Create(fty, Function::ExternalLinkage, "fwrite", mMod);
        fWriteFunc->setCallingConv(CallingConv::C);
    }
    ptr = CreatePointerCast(ptr, voidPtrTy);
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

Value * CBuilder::CreateRenameCall(Value * oldName, Value * newName) {
    Function * renameFunc = mMod->getFunction("rename");
    if (renameFunc == nullptr) {
        FunctionType * fty = FunctionType::get(getInt32Ty(), {getInt8PtrTy(), getInt8PtrTy()}, false);
        renameFunc = Function::Create(fty, Function::ExternalLinkage, "rename", mMod);
        renameFunc->setCallingConv(CallingConv::C);
    }
    return CreateCall(renameFunc, {oldName, newName});
}

Value * CBuilder::CreateRemoveCall(Value * path) {
    Function * removeFunc = mMod->getFunction("remove");
    if (removeFunc == nullptr) {
        FunctionType * fty = FunctionType::get(getInt32Ty(), {getInt8PtrTy()}, false);
        removeFunc = Function::Create(fty, Function::ExternalLinkage, "remove", mMod);
        removeFunc->setCallingConv(CallingConv::C);
    }
    return CreateCall(removeFunc, {path});
}

Value * CBuilder::CreatePThreadCreateCall(Value * thread, Value * attr, Function * start_routine, Value * arg) {
    Type * const voidPtrTy = getVoidPtrTy();
    Function * pthreadCreateFunc = mMod->getFunction("pthread_create");
    if (pthreadCreateFunc == nullptr) {
        Type * pthreadTy = getSizeTy();
        FunctionType * funVoidPtrVoidTy = FunctionType::get(getVoidTy(), {getVoidPtrTy()}, false);
        FunctionType * fty = FunctionType::get(getInt32Ty(), {pthreadTy->getPointerTo(), voidPtrTy, funVoidPtrVoidTy->getPointerTo(), voidPtrTy}, false);
        pthreadCreateFunc = Function::Create(fty, Function::ExternalLinkage, "pthread_create", mMod);
        pthreadCreateFunc->setCallingConv(CallingConv::C);
    }
    return CreateCall(pthreadCreateFunc, {thread, attr, start_routine, CreatePointerCast(arg, voidPtrTy)});
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

inline static unsigned ceil_log2(const unsigned v) {
    assert ("log2(0) is undefined!" && v != 0);
    return 32 - __builtin_clz(v - 1);
}

Value * CBuilder::CreatePopcount(Value * bits) {
    Value * ctpopFunc = Intrinsic::getDeclaration(mMod, Intrinsic::ctpop, bits->getType());
    return CreateCall(ctpopFunc, bits);
}

Value * CBuilder::CreateCountForwardZeroes(Value * value) {
    Value * cttzFunc = Intrinsic::getDeclaration(mMod, Intrinsic::cttz, value->getType());
    return CreateCall(cttzFunc, {value, ConstantInt::getFalse(getContext())});
}

Value * CBuilder::CreateCountReverseZeroes(Value * value) {
    Value * ctlzFunc = Intrinsic::getDeclaration(mMod, Intrinsic::ctlz, value->getType());
    return CreateCall(ctlzFunc, {value, ConstantInt::getFalse(getContext())});
}

Value * CBuilder::CreateCeilLog2(Value * value) {
    IntegerType * ty = cast<IntegerType>(value->getType());
    CreateAssert(value, "CreateCeilLog2: value cannot be zero");
    Value * m = CreateCountForwardZeroes(CreateSub(value, ConstantInt::get(ty, 1)));
    return CreateSub(ConstantInt::get(m->getType(), ty->getBitWidth() - 1), m);
}

Value * CBuilder::GetString(StringRef Str) {
    Value * ptr = mMod->getGlobalVariable(Str, true);
    if (ptr == nullptr) {
        ptr = CreateGlobalString(Str, Str);
    }
    Value * zero = getInt32(0);
    return CreateInBoundsGEP(ptr, { zero, zero });
}

CBuilder::CBuilder(Module * const m, const unsigned GeneralRegisterWidthInBits, const bool SupportsIndirectBr, const unsigned CacheLineAlignmentInBytes)
: IRBuilder<>(m->getContext())
, mMod(m)
, mCacheLineAlignment(CacheLineAlignmentInBytes)
, mSizeType(getIntNTy(GeneralRegisterWidthInBits))
, mFILEtype(nullptr)
, mSupportsIndirectBr(SupportsIndirectBr) {
}
