/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "CBuilder.h"
#include <llvm/IR/Mangler.h>
#include <llvm/IR/Module.h>
#include <llvm/IR/Constants.h>
#include <llvm/IR/Intrinsics.h>
#include <llvm/IR/MDBuilder.h>
#include <llvm/ADT/DenseSet.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/Support/Format.h>
#include <toolchain/toolchain.h>
#include <toolchain/driver.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdio.h>
#include <boost/format.hpp>
#include <boost/interprocess/mapped_region.hpp>

#if defined(__i386__)
typedef uint32_t unw_word_t;
#else
typedef uint64_t unw_word_t;
#endif
#if defined(HAS_MACH_VM_TYPES)
#include <mach/vm_types.h>
extern void _thread_stack_pcs(vm_address_t *buffer, unsigned max, unsigned *nb, unsigned skip);
static_assert(sizeof(vm_address_t) == sizeof(uintptr_t), "");
#elif defined(HAS_LIBUNWIND)
#define UNW_LOCAL_ONLY
#include <libunwind.h>
static_assert(sizeof(unw_word_t) <= sizeof(uintptr_t), "");
#elif defined(HAS_EXECINFO)
#include <execinfo.h>
static_assert(sizeof(void *) == sizeof(uintptr_t), "");
#endif


#if LLVM_VERSION_INTEGER < LLVM_VERSION_CODE(5, 0, 0)
#define setReturnDoesNotAlias() setDoesNotAlias(0)
#endif

using namespace llvm;

#ifdef HAS_ADDRESS_SANITIZER
Value * checkHeapAddress(CBuilder * const b, Value * const Ptr, Value * const Size) {
    Module * const m = b->getModule();
    PointerType * const voidPtrTy = b->getVoidPtrTy();
    IntegerType * const sizeTy = b->getSizeTy();
    Function * isPoisoned = m->getFunction("__asan_region_is_poisoned");
    if (LLVM_UNLIKELY(isPoisoned == nullptr)) {
        isPoisoned = Function::Create(FunctionType::get(voidPtrTy, {voidPtrTy, sizeTy}, false), Function::ExternalLinkage, "__asan_region_is_poisoned", m);
        isPoisoned->setCallingConv(CallingConv::C);
        isPoisoned->setReturnDoesNotAlias();
        #if LLVM_VERSION_INTEGER < LLVM_VERSION_CODE(5, 0, 0)
        isPoisoned->setDoesNotAlias(1);
        #endif
    }
    Value * const addr = b->CreatePointerCast(Ptr, voidPtrTy);
    Value * check = b->CreateCall(isPoisoned, { addr, b->CreateTrunc(Size, sizeTy) });
    return b->CreateICmpEQ(check, ConstantPointerNull::get(cast<PointerType>(isPoisoned->getReturnType())));
}
#define CHECK_HEAP_ADDRESS(Ptr, Size, Name) \
if (LLVM_UNLIKELY(hasAddressSanitizer())) { \
    CreateAssert(checkHeapAddress(this, Ptr, Size), Name " was given unallocated memory address"); \
}
#else
#define CHECK_HEAP_ADDRESS(Ptr, Size, Name)
#endif

static AllocaInst * resolveStackAddress(Value * Ptr) {
    for (;;) {
        if (GetElementPtrInst * gep = dyn_cast<GetElementPtrInst>(Ptr)) {
            Ptr = gep->getPointerOperand();
        } else if (CastInst * ci = dyn_cast<CastInst>(Ptr)) {
            Ptr = ci->getOperand(0);
        } else {
            return dyn_cast<AllocaInst>(Ptr);
        }
    }
}

static inline bool notConstantZeroArraySize(const AllocaInst * const Base) {
    if (const Constant * const as = dyn_cast_or_null<Constant>(Base->getArraySize())) {
        return !as->isNullValue();
    }
    return false;
}

static Value * checkStackAddress(CBuilder * const b, Value * const Ptr, Value * const Size, AllocaInst * const Base) {
    DataLayout DL(b->getModule());
    IntegerType * const intPtrTy = cast<IntegerType>(DL.getIntPtrType(Ptr->getType()));
    Value * sz = ConstantExpr::getBitCast(ConstantExpr::getSizeOf(Base->getAllocatedType()), intPtrTy);
    if (notConstantZeroArraySize(Base)) {
        sz = b->CreateMul(sz, b->CreateZExtOrTrunc(Base->getArraySize(), intPtrTy));
    }
    Value * const p = b->CreatePtrToInt(Ptr, intPtrTy);
    Value * const s = b->CreatePtrToInt(Base, intPtrTy);
    Value * const w = b->CreateAdd(p, b->CreateZExtOrTrunc(Size, intPtrTy));
    Value * const e = b->CreateAdd(s, sz);
    return b->CreateAnd(b->CreateICmpUGE(p, s), b->CreateICmpULE(w, e));
}

#define CHECK_ADDRESS(Ptr, Size, Name) \
    CreateAssert(Ptr, Name " was given a null address"); \
    if (AllocaInst * Base = resolveStackAddress(Ptr)) { \
        CreateAssert(checkStackAddress(this, Ptr, Size, Base), Name " was given an invalid stack address"); \
    } else { \
        CHECK_HEAP_ADDRESS(Ptr, Size, Name) \
    }

Value * CBuilder::CreateURem(Value * const number, Value * const divisor, const Twine & Name) {
    if (ConstantInt * const c = dyn_cast<ConstantInt>(divisor)) {
        const auto d = c->getZExtValue();
        assert ("CreateURem divisor cannot be 0!" && d);
        if (is_power_2(d)) {
            if (LLVM_UNLIKELY(d == 1)) {
                return ConstantInt::getNullValue(number->getType());
            } else {
                return CreateAnd(number, ConstantInt::get(number->getType(), d - 1), Name);
            }
        }
    }
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        CreateAssert(divisor, "CreateURem divisor cannot be 0!");
    }
    return Insert(BinaryOperator::CreateURem(number, divisor), Name);
}

Value * CBuilder::CreateUDiv(Value * const number, Value * const divisor, const Twine & Name) {
    if (ConstantInt * c = dyn_cast<ConstantInt>(divisor)) {
        const auto d = c->getZExtValue();
        assert ("CreateUDiv divisor cannot be 0!" && d);
        if (is_power_2(d)) {
            if (d > 1) {
                return CreateLShr(number, ConstantInt::get(divisor->getType(), std::log2(d)), Name);
            } else {
                return number;
            }
        }
    }
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        CreateAssert(divisor, "CreateUDiv divisor cannot be 0!");
    }
    return Insert(BinaryOperator::CreateUDiv(number, divisor), Name);
}

Value * CBuilder::CreateCeilUDiv(Value * const number, Value * const divisor, const Twine & Name) {
    assert (number->getType() == divisor->getType());
    Type * const t = number->getType();
    Constant * const ONE = ConstantInt::get(t, 1);
    // avoid overflow with x+y-1
    return CreateAdd(CreateUDiv(CreateSub(number, ONE), divisor), ONE, Name);
}

Value * CBuilder::CreateRoundDown(Value * const number, Value * const divisor, const Twine & Name) {
    if (isa<ConstantInt>(divisor)) {
        const auto d = cast<ConstantInt>(divisor)->getZExtValue();
        if (is_power_2(d)) {
            return CreateAnd(number, ConstantExpr::getNeg(cast<ConstantInt>(divisor)));
        }
    }
    return CreateMul(CreateUDiv(number, divisor), divisor, Name);
}

Value * CBuilder::CreateRoundUp(Value * const number, Value * const divisor, const Twine & Name) {
    if (isa<ConstantInt>(divisor)) {
        const auto d = cast<ConstantInt>(divisor)->getZExtValue();
        if (is_power_2(d)) {
            Constant * const ONE = ConstantInt::get(divisor->getType(), 1);
            Constant * const toAdd = ConstantExpr::getSub(cast<ConstantInt>(divisor), ONE);
            return CreateAnd(CreateAdd(number, toAdd), ConstantExpr::getNeg(cast<ConstantInt>(divisor)));
        }
    }
    return CreateMul(CreateCeilUDiv(number, divisor), divisor, Name);
}

Value * CBuilder::CreateOpenCall(Value * filename, Value * oflag, Value * mode) {
    Module * const m = getModule();
    Function * openFn = m->getFunction("open");
    if (openFn == nullptr) {
        IntegerType * const int32Ty = getInt32Ty();
        PointerType * const int8PtrTy = getInt8PtrTy();
        FunctionType * openTy = FunctionType::get(int32Ty, {int8PtrTy, int32Ty, int32Ty}, false);
        openFn = cast<Function>(m->getOrInsertFunction("open", openTy));
    }
    return CreateCall(openFn, {filename, oflag, mode});
}

// ssize_t write(int fildes, const void *buf, size_t nbyte);
Value * CBuilder::CreateWriteCall(Value * fileDescriptor, Value * buf, Value * nbyte) {
    PointerType * const voidPtrTy = getVoidPtrTy();
    Module * const m = getModule();
    Function * write = m->getFunction("write");
    if (write == nullptr) {
        IntegerType * const sizeTy = getSizeTy();
        IntegerType * const int32Ty = getInt32Ty();
        FunctionType * writeTy = FunctionType::get(sizeTy, {int32Ty, voidPtrTy, sizeTy}, false);
#if LLVM_VERSION_INTEGER < LLVM_VERSION_CODE(5, 0, 0)
        auto atts = AttributeSet().addAttribute(getContext(), 2U, Attribute::NoAlias);
#else
        auto atts = AttributeList().addAttribute(getContext(), 2U, Attribute::NoAlias);
#endif
        write = cast<Function>(m->getOrInsertFunction("write", writeTy, atts));
    }
    buf = CreatePointerCast(buf, voidPtrTy);
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        CHECK_ADDRESS(buf, nbyte, "CreateWriteCall");
    }
    return CreateCall(write, {fileDescriptor, buf, nbyte});
}

Value * CBuilder::CreateReadCall(Value * fileDescriptor, Value * buf, Value * nbyte) {
    PointerType * const voidPtrTy = getVoidPtrTy();
    Module * const m = getModule();
    Function * readFn = m->getFunction("read");
    if (readFn == nullptr) {
        IntegerType * const sizeTy = getSizeTy();
        IntegerType * const int32Ty = getInt32Ty();
        FunctionType * readTy = FunctionType::get(sizeTy, {int32Ty, voidPtrTy, sizeTy}, false);
#if LLVM_VERSION_INTEGER < LLVM_VERSION_CODE(5, 0, 0)
        auto atts = AttributeSet().addAttribute(getContext(), 2U, Attribute::NoAlias);
#else
        auto atts = AttributeList().addAttribute(getContext(), 2U, Attribute::NoAlias);
#endif
        readFn = cast<Function>(m->getOrInsertFunction("read", readTy, atts));
    }
    buf = CreatePointerCast(buf, voidPtrTy);
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        CHECK_ADDRESS(buf, nbyte, "CreateReadCall");
    }
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

Value * CBuilder::CreatePosixFAllocate(llvm::Value * fileDescriptor, llvm::Value * offset, llvm::Value * len) {
#if defined(__APPLE__) || defined(_WIN32)
    return nullptr;
#elif defined(__linux__)
    Module * const m = getModule();
    IntegerType * sizeTy = getSizeTy();
    Function * fPosixFAllocate = m->getFunction("posix_fallocate");
    if (fPosixFAllocate == nullptr) {
        FunctionType * fty = FunctionType::get(sizeTy, {getInt32Ty(), sizeTy, sizeTy}, false);
        fPosixFAllocate = Function::Create(fty, Function::ExternalLinkage, "posix_fallocate", m);
    }
    return CreateCall(fPosixFAllocate, {fileDescriptor, offset, len});
#endif
}

Value * CBuilder::CreateFSync(Value * fileDescriptor) {
    Module * const m = getModule();
    Function * fSync = m->getFunction("fsync");
    if (fSync == nullptr) {
        IntegerType * int32Ty = getInt32Ty();
        FunctionType * fty = FunctionType::get(int32Ty, {int32Ty}, true);
        fSync = Function::Create(fty, Function::ExternalLinkage, "fsync", m);
    }
    return CreateCall(fSync, fileDescriptor);
}



Value * CBuilder::CreateMkstempCall(Value * ftemplate) {
    Module * const m = getModule();
    Function * mkstempFn = m->getFunction("mkstemp");
    if (mkstempFn == nullptr) {
        FunctionType * const fty = FunctionType::get(getInt32Ty(), {getInt8PtrTy()}, false);
        mkstempFn = Function::Create(fty, Function::ExternalLinkage, "mkstemp", m);
    }
    return CreateCall(mkstempFn, ftemplate);
}

Value * CBuilder::CreateStrlenCall(Value * str) {
    Module * const m = getModule();
    Function * strlenFn = m->getFunction("strlen");
    if (strlenFn == nullptr) {
        FunctionType * const fty = FunctionType::get(getSizeTy(), {getInt8PtrTy()}, false);
        strlenFn = Function::Create(fty, Function::ExternalLinkage, "strlen", m);
    }
    return CreateCall(strlenFn, str);
}

Function * CBuilder::GetPrintf() {
    Module * const m = getModule();
    Function * printf = m->getFunction("printf");
    if (printf == nullptr) {
        FunctionType * const fty = FunctionType::get(getInt32Ty(), {getInt8PtrTy()}, true);
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

void CBuilder::CallPrintIntCond(StringRef name, Value * const value, Value * const cond, const STD_FD fd) {
    BasicBlock * const insertBefore = GetInsertBlock()->getNextNode();
    BasicBlock* const callBlock = CreateBasicBlock("callBlock", insertBefore);
    BasicBlock* const exitBlock = CreateBasicBlock("exitBlock", insertBefore);
    CreateCondBr(cond, callBlock, exitBlock);
    SetInsertPoint(callBlock);
    CallPrintInt(name, value, fd);
    CreateBr(exitBlock);
    SetInsertPoint(exitBlock);
}

void CBuilder::CallPrintInt(StringRef name, Value * const value, const STD_FD fd) {
    Module * const m = getModule();
    Constant * printRegister = m->getFunction("print_int");
    IntegerType * const int64Ty = getInt64Ty();
    if (LLVM_UNLIKELY(printRegister == nullptr)) {
        FunctionType *FT = FunctionType::get(getVoidTy(), { getInt32Ty(), getInt8PtrTy(), int64Ty }, false);
        Function * function = Function::Create(FT, Function::InternalLinkage, "print_int", m);
        auto arg = function->arg_begin();
        BasicBlock * entry = BasicBlock::Create(getContext(), "entry", function);
        IRBuilder<> builder(entry);
        Value * const fdInt = &*(arg++);
        fdInt->setName("fd");
        Value * const name = &*(arg++);
        name->setName("name");
        Value * value = &*arg;
        value->setName("value");
        std::vector<Value *> args(4);
        args[0] = fdInt;
        args[1] = GetString("%-40s = %" PRIx64 "\n");
        args[2] = name;
        args[3] = value;
        builder.CreateCall(GetDprintf(), args);
        builder.CreateRetVoid();
        printRegister = function;
    }
    Value * num = nullptr;
    if (value->getType()->isPointerTy()) {
        num = CreatePtrToInt(value, int64Ty);
    } else {
        num = CreateZExt(value, int64Ty);
    }
    assert (num->getType()->isIntegerTy());
    CreateCall(printRegister, {getInt32(static_cast<uint32_t>(fd)), GetString(name), num});
}

Value * CBuilder::CreateMalloc(Value * size) {
    Module * const m = getModule();
    IntegerType * const sizeTy = getSizeTy();
    Function * f = m->getFunction("malloc");
    if (f == nullptr) {
        PointerType * const voidPtrTy = getVoidPtrTy();
        FunctionType * fty = FunctionType::get(voidPtrTy, {sizeTy}, false);
        f = Function::Create(fty, Function::ExternalLinkage, "malloc", m);
        f->setCallingConv(CallingConv::C);
        f->setReturnDoesNotAlias();
    }
    size = CreateZExtOrTrunc(size, sizeTy);
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        CreateAssert(size, "CreateMalloc: 0-byte malloc is implementation defined");
    }
    CallInst * const ptr = CreateCall(f, size);
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        CreateAssert(ptr, "CreateMalloc: returned null pointer");
    }
    CreateMemZero(ptr, size, 1);
    return ptr;
}

Value * CBuilder::CreateCacheAlignedMalloc(Type * const type, Value * const ArraySize, const unsigned addressSpace) {
    Value * size = ConstantExpr::getSizeOf(type);
    if (ArraySize) {
        size = CreateMul(size, CreateZExtOrTrunc(ArraySize, size->getType()));
    }
    return CreatePointerCast(CreateCacheAlignedMalloc(size), type->getPointerTo(addressSpace));
}

Value * CBuilder::CreateAlignedMalloc(Value * size, const unsigned alignment) {
    if (LLVM_UNLIKELY(!is_power_2(alignment))) {
        report_fatal_error("CreateAlignedMalloc: alignment must be a power of 2");
    }
    Module * const m = getModule();
    IntegerType * const sizeTy = getSizeTy();
    PointerType * const voidPtrTy = getVoidPtrTy();
    ConstantInt * const align = ConstantInt::get(sizeTy, alignment);
    size = CreateZExtOrTrunc(size, sizeTy);
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        CreateAssert(size, "CreateAlignedMalloc: 0-byte malloc is implementation defined");
        CreateAssertZero(CreateURem(size, align), "CreateAlignedMalloc: size must be an integral multiple of alignment.");
    }
    Value * ptr = nullptr;
    if (hasAlignedAlloc()) {
        Function * f = m->getFunction("aligned_alloc");
        if (LLVM_UNLIKELY(f == nullptr)) {
            FunctionType * const fty = FunctionType::get(voidPtrTy, {sizeTy, sizeTy}, false);
            f = Function::Create(fty, Function::ExternalLinkage, "aligned_alloc", m);
            f->setCallingConv(CallingConv::C);
            f->setReturnDoesNotAlias();
        }
        ptr = CreateCall(f, {align, size});
    } else if (hasPosixMemalign()) {
        Function * f = m->getFunction("posix_memalign");
        if (LLVM_UNLIKELY(f == nullptr)) {
            FunctionType * const fty = FunctionType::get(getInt32Ty(), {voidPtrTy->getPointerTo(), sizeTy, sizeTy}, false);
            f = Function::Create(fty, Function::ExternalLinkage, "posix_memalign", m);
            f->setCallingConv(CallingConv::C);
        }
        Value * handle = CreateAlloca(voidPtrTy);
        CallInst * success = CreateCall(f, {handle, align, size});
        if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
            CreateAssertZero(success, "CreateAlignedMalloc: posix_memalign reported bad allocation");
        }
        ptr = CreateLoad(handle);
    } else {
        report_fatal_error("stdlib.h does not contain either aligned_alloc or posix_memalign");
    }
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        CreateAssert(ptr, "CreateAlignedMalloc: returned null (out of memory?)");
    }
    CreateMemZero(ptr, size, alignment);
    return ptr;
}

inline bool CBuilder::hasAlignedAlloc() const {
    return mDriver && mDriver->hasExternalFunction("aligned_alloc");
}


inline bool CBuilder::hasPosixMemalign() const {
    return mDriver && mDriver->hasExternalFunction("posix_memalign");
}

Value * CBuilder::CreateRealloc(llvm::Type * const type, llvm::Value * const base, llvm::Value * const ArraySize) {
    Value * size = ConstantExpr::getSizeOf(type);
    if (ArraySize) {
        size = CreateMul(size, CreateZExtOrTrunc(ArraySize, size->getType()));
    }
    return CreatePointerCast(CreateRealloc(base, size), type->getPointerTo());
}

Value * CBuilder::CreateRealloc(Value * const base, Value * const size) {
    assert ("Ptr is not a pointer type" && base->getType()->isPointerTy());
    assert ("Size is not an integer" && size->getType()->isIntegerTy());
    Module * const m = getModule();
    IntegerType * const sizeTy = getSizeTy();
    PointerType * const voidPtrTy = getVoidPtrTy();
    Function * f = m->getFunction("realloc");
    if (f == nullptr) {
        FunctionType * fty = FunctionType::get(voidPtrTy, {voidPtrTy, sizeTy}, false);
        f = Function::Create(fty, Function::ExternalLinkage, "realloc", m);
        f->setCallingConv(CallingConv::C);
        f->setReturnDoesNotAlias();
        #if LLVM_VERSION_INTEGER < LLVM_VERSION_CODE(5, 0, 0)
        f->setDoesNotAlias(1);
        #endif
    }
    CallInst * const ci = CreateCall(f, {CreatePointerCast(base, voidPtrTy), CreateZExtOrTrunc(size, sizeTy)});
    return CreatePointerCast(ci, base->getType());
}

void CBuilder::CreateFree(Value * const ptr) {
    assert (ptr->getType()->isPointerTy());
    Module * const m = getModule();
    Type * const voidPtrTy =  getVoidPtrTy();
    Function * f = m->getFunction("free");
    if (f == nullptr) {
        FunctionType * fty = FunctionType::get(getVoidTy(), {voidPtrTy}, false);
        f = Function::Create(fty, Function::ExternalLinkage, "free", m);
        f->setCallingConv(CallingConv::C);
    }
    CreateCall(f, CreatePointerCast(ptr, voidPtrTy));
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
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        DataLayout DL(m);
        IntegerType * const intTy = getIntPtrTy(DL);
        Value * success = CreateICmpNE(CreatePtrToInt(addr, intTy), ConstantInt::getAllOnesValue(intTy)); // MAP_FAILED = -1
        CreateAssert(success, "CreateMMap: mmap failed to allocate memory");
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
        if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
            Value * success = CreateICmpNE(CreatePtrToInt(addr, intTy), ConstantInt::getAllOnesValue(intTy)); // MAP_FAILED = -1
            CreateAssert(success, "CreateMRemap: mremap failed to allocate memory");
        }
    } else { // no OS mremap support
        ptr = CreateAnonymousMMap(newSize);
        CreateMemCpy(ptr, addr, oldSize, getPageSize());
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
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        DataLayout DL(getModule());
        IntegerType * const intPtrTy = getIntPtrTy(DL);
        CreateAssert(len, "CreateMUnmap: length cannot be 0");
        Value * const addrValue = CreatePtrToInt(addr, intPtrTy);
        Value * const pageOffset = CreateURem(addrValue, ConstantInt::get(intPtrTy, getPageSize()));
        CreateAssertZero(pageOffset, "CreateMUnmap: addr must be a multiple of the page size");
        Value * const boundCheck = CreateICmpULT(addrValue, CreateSub(ConstantInt::getAllOnesValue(intPtrTy), CreateZExtOrTrunc(len, intPtrTy)));
        CreateAssert(boundCheck, "CreateMUnmap: addresses in [addr, addr+len) are outside the valid address space range");
    }
    addr = CreatePointerCast(addr, voidPtrTy);
    return CreateCall(munmapFunc, {addr, len});
}

Value * CBuilder::CreateMProtect(Value * addr, Value * size, const Protect protect) {
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        // mprotect() changes the access protections for the calling process's
        // memory pages containing any part of the address range in the interval
        // [addr, addr+len-1].  addr must be aligned to a page boundary.

        // mprotect(): POSIX.1-2001, POSIX.1-2008, SVr4.  POSIX says that the
        // behavior of mprotect() is unspecified if it is applied to a region of
        // memory that was not obtained via mmap(2).

        // On Linux, it is always permissible to call mprotect() on any address
        // in a process's address space (except for the kernel vsyscall area).
        // In particular, it can be used to change existing code mappings to be
        // writable. (NOTE: does not appear to be true on UBUNTU 16.04, 16.10 or 18.04)

        DataLayout DL(getModule());
        IntegerType * const intPtrTy = getIntPtrTy(DL);
        Constant * const pageSize = ConstantInt::get(intPtrTy, getPageSize());
        CreateAssertZero(CreateURem(CreatePtrToInt(addr, intPtrTy), pageSize), "CreateMProtect: addr must be aligned to page boundary");
    }

    IntegerType * const sizeTy = getSizeTy();
    PointerType * const voidPtrTy = getVoidPtrTy();
    IntegerType * const int32Ty = getInt32Ty();

    Module * const m = getModule();
    Function * mprotectFunc = m->getFunction("mprotect");
    if (LLVM_UNLIKELY(mprotectFunc == nullptr)) {
        FunctionType * const fty = FunctionType::get(sizeTy, {voidPtrTy, sizeTy, int32Ty}, false);
        mprotectFunc = Function::Create(fty, Function::ExternalLinkage, "mprotect", m);
    }
    addr = CreatePointerCast(addr, voidPtrTy);
    size = CreateZExtOrTrunc(size, sizeTy);
    Value * const result = CreateCall(mprotectFunc, {addr, size, ConstantInt::get(int32Ty, (int)protect)});
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        CreateAssertZero(result, "CreateMProtect: could not change the permission of the given address range");
    }
    return result;
}

IntegerType * LLVM_READNONE CBuilder::getIntAddrTy() const {
    return IntegerType::get(getContext(), sizeof(intptr_t) * 8);
}

PointerType * LLVM_READNONE CBuilder::getVoidPtrTy(const unsigned AddressSpace) const {
    //return PointerType::get(Type::getVoidTy(getContext()), AddressSpace);
    return PointerType::get(Type::getInt8Ty(getContext()), AddressSpace);
}


llvm::Value * CBuilder::CreateAtomicFetchAndAdd(Value * const val, Value * const ptr) {
    return CreateAtomicRMW(AtomicRMWInst::Add, ptr, val, AtomicOrdering::AcquireRelease);
}

llvm::Value * CBuilder::CreateAtomicFetchAndSub(Value * const val, Value * const ptr) {
    return CreateAtomicRMW(AtomicRMWInst::Sub, ptr, val, AtomicOrdering::AcquireRelease);
}

LoadInst * CBuilder::CreateAtomicLoadAcquire(Value * ptr) {
    const auto alignment = ptr->getType()->getPointerElementType()->getPrimitiveSizeInBits() / 8;
    LoadInst * inst = CreateAlignedLoad(ptr, alignment, true);
    inst->setOrdering(AtomicOrdering::Acquire);
    return inst;
}

StoreInst * CBuilder::CreateAtomicStoreRelease(Value * val, Value * ptr) {
    const auto alignment = ptr->getType()->getPointerElementType()->getPrimitiveSizeInBits() / 8;
    StoreInst * inst = CreateAlignedStore(val, ptr, alignment, true);
    inst->setOrdering(AtomicOrdering::Release);
    return inst;
}

PointerType * LLVM_READNONE CBuilder::getFILEptrTy() {
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
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        CHECK_ADDRESS(ptr, CreateMul(size, nitems), "CreateFReadCall");
    }
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
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        CHECK_ADDRESS(ptr, CreateMul(size, nitems), "CreateFReadCall");
    }
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

Type * CBuilder::getPThreadTy() {
    return IntegerType::get(getContext(), sizeof(pthread_t) * 8);
}

Value * CBuilder::CreatePThreadCreateCall(Value * thread, Value * attr, Function * start_routine, Value * arg) {
    Module * const m = getModule();
    Type * const voidPtrTy = getVoidPtrTy();
    Function * pthreadCreateFunc = m->getFunction("pthread_create");
    if (pthreadCreateFunc == nullptr) {
        FunctionType * funVoidPtrVoidTy = FunctionType::get(getVoidTy(), {voidPtrTy}, false);
        FunctionType * fty = FunctionType::get(getInt32Ty(), {getPThreadTy()->getPointerTo(), voidPtrTy, funVoidPtrVoidTy->getPointerTo(), voidPtrTy}, false);
        pthreadCreateFunc = Function::Create(fty, Function::ExternalLinkage, "pthread_create", m);
        pthreadCreateFunc->setCallingConv(CallingConv::C);
    }
    return CreateCall(pthreadCreateFunc, {thread, attr, start_routine, CreatePointerCast(arg, voidPtrTy)});
}

Value * CBuilder::CreatePThreadYield() {
    Module * const m = getModule();
    Function * f = m->getFunction("pthread_yield");
    if (f == nullptr) {
        FunctionType * fty = FunctionType::get(getInt32Ty(), false);
        f = Function::Create(fty, Function::ExternalLinkage, "pthread_yield", m);
        f->setCallingConv(CallingConv::C);
    }
    return CreateCall(f);
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
        FunctionType * fty = FunctionType::get(getInt32Ty(), {getPThreadTy(), getVoidPtrTy()->getPointerTo()}, false);
        pthreadJoinFunc = Function::Create(fty, Function::ExternalLinkage, "pthread_join", m);
        pthreadJoinFunc->setCallingConv(CallingConv::C);
    }
    return CreateCall(pthreadJoinFunc, {thread, value_ptr});
}

Value * CBuilder::CreatePThreadSelf() {
    Module * const m = getModule();
    Function * pthreadSelfFunc = m->getFunction("pthread_self");
    if (pthreadSelfFunc == nullptr) {
        FunctionType * fty = FunctionType::get(getPThreadTy(), false);
        pthreadSelfFunc = Function::Create(fty, Function::ExternalLinkage, "pthread_self", m);
        pthreadSelfFunc->setCallingConv(CallingConv::C);
    }
    return CreateCall(pthreadSelfFunc);
}

void __report_failure(const char * name, const char * msg, const uintptr_t * trace, const uint32_t n) {
    // TODO: look into boost stacktrace, available from version 1.65
    raw_fd_ostream out(STDERR_FILENO, false);
    if (trace) {
        SmallVector<char, 4096> tmp;
        raw_svector_ostream trace_string(tmp);
        for (uint32_t i = 0; i < n; ++i) {
            const auto pc = trace[i];
            trace_string << format_hex(pc, 16) << "   ";
            #ifdef __APPLE__
            const auto translator = "atos -o %s %p";
            #else
            const auto translator = "addr2line -fpCe %s %p";
            #endif
            const auto cmd = boost::format(translator) % codegen::ProgramName.data() % pc;
            FILE * const f = popen(cmd.str().data(), "r");
            if (f) {
                char buffer[1024] = {0};
                while(fgets(buffer, sizeof(buffer), f)) {
                    trace_string << buffer;
                }
                pclose(f);
            } else { // TODO: internal default


            }
        }
        out.changeColor(raw_fd_ostream::WHITE, true);
        out << "Compilation Stacktrace:\n";
        out.resetColor();
        out << trace_string.str();
    }
    if (name) {
        out.changeColor(raw_fd_ostream::RED, true);
        out << name << ": ";
    }
    out.changeColor(raw_fd_ostream::WHITE, true);
    out << msg << "\n";
    if (trace == nullptr) {
        out.changeColor(raw_fd_ostream::WHITE, true);
        out << "No debug symbols loaded.\n";
    }
    out.resetColor();
    out.flush();

}

#if defined(HAS_MACH_VM_TYPES)

/*
 * Copyright (c) 1999, 2007 Apple Inc. All rights reserved.
 *
 * @APPLE_LICENSE_HEADER_START@
 *
 * This file contains Original Code and/or Modifications of Original Code
 * as defined in and that are subject to the Apple Public Source License
 * Version 2.0 (the 'License'). You may not use this file except in
 * compliance with the License. Please obtain a copy of the License at
 * http://www.opensource.apple.com/apsl/ and read it before using this
 * file.
 *
 * The Original Code and all software distributed under the License are
 * distributed on an 'AS IS' basis, WITHOUT WARRANTY OF ANY KIND, EITHER
 * EXPRESS OR IMPLIED, AND APPLE HEREBY DISCLAIMS ALL SUCH WARRANTIES,
 * INCLUDING WITHOUT LIMITATION, ANY WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE, QUIET ENJOYMENT OR NON-INFRINGEMENT.
 * Please see the License for the specific language governing rights and
 * limitations under the License.
 *
 * @APPLE_LICENSE_HEADER_END@
 */

#include <pthread.h>
#include <mach/mach.h>
#include <mach/vm_statistics.h>
#include <stdlib.h>

#if defined(__i386__) || defined(__x86_64__)
#define FP_LINK_OFFSET 1
#elif defined(__ppc__) || defined(__ppc64__)
#define FP_LINK_OFFSET 2
#else
#error  ********** Unimplemented architecture
#endif

#define	INSTACK(a)	((uintptr_t)(a) >= stackbot && (uintptr_t)(a) <= stacktop)
#if defined(__ppc__) || defined(__ppc64__) || defined(__x86_64__)
#define	ISALIGNED(a)	((((uintptr_t)(a)) & 0xf) == 0)
#elif defined(__i386__)
#define	ISALIGNED(a)	((((uintptr_t)(a)) & 0xf) == 8)
#endif

__private_extern__  __attribute__((noinline))
void
_thread_stack_pcs(vm_address_t *buffer, unsigned max, unsigned *nb, unsigned skip)
{
    void *frame, *next;
    pthread_t self = pthread_self();
    uintptr_t stacktop = (uintptr_t)(pthread_get_stackaddr_np(self));
    uintptr_t stackbot = stacktop - (uintptr_t)(pthread_get_stacksize_np(self));

    *nb = 0;

    /* make sure return address is never out of bounds */
    stacktop -= (FP_LINK_OFFSET + 1) * sizeof(void *);

    /*
     * The original implementation called the first_frame_address() function,
     * which returned the stack frame pointer.  The problem was that in ppc,
     * it was a leaf function, so no new stack frame was set up with
     * optimization turned on (while a new stack frame was set up without
     * optimization).  We now inline the code to get the stack frame pointer,
     * so we are consistent about the stack frame.
     */
#if defined(__i386__) || defined(__x86_64__)
    frame = __builtin_frame_address(0);
#elif defined(__ppc__) || defined(__ppc64__)
    /* __builtin_frame_address IS BROKEN IN BEAKER: RADAR #2340421 */
    __asm__ volatile("mr %0, r1" : "=r" (frame));
#endif
    if(!INSTACK(frame) || !ISALIGNED(frame))
        return;
#if defined(__ppc__) || defined(__ppc64__)
    /* back up the stack pointer up over the current stack frame */
    next = *(void **)frame;
    if(!INSTACK(next) || !ISALIGNED(next) || next <= frame)
        return;
    frame = next;
#endif
    while (skip--) {
        next = *(void **)frame;
        if(!INSTACK(next) || !ISALIGNED(next) || next <= frame)
            return;
        frame = next;
    }
    while (max--) {
        buffer[*nb] = *(vm_address_t *)(((void **)frame) + FP_LINK_OFFSET);
        (*nb)++;
        next = *(void **)frame;
        if(!INSTACK(next) || !ISALIGNED(next) || next <= frame)
            return;
        frame = next;
    }
}
#endif

void CBuilder::__CreateAssert(Value * const assertion, const Twine failureMessage) {
    if (LLVM_UNLIKELY(isa<Constant>(assertion))) {
        if (LLVM_UNLIKELY(cast<Constant>(assertion)->isNullValue())) {
            report_fatal_error(failureMessage);
        }
    } else if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        Module * const m = getModule();
        Type * const stackTy = IntegerType::get(getContext(), sizeof(uintptr_t) * 8);
        PointerType * const stackPtrTy = stackTy->getPointerTo();
        PointerType * const int8PtrTy = getInt8PtrTy();
        Function * function = m->getFunction("assert");
        if (LLVM_UNLIKELY(function == nullptr)) {
            auto ip = saveIP();
            IntegerType * const int1Ty = getInt1Ty();
            FunctionType * fty = FunctionType::get(getVoidTy(), { int1Ty, int8PtrTy, int8PtrTy, stackPtrTy, getInt32Ty() }, false);
            function = Function::Create(fty, Function::PrivateLinkage, "assert", m);
            function->setDoesNotThrow();
            #if LLVM_VERSION_INTEGER < LLVM_VERSION_CODE(5, 0, 0)
            function->setDoesNotAlias(2);
            #endif
            BasicBlock * const entry = BasicBlock::Create(getContext(), "", function);
            BasicBlock * const failure = BasicBlock::Create(getContext(), "", function);
            BasicBlock * const success = BasicBlock::Create(getContext(), "", function);
            auto arg = function->arg_begin();
            arg->setName("assertion");
            Value * assertion = &*arg++;
            arg->setName("name");
            Value * name = &*arg++;
            arg->setName("msg");
            Value * msg = &*arg++;
            arg->setName("trace");
            Value * trace = &*arg++;
            arg->setName("depth");
            Value * depth = &*arg++;
            SetInsertPoint(entry);
            IRBuilder<>::CreateCondBr(assertion, success, failure);
            IRBuilder<>::SetInsertPoint(failure);
            IRBuilder<>::CreateCall(LinkFunction("__report_failure", __report_failure), { name, msg, trace, depth });
            CreateExit(-1);
            IRBuilder<>::CreateBr(success); // necessary to satisfy the LLVM verifier. this is never executed.
            SetInsertPoint(success);
            IRBuilder<>::CreateRetVoid();
            restoreIP(ip);
        }
        #ifndef NDEBUG
        SmallVector<unw_word_t, 64> stack;
        #if defined(HAS_MACH_VM_TYPES)
        for (;;) {
            unsigned int n;
            _thread_stack_pcs(reinterpret_cast<vm_address_t *>(stack.data()), stack.capacity(), &n, 1);
            if (LLVM_UNLIKELY(n < stack.capacity() || stack[n - 1] == 0)) {
                while (n >= 1 && stack[n - 1] == 0) {
                    n -= 1;
                }
                stack.set_size(n);
                break;
            }
            stack.reserve(n * 2);
        }
        #elif defined(HAS_LIBUNWIND)
        unw_context_t context;
        // Initialize cursor to current frame for local unwinding.
        unw_getcontext(&context);
        unw_cursor_t cursor;
        unw_init_local(&cursor, &context);
        // Unwind frames one by one, going up the frame stack.
        while (unw_step(&cursor) > 0) {
            unw_word_t pc;
            unw_get_reg(&cursor, UNW_REG_IP, &pc);
            if (pc == 0) {
                break;
            }
            stack.push_back(pc);
        }
        #elif defined(HAS_EXECINFO)
        for (;;) {
            const auto n = backtrace(reinterpret_cast<void **>(stack.data()), stack.capacity());
            if (LLVM_LIKELY(n < (int)stack.capacity())) {
                stack.set_size(n);
                break;
            }
            stack.reserve(n * 2);
        }
        #endif
        // TODO: look into how to safely use __builtin_return_address(0)?


        const unsigned FIRST_NON_ASSERT = 2;
        Value * trace = nullptr;
        ConstantInt * depth = nullptr;
        if (LLVM_UNLIKELY(stack.size() < FIRST_NON_ASSERT)) {
            trace = ConstantPointerNull::get(stackPtrTy);
            depth = getInt32(0);
        } else {
            const auto n = stack.size() - FIRST_NON_ASSERT;
            for (GlobalVariable & gv : m->getGlobalList()) {
                Type * const ty = gv.getValueType();
                if (ty->isArrayTy() && ty->getArrayElementType() == stackTy && ty->getArrayNumElements() == n) {
                    const ConstantDataArray * const array = cast<ConstantDataArray>(gv.getOperand(0));
                    bool found = true;
                    for (size_t i = 0; i < n; ++i) {
                        if (LLVM_LIKELY(array->getElementAsInteger(i) != stack[i + FIRST_NON_ASSERT])) {
                            found = false;
                            break;
                        }
                    }
                    if (LLVM_UNLIKELY(found)) {
                        trace = &gv;
                        break;
                    }
                }
            }
            if (LLVM_LIKELY(trace == nullptr)) {
                Constant * const initializer = ConstantDataArray::get(getContext(), ArrayRef<unw_word_t>(stack.data() + FIRST_NON_ASSERT, n));
                trace = new GlobalVariable(*m, initializer->getType(), true, GlobalVariable::InternalLinkage, initializer);
            }
            trace = CreatePointerCast(trace, stackPtrTy);
            depth = getInt32(n);
        }
        #else
        Value * trace = ConstantPointerNull::get(stackPtrTy);
        Value * depth = getInt32(0);
        #endif
        Value * const name = GetString(getKernelName());
        SmallVector<char, 1024> tmp;
        Value * const msg = GetString(failureMessage.toStringRef(tmp));
        IRBuilder<>::CreateCall(function, {assertion, name, msg, trace, depth});
    } else { // if assertions are not enabled, make it a compiler assumption.

        // INVESTIGATE: while interesting, this does not seem to produce faster code and only provides a trivial
        // reduction of compiled code size in LLVM 3.8 but nearly doubles JIT compilation time. This may have been
        // improved with later versions of LLVM but it's likely that assumptions ought to be hand placed once
        // they're proven to improve performance.

        // IRBuilder<>::CreateAssumption(assertion);
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

BasicBlock * CBuilder::CreateBasicBlock(const StringRef name, BasicBlock * insertBefore) {
    return BasicBlock::Create(getContext(), name, GetInsertBlock()->getParent(), insertBefore);
}

bool CBuilder::supportsIndirectBr() const {
    return !codegen::DebugOptionIsSet(codegen::DisableIndirectBranch);
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

Value * CBuilder::CreateCountForwardZeroes(Value * value, const bool guaranteedNonZero) {
    if (LLVM_UNLIKELY(guaranteedNonZero && codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        CreateAssert(value, "CreateCountForwardZeroes: value cannot be zero!");
    }
    Value * cttzFunc = Intrinsic::getDeclaration(getModule(), Intrinsic::cttz, value->getType());
    return CreateCall(cttzFunc, {value, getInt1(guaranteedNonZero)});
}

Value * CBuilder::CreateCountReverseZeroes(Value * value, const bool guaranteedNonZero) {
    if (LLVM_UNLIKELY(guaranteedNonZero && codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        CreateAssert(value, "CreateCountReverseZeroes: value cannot be zero!");
    }
    Value * ctlzFunc = Intrinsic::getDeclaration(getModule(), Intrinsic::ctlz, value->getType());
    return CreateCall(ctlzFunc, {value, getInt1(guaranteedNonZero)});
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

Value * CBuilder::CreateExtractBitField(Value * bits, Value * start, Value * length) {
    Constant * One = ConstantInt::get(bits->getType(), 1);
    return CreateAnd(CreateLShr(bits, start), CreateSub(CreateShl(One, length), One));
}

Value * CBuilder::CreateCeilLog2(Value * value) {
    IntegerType * ty = cast<IntegerType>(value->getType());
    Value * m = CreateCountReverseZeroes(CreateSub(value, ConstantInt::get(ty, 1)));
    return CreateSub(ConstantInt::get(m->getType(), ty->getBitWidth()), m);
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

Function * CBuilder::LinkFunction(StringRef name, FunctionType * type, void * functionPtr) const {
    assert (mDriver);
    return mDriver->addLinkFunction(getModule(), name, type, functionPtr);
}

LoadInst * CBuilder::CreateLoad(Value *Ptr, const char * Name) {
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        CHECK_ADDRESS(Ptr, ConstantExpr::getSizeOf(Ptr->getType()->getPointerElementType()), "CreateLoad");
    }
    return IRBuilder<>::CreateLoad(Ptr, Name);
}

LoadInst * CBuilder::CreateLoad(Value * Ptr, const Twine & Name) {
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        CHECK_ADDRESS(Ptr, ConstantExpr::getSizeOf(Ptr->getType()->getPointerElementType()), "CreateLoad");
    }
    return IRBuilder<>::CreateLoad(Ptr, Name);
}

LoadInst * CBuilder::CreateLoad(Type * Ty, Value *Ptr, const Twine & Name) {
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        CHECK_ADDRESS(Ptr, ConstantExpr::getSizeOf(Ty), "CreateLoad");
    }
    return IRBuilder<>::CreateLoad(Ty, Ptr, Name);
}

LoadInst * CBuilder::CreateLoad(Value * Ptr, bool isVolatile, const Twine & Name) {
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        CHECK_ADDRESS(Ptr, ConstantExpr::getSizeOf(Ptr->getType()->getPointerElementType()), "CreateLoad");
    }
    return IRBuilder<>::CreateLoad(Ptr, isVolatile, Name);
}

StoreInst * CBuilder::CreateStore(Value * Val, Value * Ptr, bool isVolatile) {
    assert ("Ptr is not a pointer type for Val" &&
            Ptr->getType()->isPointerTy() && Val->getType() == Ptr->getType()->getPointerElementType());
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        CHECK_ADDRESS(Ptr, ConstantExpr::getSizeOf(Val->getType()), "CreateStore");
    }
    return IRBuilder<>::CreateStore(Val, Ptr, isVolatile);
}

inline bool CBuilder::hasAddressSanitizer() const {
    return mDriver && mDriver->hasExternalFunction("__asan_region_is_poisoned");
}

LoadInst * CBuilder::CreateAlignedLoad(Value * Ptr, unsigned Align, const char * Name) {
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        DataLayout DL(getModule());
        IntegerType * const intPtrTy = cast<IntegerType>(DL.getIntPtrType(Ptr->getType()));
        Value * alignmentOffset = CreateURem(CreatePtrToInt(Ptr, intPtrTy), ConstantInt::get(intPtrTy, Align));
        CreateAssertZero(alignmentOffset, "CreateAlignedLoad: pointer is misaligned");
    }
    LoadInst * LI = CreateLoad(Ptr, Name);
    LI->setAlignment(Align);
    return LI;
}

LoadInst * CBuilder::CreateAlignedLoad(Value * Ptr, unsigned Align, const Twine & Name) {
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        DataLayout DL(getModule());
        IntegerType * const intPtrTy = cast<IntegerType>(DL.getIntPtrType(Ptr->getType()));
        Value * alignmentOffset = CreateURem(CreatePtrToInt(Ptr, intPtrTy), ConstantInt::get(intPtrTy, Align));
        CreateAssertZero(alignmentOffset, "CreateAlignedLoad: pointer is misaligned");
    }
    LoadInst * LI = CreateLoad(Ptr, Name);
    LI->setAlignment(Align);
    return LI;
}

LoadInst * CBuilder::CreateAlignedLoad(Value * Ptr, unsigned Align, bool isVolatile, const Twine & Name) {
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        DataLayout DL(getModule());
        IntegerType * const intPtrTy = cast<IntegerType>(DL.getIntPtrType(Ptr->getType()));
        Value * alignmentOffset = CreateURem(CreatePtrToInt(Ptr, intPtrTy), ConstantInt::get(intPtrTy, Align));
        CreateAssertZero(alignmentOffset, "CreateAlignedLoad: pointer is misaligned");
    }
    LoadInst * LI = CreateLoad(Ptr, isVolatile, Name);
    LI->setAlignment(Align);
    return LI;
}

StoreInst * CBuilder::CreateAlignedStore(Value * Val, Value * Ptr, unsigned Align, bool isVolatile) {
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        DataLayout DL(getModule());
        IntegerType * const intPtrTy = cast<IntegerType>(DL.getIntPtrType(Ptr->getType()));
        Value * alignmentOffset = CreateURem(CreatePtrToInt(Ptr, intPtrTy), ConstantInt::get(intPtrTy, Align));
        CreateAssertZero(alignmentOffset, "CreateAlignedStore: pointer is misaligned");
    }
    StoreInst *SI = CreateStore(Val, Ptr, isVolatile);
    SI->setAlignment(Align);
    return SI;
}

Value * CBuilder::CreateMemChr(Value * ptr, Value * byteVal, Value * num) {
    Module * const m = getModule();
    Function * memchrFn = m->getFunction("memchr");
    if (memchrFn == nullptr) {
        IntegerType * const int32Ty = getInt32Ty();
        IntegerType * const sizeTy = getSizeTy();
        PointerType * const voidPtrTy = getVoidPtrTy();
        FunctionType * memchrTy = FunctionType::get(voidPtrTy, {voidPtrTy, int32Ty, sizeTy}, false);
        memchrFn = cast<Function>(m->getOrInsertFunction("memchr", memchrTy));
    }
    return CreateCall(memchrFn, {ptr, byteVal, num});
}

CallInst * CBuilder::CreateMemMove(Value * Dst, Value * Src, Value *Size, unsigned Align, bool isVolatile,
                                   MDNode *TBAATag, MDNode *ScopeTag, MDNode *NoAliasTag) {
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        CHECK_ADDRESS(Src, Size, "CreateMemMove: Src");
        CHECK_ADDRESS(Dst, Size, "CreateMemMove: Dst");
        // If the call to this intrinisic has an alignment value that is not 0 or 1, then the caller
        // guarantees that both the source and destination pointers are aligned to that boundary.
        if (Align > 1) {
            DataLayout DL(getModule());
            IntegerType * const intPtrTy = DL.getIntPtrType(getContext());
            Value * intSrc = CreatePtrToInt(Src, intPtrTy);
            Value * intDst = CreatePtrToInt(Dst, intPtrTy);
            ConstantInt * align = ConstantInt::get(intPtrTy, Align);
            CreateAssertZero(CreateURem(intSrc, align), "CreateMemMove: Src pointer is misaligned");
            CreateAssertZero(CreateURem(intDst, align), "CreateMemMove: Dst pointer is misaligned");

        }
    }
#if LLVM_VERSION_INTEGER >= LLVM_VERSION_CODE(7, 0, 0)
    return IRBuilder<>::CreateMemMove(Dst, Align, Src, Align, Size, isVolatile, TBAATag, ScopeTag, NoAliasTag);
#else
    return IRBuilder<>::CreateMemMove(Dst, Src, Size, Align, isVolatile, TBAATag, ScopeTag, NoAliasTag);
#endif
}

CallInst * CBuilder::CreateMemCpy(Value *Dst, Value *Src, Value *Size, unsigned Align, bool isVolatile,
                                  MDNode *TBAATag, MDNode *TBAAStructTag, MDNode *ScopeTag, MDNode *NoAliasTag) {
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        CHECK_ADDRESS(Src, Size, "CreateMemCpy: Src");
        CHECK_ADDRESS(Dst, Size, "CreateMemCpy: Dst");
        DataLayout DL(getModule());
        IntegerType * const intPtrTy = DL.getIntPtrType(getContext());
        Value * intSrc = CreatePtrToInt(Src, intPtrTy);
        Value * intDst = CreatePtrToInt(Dst, intPtrTy);
        // If the call to this intrinisic has an alignment value that is not 0 or 1, then the caller
        // guarantees that both the source and destination pointers are aligned to that boundary.
        if (Align > 1) {
            ConstantInt * align = ConstantInt::get(intPtrTy, Align);
            CreateAssertZero(CreateURem(intSrc, align), "CreateMemCpy: Src is misaligned");
            CreateAssertZero(CreateURem(intDst, align), "CreateMemCpy: Dst is misaligned");
        }
        Value * intSize = CreateZExtOrTrunc(Size, intPtrTy);
        Value * nonOverlapping = CreateOr(CreateICmpULT(CreateAdd(intSrc, intSize), intDst),
                                          CreateICmpULT(CreateAdd(intDst, intSize), intSrc));
        CreateAssert(nonOverlapping, "CreateMemCpy: overlapping ranges is undefined");
    }
#if LLVM_VERSION_INTEGER >= LLVM_VERSION_CODE(7, 0, 0)
    return IRBuilder<>::CreateMemCpy(Dst, Align, Src, Align, Size, isVolatile, TBAATag, TBAAStructTag, ScopeTag, NoAliasTag);
#else
    return IRBuilder<>::CreateMemCpy(Dst, Src, Size, Align, isVolatile, TBAATag, TBAAStructTag, ScopeTag, NoAliasTag);
#endif
}

CallInst * CBuilder::CreateMemSet(Value * Ptr, Value * Val, Value * Size, unsigned Align,
                       bool isVolatile, MDNode * TBAATag, MDNode * ScopeTag, MDNode * NoAliasTag) {
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        CHECK_ADDRESS(Ptr, Size, "CreateMemSet");
        if (Align > 1) {
            DataLayout DL(getModule());
            IntegerType * const intPtrTy = DL.getIntPtrType(getContext());
            Value * intPtr = CreatePtrToInt(Ptr, intPtrTy);
            ConstantInt * align = ConstantInt::get(intPtrTy, Align);
            CreateAssertZero(CreateURem(intPtr, align), "CreateMemSet: Ptr is misaligned");
        }
    }
    return IRBuilder<>::CreateMemSet(Ptr, Val, Size, Align, isVolatile, TBAATag, ScopeTag, NoAliasTag);
}

CallInst * CBuilder::CreateMemCmp(Value * Ptr1, Value * Ptr2, Value * Num) {
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        CHECK_ADDRESS(Ptr1, Num, "CreateMemCmp: Ptr1");
        CHECK_ADDRESS(Ptr2, Num, "CreateMemCmp: Ptr2");
    }
    Module * const m = getModule();
    Function * f = m->getFunction("memcmp");
    PointerType * const voidPtrTy = getVoidPtrTy();
    IntegerType * const sizeTy = getSizeTy();
    if (f == nullptr) {
        FunctionType * const fty = FunctionType::get(getInt32Ty(), {voidPtrTy, voidPtrTy, sizeTy}, false);
        f = Function::Create(fty, Function::ExternalLinkage, "memcmp", m);
        f->setCallingConv(CallingConv::C);
        #if LLVM_VERSION_INTEGER < LLVM_VERSION_CODE(5, 0, 0)
        f->setDoesNotAlias(1);
        f->setDoesNotAlias(2);
        #endif
    }
    Ptr1 = CreatePointerCast(Ptr1, voidPtrTy);
    Ptr2 = CreatePointerCast(Ptr2, voidPtrTy);
    Num = CreateZExtOrTrunc(Num, sizeTy);
    return CreateCall(f, {Ptr1, Ptr2, Num});
}

Value * CBuilder::CreateExtractElement(Value * Vec, Value *Idx, const Twine & Name) {
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        if (LLVM_UNLIKELY(!Vec->getType()->isVectorTy())) {
            report_fatal_error("CreateExtractElement: Vec argument is not a vector type");
        }
        Constant * const Size = ConstantInt::get(Idx->getType(), Vec->getType()->getVectorNumElements());
        // exctracting an element from a position that exceeds the length of the vector is undefined
        CreateAssert(CreateICmpULT(Idx, Size), "CreateExtractElement: Idx is greater than Vec size");
    }
    return IRBuilder<>::CreateExtractElement(Vec, Idx, Name);
}

Value * CBuilder::CreateInsertElement(Value * Vec, Value * NewElt, Value * Idx, const Twine & Name) {
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        if (LLVM_UNLIKELY(!Vec->getType()->isVectorTy())) {
            report_fatal_error("CreateExtractElement: Vec argument is not a vector type");
        }
        Constant * const Size = ConstantInt::get(Idx->getType(), Vec->getType()->getVectorNumElements());
        // inserting an element into a position that exceeds the length of the vector is undefined
        CreateAssert(CreateICmpULT(Idx, Size), "CreateInsertElement: Idx is greater than Vec size");
    }
    return IRBuilder<>::CreateInsertElement(Vec, NewElt, Idx, Name);
}

CallInst * CBuilder::CreateSRandCall(Value * randomSeed) {
    Module * const m = getModule();
    Function * srandFunc = m->getFunction("srand");
    if (srandFunc == nullptr) {
        FunctionType * fty = FunctionType::get(getVoidTy(), {getInt32Ty()}, false);
        srandFunc = Function::Create(fty, Function::ExternalLinkage, "srand", m);
        srandFunc->setCallingConv(CallingConv::C);
    }
    return CreateCall(srandFunc, randomSeed);
}

CallInst * CBuilder::CreateRandCall() {
    Module * const m = getModule();
    Function * randFunc = m->getFunction("rand");
    if (randFunc == nullptr) {
        FunctionType * fty = FunctionType::get(getInt32Ty(), false);
        randFunc = Function::Create(fty, Function::ExternalLinkage, "rand", m);
        randFunc->setCallingConv(CallingConv::C);
    }
    return CreateCall(randFunc, {});
}

unsigned CBuilder::getPageSize() {
    return boost::interprocess::mapped_region::get_page_size();
}


CBuilder::CBuilder(LLVMContext & C)
: IRBuilder<>(C)
, mCacheLineAlignment(64)
, mSizeType(IntegerType::get(getContext(), sizeof(size_t) * 8))
, mFILEtype(nullptr)
, mDriver(nullptr) {

}

struct RemoveRedundantAssertionsPass : public ModulePass {
    static char ID;
    RemoveRedundantAssertionsPass() : ModulePass(ID) { }

    virtual bool runOnModule(Module &M) override;
};

ModulePass * createRemoveRedundantAssertionsPass() {
    return new RemoveRedundantAssertionsPass();
}

char RemoveRedundantAssertionsPass::ID = 0;

bool RemoveRedundantAssertionsPass::runOnModule(Module & M) {
    Function * const assertFunc = M.getFunction("assert");
    if (LLVM_UNLIKELY(assertFunc == nullptr)) {
        return false;
    }
    bool modified = false;
    DenseSet<Value *> S;
    for (auto & F : M) {
        for (auto & B : F) {
            S.clear();
            for (BasicBlock::iterator i = B.begin(); i != B.end(); ) {
                Instruction & inst = *i;
                if (LLVM_UNLIKELY(isa<CallInst>(inst))) {
                    CallInst & ci = cast<CallInst>(inst);
                    if (ci.getCalledFunction() == assertFunc) {
                        bool remove = false;
                        Value * const check = ci.getOperand(0);
                        if (LLVM_UNLIKELY(isa<Constant>(check))) {
                            if (LLVM_LIKELY(cast<Constant>(check)->isOneValue())) {
                                remove = true;
                            } else {
                                // TODO: show all static failures with their compilation context
                            }
                        } else if (LLVM_UNLIKELY(S.count(check))) { // will never be executed
                            remove = true;
                        } else {
                            S.insert(check);
                        }
                        if (LLVM_UNLIKELY(remove)) {
                            i = ci.eraseFromParent();
                            modified = true;
                            continue;
                        }
                    }
                }
                ++i;
            }
        }
    }
    return modified;
}
