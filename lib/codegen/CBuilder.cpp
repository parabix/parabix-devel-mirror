/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <codegen/CBuilder.h>

#include <llvm/IR/Mangler.h>
#include <llvm/IR/Module.h>
#include <llvm/IR/Constants.h>
#include <llvm/IR/Intrinsics.h>
#include <llvm/IR/MDBuilder.h>
#include <llvm/IR/Metadata.h>
#include <llvm/IR/Dominators.h>
#include <llvm/ADT/DenseSet.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/Support/Format.h>
#include <toolchain/toolchain.h>
#include <stdlib.h>
#include <stdarg.h>
#include <cstdarg>
#include <sys/mman.h>
#include <unistd.h>
#include <stdio.h>
#include <boost/format.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <cxxabi.h>

#ifdef HAS_ADDRESS_SANITIZER
#include <llvm/Analysis/AliasAnalysis.h>
#endif

#if defined(__i386__)
typedef uint32_t unw_word_t;
#define PRISz PRId32
#else
typedef uint64_t unw_word_t;
#define PRISz PRId64
#endif
#if defined(HAS_LIBUNWIND)
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


static int accumulatedFreeCalls = 0;

extern "C" void free_debug_wrapper(void * ptr) {
    accumulatedFreeCalls++;
    if (accumulatedFreeCalls <= codegen::FreeCallBisectLimit) {
        free(ptr);
    }
}

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
    return b->CreateICmpEQ(check, ConstantPointerNull::get(voidPtrTy));
}

#define CHECK_HEAP_ADDRESS(Ptr, Size, Name) \
if (LLVM_UNLIKELY(hasAddressSanitizer())) { \
    __CreateAssert(checkHeapAddress(this, Ptr, Size), Name " was given an unallocated %" PRIuMAX "-byte memory address 0x%" PRIxPTR, {Size, Ptr}); \
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
    if (LLVM_LIKELY(isa<Constant>(divisor))) {
        if (LLVM_UNLIKELY(cast<Constant>(divisor)->isOneValue())) {
            return number;
        }
    }
    // TODO: verify the ASM reuses the remainder from the initial division
    IntegerType * const intTy = cast<IntegerType>(number->getType());
    Value * const quot = CreateUDiv(number, divisor);
    Value * const rem = CreateURem(number, divisor);
    return CreateAdd(quot, CreateZExt(CreateIsNotNull(rem), intTy), Name);
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
        openFn = Function::Create(openTy, Function::ExternalLinkage, "open", m);
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
        write = Function::Create(writeTy, Function::ExternalLinkage, "write", m);
        write->addAttribute(2U, Attribute::NoAlias);
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
        readFn = Function::Create(readTy, Function::ExternalLinkage, "read", m);
        readFn->addAttribute(2U, Attribute::NoAlias);
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

Value * CBuilder::CreatePosixFAllocate(Value * fileDescriptor, Value * offset, Value * len) {
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
        Function * printFn = Function::Create(FT, Function::InternalLinkage, "print_int", m);
        auto arg = printFn->arg_begin();
        BasicBlock * entry = BasicBlock::Create(getContext(), "entry", printFn);
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
        printRegister = printFn;
    }
    Value * num = nullptr;
    if (value->getType()->isPointerTy()) {
        num = CreatePtrToInt(value, int64Ty);
    } else if (value->getType()->isIntegerTy()) {
        num = CreateZExt(value, int64Ty);
    } else {
        report_fatal_error("CallPrintInt was given a non-integer/non-pointer value.");
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
    size = CreateRoundUp(size, ConstantInt::get(size->getType(), getCacheAlignment()));
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
        Constant * const ZERO = ConstantInt::get(sizeTy, 0);
        __CreateAssert(CreateICmpNE(size, ZERO),
                       "CreateAlignedMalloc: 0-byte malloc "
                       "is implementation defined", {});

        __CreateAssert(CreateICmpEQ(CreateURem(size, align), ZERO),
                       "CreateAlignedMalloc: size (%d) must be an "
                       "integral multiple of alignment (%d).", {size, align});
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

Value * CBuilder::CreateRealloc(Type * const type, Value * const base, Value * const ArraySize) {
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
    if (codegen::FreeCallBisectLimit >= 0) {
        Function * dispatcher = m->getFunction("free_debug_wrapper");
        if (dispatcher == nullptr) {
            FunctionType * ty = FunctionType::get(getVoidTy(), {voidPtrTy}, false);
            dispatcher = Function::Create(ty, Function::ExternalLinkage, "free_debug_wrapper", m);
            dispatcher->setCallingConv(CallingConv::C);
            assert (dispatcher);
            CreateCall(dispatcher, CreatePointerCast(ptr, voidPtrTy));
        }
    } else {
        Function * f = m->getFunction("free");
        if (f == nullptr) {
            FunctionType * fty = FunctionType::get(getVoidTy(), {voidPtrTy}, false);
            f = Function::Create(fty, Function::ExternalLinkage, "free", m);
            f->setCallingConv(CallingConv::C);
        }
        CreateCall(f, CreatePointerCast(ptr, voidPtrTy));
    }
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


Value * CBuilder::CreateAtomicFetchAndAdd(Value * const val, Value * const ptr) {
    return CreateAtomicRMW(AtomicRMWInst::Add, ptr, val, AtomicOrdering::AcquireRelease);
}

Value * CBuilder::CreateAtomicFetchAndSub(Value * const val, Value * const ptr) {
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

void CBuilder::setNontemporal(StoreInst * s) {
    s->setMetadata(LLVMContext::MD_nontemporal,
                   MDNode::get(getContext(), {ConstantAsMetadata::get(getInt32(1))}));
}

Value * CBuilder::CreatePrefetch(Value * ptr, PrefetchRW mode, unsigned locality, CacheType c) {
    Value * prefetchIntrin = Intrinsic::getDeclaration(getModule(), Intrinsic::prefetch);
    Value * modeVal = getInt32(mode == PrefetchRW::Read ? 0 : 1);
    Value * localityVal = getInt32(locality > 3 ? 3 : locality);
    Value * cacheKind = getInt32(c == CacheType::Instruction ? 0 : 1);
    return CreateCall(prefetchIntrin, {CreateBitCast(ptr, getInt8PtrTy()), modeVal, localityVal, cacheKind});
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
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        __CreateAssert(CreateIsNotNull(thread), "PThreadCreate: thread cannot be null", {});
        __CreateAssert(CreateIsNotNull(start_routine), "PThreadCreate: start_routine cannot be null", {});
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

extern "C"
BOOST_NOINLINE
void __report_failure_v(const char * name, const char * fmt, const uintptr_t * trace, const uint32_t traceLength, va_list & args) {
    // TODO: look into boost stacktrace, available from version 1.65
    raw_fd_ostream out(STDERR_FILENO, false);
    if (trace) {
        SmallVector<char, 4096> tmp;
        raw_svector_ostream trace_string(tmp);
        for (uint32_t i = 0; i < traceLength; ++i) {
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
    char buffer[1024] = {0};
    const auto m = std::vsprintf(buffer, fmt, args);
    assert (m > 0);
    out.write(buffer, m);
    if (trace == nullptr) {
        out.changeColor(raw_fd_ostream::WHITE, true);
        out << "No debug symbols loaded.\n";
    }
    out.resetColor();
    out << "\n\n";
    out.flush();
}

extern "C"
BOOST_NOINLINE
void __report_failure(const char * name, const char * fmt, const uintptr_t * trace, const uint32_t traceLength, ...) {
    va_list args;
    va_start(args, traceLength);
    __report_failure_v(name, fmt, trace, traceLength, args);
    va_end(args);
}

void CBuilder::__CreateAssert(Value * const assertion, const Twine format, std::initializer_list<llvm::Value *> params) {

    if (LLVM_UNLIKELY(isa<Constant>(assertion))) {
        if (LLVM_LIKELY(!cast<Constant>(assertion)->isNullValue())) {
            return;
        }
    }

    Module * const m = getModule();
    LLVMContext & C = getContext();
    Type * const stackTy = IntegerType::get(C, sizeof(uintptr_t) * 8);
    PointerType * const stackPtrTy = stackTy->getPointerTo();
    PointerType * const int8PtrTy = getInt8PtrTy();

    Function * assertFunc = m->getFunction("assert");
    if (LLVM_UNLIKELY(assertFunc == nullptr)) {

        auto ip = saveIP();
        IntegerType * const int1Ty = getInt1Ty();
        IntegerType * const int32Ty = getInt32Ty();
        PointerType * const int8PtrPtrTy = int8PtrTy->getPointerTo();
        // va_list is platform specific but since we are not directly modifying
        // any use of this type in LLVM code, just ensure it is large enough.
        ArrayType * const vaListTy = ArrayType::get(getInt8Ty(), sizeof(va_list));
        Type * const voidTy = getVoidTy();

        FunctionType * fty = FunctionType::get(voidTy, { int1Ty, int8PtrTy, int8PtrTy, stackPtrTy, int32Ty }, true);
        assertFunc = Function::Create(fty, Function::PrivateLinkage, "assert", m);
        #if LLVM_VERSION_INTEGER < LLVM_VERSION_CODE(5, 0, 0)
        assertFunc->setDoesNotAlias(2);
        #endif
        BasicBlock * const entry = BasicBlock::Create(C, "", assertFunc);
        BasicBlock * const failure = BasicBlock::Create(C, "", assertFunc);
        BasicBlock * const success = BasicBlock::Create(C, "", assertFunc);
        auto arg = assertFunc->arg_begin();
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

        assertFunc->setHasUWTable();
        assertFunc->setPersonalityFn(getDefaultPersonalityFunction());

        Value * const vaList = CreatePointerCast(CreateAlignedAlloca(vaListTy, mCacheLineAlignment), int8PtrTy);
        FunctionType * vaFuncTy = FunctionType::get(voidTy, { int8PtrTy }, false);
        Function * const vaStart = Function::Create(vaFuncTy, Function::ExternalLinkage, "llvm.va_start", m);
        Function * const vaEnd = Function::Create(vaFuncTy, Function::ExternalLinkage, "llvm.va_end", m);

        CreateCondBr(assertion, success, failure);

        SetInsertPoint(failure);
        FunctionType * const rfTy = FunctionType::get(voidTy, { int8PtrTy, int8PtrTy, stackPtrTy, int32Ty, int8PtrTy }, false);
        Function * const reportFn = mDriver->addLinkFunction(m, "__report_failure_v", rfTy,
                                                             reinterpret_cast<void *>(&__report_failure_v));
        reportFn->setCallingConv(CallingConv::C);

        CreateCall(vaStart, vaList);

        FixedArray<Value *, 5> args;
        args[0] = name;
        args[1] = msg;
        args[2] = trace;
        args[3] = depth;
        args[4] = vaList;

        CreateCall(reportFn, args);
        CreateCall(vaEnd, vaList);

        Function * alloc_exception = getAllocateException();
        Value * const exception = CreateCall(alloc_exception, { ConstantExpr::getSizeOf(int8PtrTy) } );
        Constant * const nil = ConstantPointerNull::get(int8PtrTy);
        IRBuilder<>::CreateStore(nil, CreateBitCast(exception, int8PtrPtrTy));
        // NOTE: the second argument is supposed to point to a std::type_info object.
        // The external value Clang passes into it resolves to "null" when RTTI is disabled.
        // This appears to work here but ought to be verified.
        CreateCall(getThrow(), { exception, nil, nil });
        CreateUnreachable();

        SetInsertPoint(success);
        CreateRetVoid();

        restoreIP(ip);
    }
    #ifndef NDEBUG
    SmallVector<unw_word_t, 64> stack;
    #if defined(HAS_EXECINFO)
    for (;;) {
        const auto n = backtrace(reinterpret_cast<void **>(stack.data()), stack.capacity());
        if (LLVM_LIKELY(n < (int)stack.capacity())) {
            stack.set_size(n);
            break;
        }
        stack.reserve(n * 2);
    }
    #endif
    const unsigned FIRST_NON_ASSERT = 2;
    Constant * trace = nullptr;
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
        trace = ConstantExpr::getPointerCast(trace, stackPtrTy);
        depth = getInt32(n);
    }
    #else
    Value * trace = ConstantPointerNull::get(stackPtrTy);
    Value * depth = getInt32(0);
    #endif
    Value * const name = GetString(getKernelName());
    SmallVector<char, 1024> tmp;
    const StringRef fmt = format.toStringRef(tmp);
    // TODO: add a check that the number of var_args == number of message args?


    SmallVector<Value *, 12> args(5);
    args[0] = assertion;
    args[1] = name;
    args[2] = GetString(fmt);
    args[3] = trace;
    args[4] = depth;
    args.insert(args.begin() + 5, params);
    assert (args.size() == params.size() + 5);
    IRBuilder<>::CreateCall(assertFunc, args);
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

Value * CBuilder::CreateResetLowestBit(Value * bits, const Twine & Name) {
    return CreateAnd(bits, CreateSub(bits, ConstantInt::get(bits->getType(), 1)), Name);
}

Value * CBuilder::CreateIsolateLowestBit(Value * bits, const Twine & Name) {
    return CreateAnd(bits, CreateNeg(bits), Name);
}

Value * CBuilder::CreateMaskToLowestBitInclusive(Value * bits, const Twine & Name) {
    return CreateXor(bits, CreateSub(bits, ConstantInt::get(bits->getType(), 1)), Name);
}

Value * CBuilder::CreateMaskToLowestBitExclusive(Value * bits, const Twine & Name) {
    return CreateAnd(CreateSub(bits, ConstantInt::get(bits->getType(), 1)), CreateNot(bits), Name);
}

Value * CBuilder::CreateZeroHiBitsFrom(Value * bits, Value * pos, const Twine & Name) {
    Type * Ty = bits->getType();
    Constant * one = Constant::getIntegerValue(Ty, APInt(Ty->getScalarSizeInBits(), 1));
    Value * mask = CreateSub(CreateShl(one, pos), one);
    return CreateAnd(bits, mask, Name);
}

Value * CBuilder::CreateExtractBitField(Value * bits, Value * start, Value * length, const Twine & Name) {
    Constant * One = ConstantInt::get(bits->getType(), 1);
    return CreateAnd(CreateLShr(bits, start), CreateSub(CreateShl(One, length), One), Name);
}

Value * CBuilder::CreateCeilLog2(Value * value, const Twine & Name) {
    IntegerType * ty = cast<IntegerType>(value->getType());
    Value * m = CreateCountReverseZeroes(CreateSub(value, ConstantInt::get(ty, 1)));
    return CreateSub(ConstantInt::get(m->getType(), ty->getBitWidth()), m, Name);
}

Value * CBuilder::CreateLog2(Value * value, const Twine & Name) {
    IntegerType * ty = cast<IntegerType>(value->getType());
    Value * m = CreateCountReverseZeroes(value);
    return CreateSub(ConstantInt::get(m->getType(), ty->getBitWidth() - 1), m, Name);
}

Constant * CBuilder::GetString(StringRef Str) {
    Module * const m = getModule();
    GlobalVariable * ptr = m->getGlobalVariable(Str, true);
    if (ptr == nullptr) {
        ptr = CreateGlobalString(Str, Str);
    }
    return ConstantExpr::getPointerCast(ptr, getInt8PtrTy());
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
        memchrFn = Function::Create(memchrTy, Function::ExternalLinkage, "memchr", m);
    }
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        CHECK_ADDRESS(ptr, num, "CreateMemChr: Src");
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
        __CreateAssert(CreateICmpULT(Idx, Size), "CreateExtractElement: Idx (%" PRISz ") is greater than Vec size (%" PRISz ")", { Idx, Size });
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
        __CreateAssert(CreateICmpULT(Idx, Size), "CreateInsertElement: Idx (%" PRISz ") is greater than Vec size (%" PRISz ")", { Idx, Size });
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

BasicBlock * CBuilder::WriteDefaultRethrowBlock() {

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

    SetInsertPoint(handleCatch);
    LandingPadInst * const caughtResult = CreateLandingPad(caughtResultType, 1);
    caughtResult->addClause(catchAny);
    caughtResult->setCleanup(false);
    Value * const exception = CreateExtractValue(caughtResult, 0);
    CallInst * beginCatch = CreateCall(getBeginCatch(), {exception});
    beginCatch->setTailCall(true);
    beginCatch->addAttribute(-1, Attribute::NoUnwind);

    InvokeInst * const rethrowInst = CreateInvoke(getRethrow(), handleUnreachable, handleRethrow);
    rethrowInst->addAttribute(-1, Attribute::NoReturn);

    SetInsertPoint(handleRethrow);
    LandingPadInst * const caughtResult2 = CreateLandingPad(caughtResultType, 1);
    caughtResult2->setCleanup(true);
    CreateInvoke(getEndCatch(), handleResume, handleExit);

    SetInsertPoint(handleResume);
    CreateResume(caughtResult2);

    SetInsertPoint(handleExit);
    LandingPadInst * const caughtResult3 = CreateLandingPad(caughtResultType, 1);
    caughtResult3->addClause(catchAny);
    Value * const exception3 = CreateExtractValue(caughtResult3, 0);
    CallInst * beginCatch2 = CreateCall(getBeginCatch(), {exception3});
    beginCatch2->setTailCall(true);
    beginCatch2->addAttribute(-1, Attribute::NoUnwind);
    // should call std::terminate
    CreateExit(-1);
    CreateBr(handleUnreachable);

    SetInsertPoint(handleUnreachable);
    CreateUnreachable();

    restoreIP(ip);

    return handleCatch;
}

Function * CBuilder::getDefaultPersonalityFunction() {
    const auto GNU_NAME = "__gxx_personality_v0";
    Module * const m = getModule();
    Function * personalityFn = m->getFunction(GNU_NAME);
    if (personalityFn == nullptr) {
        FunctionType * const fTy = FunctionType::get(getInt32Ty(), true);
        personalityFn = Function::Create(fTy, Function::ExternalLinkage, GNU_NAME, m);
    }
    return personalityFn;
}

Function * CBuilder::getAllocateException() {
    const auto GNU_NAME = "__cxa_allocate_exception";
    Module * const m = getModule();
    Function * cxa_alloc_excp = m->getFunction(GNU_NAME);
    if (cxa_alloc_excp == nullptr) {
        PointerType * const int8PtrTy = getInt8PtrTy();
        IntegerType * const int64Ty = getInt64Ty();
        FunctionType * const fTy = FunctionType::get(int8PtrTy, { int64Ty }, false);
        cxa_alloc_excp = Function::Create(fTy, Function::ExternalLinkage, GNU_NAME, m);
        cxa_alloc_excp->addFnAttr(Attribute::NoUnwind);
    }
    return cxa_alloc_excp;
}

Function * CBuilder::getThrow() {
    const auto GNU_NAME = "__cxa_throw";
    Module * const m = getModule();
    Function * cxa_throw = m->getFunction(GNU_NAME);
    if (cxa_throw == nullptr) {
        Type * const voidTy = getVoidTy();
        PointerType * const int8PtrTy = getInt8PtrTy();
        FunctionType * const fTy = FunctionType::get(voidTy, { int8PtrTy, int8PtrTy, int8PtrTy }, false);
        cxa_throw = Function::Create(fTy, Function::ExternalLinkage, GNU_NAME, m);
        cxa_throw->setDoesNotReturn();
    }
    return cxa_throw;
}

Function * CBuilder::getBeginCatch() {
    const auto GNU_NAME = "__cxa_begin_catch";
    Module * const m = getModule();
    Function * cxa_begin = m->getFunction(GNU_NAME);
    if (cxa_begin == nullptr) {
        PointerType * const int8PtrTy = getInt8PtrTy();
        FunctionType * const fTy = FunctionType::get(int8PtrTy, { int8PtrTy }, false);
        cxa_begin = Function::Create(fTy, Function::ExternalLinkage, GNU_NAME, m);
        cxa_begin->setDoesNotThrow();
        cxa_begin->setDoesNotRecurse();
    }
    return cxa_begin;
}

Function * CBuilder::getEndCatch() {
    const auto GNU_NAME = "__cxa_end_catch";
    Module * const m = getModule();
    Function * cxa_end = m->getFunction(GNU_NAME);
    if (cxa_end == nullptr) {
        FunctionType * const fTy = FunctionType::get(getVoidTy(), false);
        cxa_end = Function::Create(fTy, Function::ExternalLinkage, GNU_NAME, m);
    }
    return cxa_end;
}

Function * CBuilder::getRethrow() {
    const auto GNU_NAME = "__cxa_rethrow";
    Module * const m = getModule();
    Function * cxa_rethrow = m->getFunction(GNU_NAME);
    if (cxa_rethrow == nullptr) {
        FunctionType * const fTy = FunctionType::get(getVoidTy(), false);
        cxa_rethrow = Function::Create(fTy, Function::ExternalLinkage, GNU_NAME, m);
        cxa_rethrow->setDoesNotReturn();
    }
    return cxa_rethrow;
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

    void getAnalysisUsage(AnalysisUsage &AU) const override {
//        #ifdef HAS_ADDRESS_SANITIZER
//        AU.addRequired<DominatorTree>();
//        AU.addRequired<AliasAnalysis>();
//        AU.addPreserved<AliasAnalysis>();
//        #endif
    }

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




    CBuilder builder(M.getContext());
    builder.setModule(&M);



//    #ifdef HAS_ADDRESS_SANITIZER
//    Function * const isPoisoned = M->getFunction("__asan_region_is_poisoned");
//    DenseSet<CallInst *> alreadyTested;

//    AliasAnalysis & AA = getAnalysis<AliasAnalysis>();
//    #endif

    for (Function & F : M) {

        //DominatorTree & DT = getAnalysis<DominatorTree>(F);

        S.clear();
        // scan through each instruction and remove any trivially true or redundant assertions.
        for (auto & B : F) {
            for (auto i = B.begin(); i != B.end(); ) {
                Instruction & inst = *i;
                if (LLVM_UNLIKELY(isa<CallInst>(inst))) {
                    CallInst & ci = cast<CallInst>(inst);
                    // if we're using address sanitizer, try to determine whether we're
                    // rechecking the same address
//                    #ifdef HAS_ADDRESS_SANITIZER
//                    if (ci.getCalledFunction() == isPoisoned) {
//                        bool alreadyProven = false;
//                        ConstantInt * const length = dyn_cast<ConstantInt>(ci.getArgOperand(1));
//                        if (length) {
//                            Value * const ptr = ci.getArgOperand(0);
//                            for (CallInst * prior : alreadyTested) {
//                                if (DT.dominates(prior, &ci)) {
//                                    Value * const priorPtr = prior->getArgOperand(0);
//                                    ConstantInt * const priorLength = cast<ConstantInt>(prior->getArgOperand(1));
//                                    const auto result = AA.alias(ptr, length->getLimitedValue(), priorPtr, priorLength->getLimitedValue());
//                                    if (LLVM_UNLIKELY(result == AliasResult::MustAlias)) {
//                                        alreadyProven = true;
//                                        break;
//                                    }
//                                }
//                            }
//                        }
//                        if (alreadyProven) {
//                            PointerType * const voidPtrTy = IntegerType::getInt8PtrTy(M.getContext());
//                            i = ci.replaceAllUsesWith(ConstantPointerNull::get(voidPtrTy));
//                            modified = true;
//                        } else {
//                            alreadyTested.insert(&ci);
//                        }
//                        continue;
//                    }
//                    #endif
                    if (ci.getCalledFunction() == assertFunc) {
                        assert (ci.getNumArgOperands() >= 5);
                        bool remove = false;
                        Value * const check = ci.getOperand(0);
                        Constant * static_check = nullptr;
                        if (isa<Constant>(check)) {
                            static_check = cast<Constant>(check);
                        } else if (LLVM_LIKELY(isa<ICmpInst>(check))) {
                            ICmpInst * const icmp = cast<ICmpInst>(check);
                            Value * const op0 = icmp->getOperand(0);
                            Value * const op1 = icmp->getOperand(1);
                            if (LLVM_UNLIKELY(isa<Constant>(op0) && isa<Constant>(op1))) {
                                static_check = ConstantExpr::getICmp(icmp->getPredicate(), cast<Constant>(op0), cast<Constant>(op1));
                            }
                        }
                        if (static_check) {
                            if (LLVM_UNLIKELY(static_check->isNullValue())) {
                                // show any static failures with their compilation context
                                auto extract = [](const Value * a) -> const char * {
                                    const GlobalVariable * const g = cast<GlobalVariable>(a->stripPointerCasts());
                                    const ConstantDataArray * const d = cast<ConstantDataArray>(g->getInitializer());
                                    return d->getRawDataValues().data();
                                };
                                const char * const name = extract(ci.getOperand(1));
                                const char * const msg = extract(ci.getOperand(2));
                                const uintptr_t * const trace = reinterpret_cast<const uintptr_t *>(extract(ci.getOperand(3)));
                                const uint32_t n = cast<ConstantInt>(ci.getOperand(4))->getLimitedValue();

                                // since we may not necessarily be able to statically evaluate every varadic param,
                                // attempt to fill in what constants we can and report <any> for all others.
                                boost::format fmt(msg);
                                for (unsigned i = 5; i < ci.getNumArgOperands(); ++i) {
                                    Value * const arg = ci.getOperand(i);
                                    if (LLVM_LIKELY(isa<Constant>(arg))) {
                                        if (LLVM_LIKELY(isa<ConstantInt>(arg))) {
                                            const auto v = cast<ConstantInt>(arg)->getLimitedValue();
                                            fmt % v;
                                        } else if (isa<ConstantFP>(arg)) {
                                            const auto & f = cast<ConstantFP>(arg)->getValueAPF();
                                            fmt % f.convertToDouble();
                                        } else {
                                            Type * const ty = arg->getType();
                                            if (ty->isPointerTy()) {
                                                fmt % extract(arg);
                                            } else {
                                                fmt % "<unknown>";
                                            }
                                        }
                                    } else {
                                        fmt % "<any>";
                                    }
                                }
                                __report_failure(name, msg, trace, n);
                                exit(-1);
                            }
                            remove = true;
                        } else if (LLVM_UNLIKELY(S.count(check))) {
                            // a duplicate check will never be executed if true
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

        // if we have any runtime assertions, replace the calls to each with an invoke.
        if (LLVM_UNLIKELY(S.empty())) continue;

        // Gather all assertions from the appropriate users of S.
        SmallVector<CallInst *, 64> assertList;
        assertList.reserve(S.size());
        for (Value * s : S) {
            for (User * u : s->users()) {
                if (isa<CallInst>(u)) {
                    CallInst * const c = cast<CallInst>(u);
                    if (LLVM_LIKELY(c->getCalledFunction() == assertFunc)) {
                        assertList.push_back(c);
                        break;
                    }
                }
            }
        }
        assert (assertList.size() == S.size());
        S.clear();

        builder.SetInsertPoint(&F.front());
        BasicBlock * const rethrow = builder.WriteDefaultRethrowBlock();

        SmallVector<Value *, 16> args;

        for (CallInst * ci : assertList) {
            BasicBlock * const bb = ci->getParent();
            const auto n = ci->getNumArgOperands();
            assert (n >= 5);
            args.clear();
            for (unsigned i = 0; i < n; ++i) {
                args.push_back(ci->getArgOperand(i));
            }
            auto next = ci->eraseFromParent();
            // note: split automatically inserts an unconditional branch to the new block
            BasicBlock * const assertFinally = bb->splitBasicBlock(next, "__assert_ok");
            assert(dyn_cast_or_null<BranchInst>(bb->getTerminator()));
            bb->getTerminator()->eraseFromParent();
            builder.SetInsertPoint(bb);
            builder.CreateInvoke(assertFunc, assertFinally, rethrow, args);
            assert(dyn_cast_or_null<InvokeInst>(bb->getTerminator()));
        }
    }


    return modified;
}

std::string CBuilder::getKernelName() const {
    report_fatal_error("CBuilder does not have a default kernel name.");
}