/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef CBUILDER_H
#define CBUILDER_H

#include <IR_Gen/FunctionTypeBuilder.h>
#include <llvm/IR/IRBuilder.h>
#include <llvm/IR/Constants.h>
#include <llvm/ADT/Triple.h>
#ifndef NDEBUG
#include <llvm/IR/Function.h>
#endif

namespace kernels { class KernelBuilder; }
namespace llvm { class Function; }
namespace llvm { class IntegerType; }
namespace llvm { class Module; }
namespace llvm { class PointerType; }
namespace llvm { class Type; }
namespace llvm { class Value; }

class Driver;

class CBuilder : public llvm::IRBuilder<> {
    using Predicate = llvm::CmpInst::Predicate;
public:

    CBuilder(llvm::LLVMContext & C);
    
    virtual ~CBuilder() {}

    llvm::Module * getModule() const {
        #ifndef NDEBUG
        llvm::BasicBlock * const bb = GetInsertBlock();
        if (bb) {
            llvm::Function * const f = bb->getParent();
            assert ("CBuilder has an insert point that is not contained within a Function" && f);
            assert ("CBuilder module differs from insertion point module" && (mModule == f->getParent()));
        }
        #endif
        return mModule;
    }

    void setModule(llvm::Module * module) {
        mModule = module;
        ClearInsertionPoint();
    }
    
    // UDiv and URem with optimization for division by power-of-2 constants
    llvm::Value * CreateUDiv(llvm::Value * number, llvm::Value * divisor, const llvm::Twine &Name = "");
    llvm::Value * CreateURem(llvm::Value * number, llvm::Value * divisor, const llvm::Twine &Name = "");

    // Division with rounding up to the ceiling
    // Equivalent to CreateUDiv(CreateAdd(number, CreateSub(divisor, ConstantInt::get(divisor->getType(), 1))), divisor)
    llvm::Value * CreateUDivCeil(llvm::Value * number, llvm::Value * divisor, const llvm::Twine &Name = "");
    
    // Round up to a multiple of divisor.
    llvm::Value * CreateRoundUp(llvm::Value * number, llvm::Value * divisor, const llvm::Twine &Name = "");
            
    // Get minimum of two unsigned numbers
    llvm::Value * CreateUMin(llvm::Value * const a, llvm::Value * const b) {
        if (a == nullptr) return b;
        if (b == nullptr) return a;
        assert (a->getType() == b->getType());
        return CreateSelect(CreateICmpULT(a, b), a, b);
    }

    // Get minimum of two signed numbers
    llvm::Value * CreateSMin(llvm::Value * const a, llvm::Value * const b) {
        if (a == nullptr) return b;
        if (b == nullptr) return a;
        assert (a->getType() == b->getType());
        return CreateSelect(CreateICmpSLT(a, b), a, b);
    }

    // Get maximum of two unsigned numbers
    llvm::Value * CreateUMax(llvm::Value * const a, llvm::Value * const b) {
        if (a == nullptr) return b;
        if (b == nullptr) return a;
        assert (a->getType() == b->getType());
        return CreateSelect(CreateICmpUGT(a, b), a, b);
    }

    // Get maximum of two signed numbers
    llvm::Value * CreateSMax(llvm::Value * const a, llvm::Value * const b) {
        if (a == nullptr) return b;
        if (b == nullptr) return a;
        assert (a->getType() == b->getType());
        return CreateSelect(CreateICmpSGT(a, b), a, b);
    }

    llvm::Value * CreateMalloc(llvm::Value * const size);

    llvm::Value * CreateAlignedMalloc(llvm::Value * const size, const unsigned alignment);

    llvm::Value * CreateCacheAlignedMalloc(llvm::Value * const size) {
        return CreateAlignedMalloc(size, getCacheAlignment());
    }
    
    void CreateFree(llvm::Value * const ptr);

    llvm::Value * CreateRealloc(llvm::Value * const ptr, llvm::Value * const size);

    llvm::CallInst * CreateMemZero(llvm::Value * const ptr, llvm::Value * const size, const unsigned alignment = 1) {
        return CreateMemSet(ptr, getInt8(0), size, alignment);
    }

    llvm::AllocaInst * CreateAlignedAlloca(llvm::Type * const Ty, const unsigned alignment, llvm::Value * const ArraySize = nullptr) {
        llvm::AllocaInst * instr = CreateAlloca(Ty, ArraySize);
        instr->setAlignment(alignment);
        return instr;
    }

    llvm::AllocaInst * CreateCacheAlignedAlloca(llvm::Type * const Ty, llvm::Value * const ArraySize = nullptr) {
        return CreateAlignedAlloca(Ty, getCacheAlignment(), ArraySize);
    }

    // stdio.h functions
    //
    //  Create a call to:  FILE * fopen(const char *filename, const char *mode);
    llvm::Value * CreateFOpenCall(llvm::Value * filename, llvm::Value * mode);
    //  Create a call to:  size_t fread(void *ptr, size_t size, size_t nitems, FILE *stream);
    llvm::Value * CreateFReadCall(llvm::Value * ptr, llvm::Value * size, llvm::Value * nitems, llvm::Value * stream);
    //  Create a call to:  size_t fwrite(const void *ptr, size_t size, size_t nitems, FILE *stream));
    llvm::Value * CreateFWriteCall(llvm::Value * ptr, llvm::Value * size, llvm::Value * nitems, llvm::Value * stream);
    //  Create a call to:  int fclose ( FILE * stream );
    llvm::Value * CreateFCloseCall(llvm::Value * stream);
    //  Create a call to:  int remove(const char *path);
    llvm::Value * CreateRemoveCall(llvm::Value * path);
    
    //  Create a call to:  int rename(const char *old, const char *new);
    llvm::Value * CreateRenameCall(llvm::Value * oldName, llvm::Value * newName);
    
    llvm::Function * GetPrintf();

    llvm::Function * GetDprintf();

    //  Create calls to unistd.h functions.
    //
    //  Create a call to:  int open(const char *filename, int oflag, ...);
    llvm::Value * CreateOpenCall(llvm::Value * filename, llvm::Value * oflag, llvm::Value * mode);
    //  Create a call to:  ssize_t write(int fildes, const void *buf, size_t nbyte);
    llvm::Value * CreateWriteCall(llvm::Value * fileDescriptor, llvm::Value * buf, llvm::Value * nbyte);
    //  Create a call to:  ssize_t read(int fildes, void *buf, size_t nbyte);
    llvm::Value * CreateReadCall(llvm::Value * fileDescriptor, llvm::Value * buf, llvm::Value * nbyte);
    //  Create a call to:  int close(int filedes);
    llvm::Value * CreateCloseCall(llvm::Value * fileDescriptor);
    //  Create a call to:  int unlink(const char *path);
    llvm::Value * CreateUnlinkCall(llvm::Value * path);

    llvm::Value * CreateFileSize(llvm::Value * fileDescriptor);

    //  Create calls to stdlib.h functions.
    //
    //  Create a call to:  int mkstemp (char *template);
    llvm::Value * CreateMkstempCall(llvm::Value * ftemplate);
    
    //  Create a call to:  size_t strlen(const char *str);
    llvm::Value * CreateStrlenCall(llvm::Value * str);
    
    llvm::Value * CreateAnonymousMMap(llvm::Value * size);

    llvm::Value * CreateFileSourceMMap(llvm::Value * fd, llvm::Value * size);

    enum Advice {
        ADVICE_NORMAL
        , ADVICE_RANDOM
        , ADVICE_SEQUENTIAL
        , ADVICE_WILLNEED
        , ADVICE_DONTNEED
    };

    llvm::Value * CreateMAdvise(llvm::Value * addr, llvm::Value * length, Advice advice);

    llvm::Value * CreateMMap(llvm::Value * const addr, llvm::Value * size, llvm::Value * const prot, llvm::Value * const flags, llvm::Value * const fd, llvm::Value * const offset);

    llvm::Value * CreateMRemap(llvm::Value * addr, llvm::Value * oldSize, llvm::Value * newSize);

    llvm::Value * CreateMUnmap(llvm::Value * addr, llvm::Value * size);

    enum Protect {
        NONE = 0
        , READ = 1
        , WRITE = 2
        , EXEC = 4
    };

    llvm::Value * CreateMProtect(llvm::Value * addr, llvm::Value * size, int protect);

    //  Posix thread (pthread.h) functions.
    //
    //  Create a call to:  int pthread_create(pthread_t *thread, const pthread_attr_t *attr,
    //                    void *(*start_routine)(void*), void *arg);
    llvm::Value * CreatePThreadCreateCall(llvm::Value * thread, llvm::Value * attr, llvm::Function * start_routine, llvm::Value * arg);

    //  Create a call to:  int pthread_yield(void);
    llvm::Value * CreatePThreadYield();
    
    //  Create a call to:  void pthread_exit(void *value_ptr);
    llvm::Value * CreatePThreadExitCall(llvm::Value * value_ptr);
    
    //  Create a call to:  int pthread_join(pthread_t thread, void **value_ptr);
    llvm::Value * CreatePThreadJoinCall(llvm::Value * thread, llvm::Value * value_ptr);
    
    void CallPrintInt(const std::string & name, llvm::Value * const value);
    
    void CallPrintIntToStderr(const std::string & name, llvm::Value * const value);
    
    llvm::Value * GetString(llvm::StringRef Str);

    void CallPrintMsgToStderr(const std::string & message);

    inline llvm::IntegerType * getSizeTy() const {
        assert (mSizeType);
        return mSizeType;
    }
    
    inline llvm::ConstantInt * getSize(const size_t value) {
        return llvm::ConstantInt::get(getSizeTy(), value);
    }
    
    llvm::IntegerType * LLVM_READNONE getIntAddrTy() const;
    
    llvm::PointerType * LLVM_READNONE getVoidPtrTy(const unsigned AddressSpace = 0) const;
    
    llvm::PointerType * LLVM_READNONE getFILEptrTy();
    
    inline unsigned getCacheAlignment() const {
        return mCacheLineAlignment;
    }
    
    virtual llvm::LoadInst* CreateAtomicLoadAcquire(llvm::Value * ptr);

    virtual llvm::StoreInst *  CreateAtomicStoreRelease(llvm::Value * val, llvm::Value * ptr);

    void CreateAssert(llvm::Value * assertion, const llvm::Twine & failureMessage) {
        if (LLVM_UNLIKELY(assertion->getType()->isVectorTy())) {
            assertion = CreateBitCast(assertion, getIntNTy(assertion->getType()->getPrimitiveSizeInBits()));
        }
        return __CreateAssert(CreateIsNotNull(assertion), failureMessage);
    }

    void CreateAssertZero(llvm::Value * assertion, const llvm::Twine & failureMessage) {
        if (LLVM_UNLIKELY(assertion->getType()->isVectorTy())) {
            assertion = CreateBitCast(assertion, getIntNTy(assertion->getType()->getPrimitiveSizeInBits()));
        }
        return __CreateAssert(CreateIsNull(assertion), failureMessage);
    }

    void CreateExit(const int exitCode);

    llvm::BranchInst * CreateLikelyCondBr(llvm::Value * Cond, llvm::BasicBlock * True, llvm::BasicBlock * False, const int probability = 90);

    llvm::BranchInst * CreateUnlikelyCondBr(llvm::Value * Cond, llvm::BasicBlock * True, llvm::BasicBlock * False, const int probability = 90) {
        return CreateLikelyCondBr(Cond, True, False, 100 - probability);
    }

    llvm::BasicBlock * CreateBasicBlock(const llvm::StringRef name = "", llvm::BasicBlock * insertBefore = nullptr);

    virtual bool supportsIndirectBr() const;

    llvm::Value * CreatePopcount(llvm::Value * bits);

    // TODO: AVX512 offers these as vector instructions
    llvm::Value * CreateCountForwardZeroes(llvm::Value * value, const bool guaranteedNonZero = false);
    llvm::Value * CreateCountReverseZeroes(llvm::Value * value, const bool guaranteedNonZero = false);
    
    // Useful bit manipulation operations  
    llvm::Value * CreateResetLowestBit(llvm::Value * bits);   
    
    llvm::Value * CreateIsolateLowestBit(llvm::Value * bits);
    
    llvm::Value * CreateMaskToLowestBitInclusive(llvm::Value * bits);
    
    llvm::Value * CreateMaskToLowestBitExclusive(llvm::Value * bits);
    
    llvm::Value * CreateExtractBitField(llvm::Value * bits, llvm::Value * start, llvm::Value * length);
    
    llvm::Value * CreateCeilLog2(llvm::Value * value);
    
    llvm::Value * CreateReadCycleCounter();

    template <typename ExternalFunctionType>
    llvm::Function * LinkFunction(llvm::StringRef name, ExternalFunctionType * functionPtr) const;

    virtual llvm::LoadInst * CreateLoad(llvm::Value * Ptr, const char * Name);

    virtual llvm::LoadInst * CreateLoad(llvm::Value * Ptr, const llvm::Twine & Name = "");

    virtual llvm::LoadInst * CreateLoad(llvm::Type * Ty, llvm::Value * Ptr, const llvm::Twine & Name = "");

    virtual llvm::LoadInst * CreateLoad(llvm::Value * Ptr, bool isVolatile, const llvm::Twine & Name = "");

    virtual llvm::StoreInst * CreateStore(llvm::Value * Val, llvm::Value * Ptr, bool isVolatile = false);

    llvm::LoadInst * CreateAlignedLoad(llvm::Value * Ptr, unsigned Align, const char * Name);

    llvm::LoadInst * CreateAlignedLoad(llvm::Value * Ptr, unsigned Align, const llvm::Twine & Name = "");

    llvm::LoadInst * CreateAlignedLoad(llvm::Value * Ptr, unsigned Align, bool isVolatile, const llvm::Twine & Name = "");

    llvm::StoreInst * CreateAlignedStore(llvm::Value * Val, llvm::Value * Ptr, unsigned Align, bool isVolatile = false);

    llvm::CallInst * CreateMemMove(llvm::Value *Dst, llvm::Value *Src, uint64_t Size, unsigned Align,
                            bool isVolatile = false, llvm::MDNode *TBAATag = nullptr,
                            llvm::MDNode *ScopeTag = nullptr,
                            llvm::MDNode *NoAliasTag = nullptr) {
        return CreateMemMove(Dst, Src, getInt64(Size), Align, isVolatile, TBAATag, ScopeTag, NoAliasTag);
    }

    llvm::CallInst * CreateMemMove(llvm::Value *Dst, llvm::Value *Src, llvm::Value *Size, unsigned Align,
                            bool isVolatile = false, llvm::MDNode *TBAATag = nullptr,
                            llvm::MDNode *ScopeTag = nullptr,
                            llvm::MDNode *NoAliasTag = nullptr);

    llvm::CallInst * CreateMemCpy(llvm::Value *Dst, llvm::Value *Src, uint64_t Size, unsigned Align,
                           bool isVolatile = false, llvm::MDNode *TBAATag = nullptr,
                           llvm::MDNode *TBAAStructTag = nullptr,
                           llvm::MDNode *ScopeTag = nullptr,
                           llvm::MDNode *NoAliasTag = nullptr) {
        return CreateMemCpy(Dst, Src, getInt64(Size), Align, isVolatile, TBAATag, TBAAStructTag, ScopeTag, NoAliasTag);
    }

    llvm::CallInst * CreateMemCpy(llvm::Value *Dst, llvm::Value *Src, llvm::Value *Size, unsigned Align,
                           bool isVolatile = false, llvm::MDNode *TBAATag = nullptr,
                           llvm::MDNode *TBAAStructTag = nullptr,
                           llvm::MDNode *ScopeTag = nullptr,
                           llvm::MDNode *NoAliasTag = nullptr);

    llvm::CallInst * CreateMemSet(llvm::Value *Ptr, llvm::Value *Val, uint64_t Size, unsigned Align,
                           bool isVolatile = false, llvm::MDNode *TBAATag = nullptr,
                           llvm::MDNode *ScopeTag = nullptr,
                           llvm::MDNode *NoAliasTag = nullptr) {
        return CreateMemSet(Ptr, Val, getInt64(Size), Align, isVolatile, TBAATag, ScopeTag, NoAliasTag);
    }

    llvm::CallInst * CreateMemSet(llvm::Value *Ptr, llvm::Value *Val, llvm::Value *Size, unsigned Align,
                           bool isVolatile = false, llvm::MDNode *TBAATag = nullptr,
                           llvm::MDNode *ScopeTag = nullptr,
                           llvm::MDNode *NoAliasTag = nullptr);

    void setDriver(Driver * const driver) {
        mDriver = driver;
    }

protected:

    bool hasAlignedAlloc() const;

    bool hasPosixMemalign() const;

    bool hasAddressSanitizer() const;

    void __CreateAssert(llvm::Value * assertion, const llvm::Twine & failureMessage);

    llvm::Function * LinkFunction(llvm::StringRef name, llvm::FunctionType * type, void * functionPtr) const;

protected:

    llvm::Module *                  mModule;
    unsigned                        mCacheLineAlignment;
    llvm::IntegerType * const       mSizeType;
    llvm::StructType *              mFILEtype;
    Driver *                        mDriver;    
    llvm::LLVMContext               mContext;
    const std::string               mTriple;
};

template <typename ExternalFunctionType>
llvm::Function *CBuilder::LinkFunction(llvm::StringRef name, ExternalFunctionType * functionPtr) const {
    llvm::FunctionType * const type = FunctionTypeBuilder<ExternalFunctionType>::get(getContext());
    assert ("FunctionTypeBuilder did not resolve a function type." && type);
    return LinkFunction(name, type, reinterpret_cast<void *>(functionPtr));
}

#endif
