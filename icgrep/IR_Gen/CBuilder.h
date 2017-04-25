/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef CBUILDER_H
#define CBUILDER_H

#include <IR_Gen/FunctionTypeBuilder.h>
#include <llvm/IR/IRBuilder.h>
#include <llvm/IR/Constants.h>
#include <llvm/ADT/Triple.h>

namespace kernels { class KernelBuilder; }
namespace llvm { class Function; }
namespace llvm { class IntegerType; }
namespace llvm { class Module; }
namespace llvm { class PointerType; }
namespace llvm { class Type; }
namespace llvm { class Value; }

class ParabixDriver;

class CBuilder : public llvm::IRBuilder<> {
    friend class ParabixDriver;
public:

    CBuilder(llvm::Module * const module, const unsigned GeneralRegisterWidthInBits, const bool SupportsIndirectBr, const unsigned CacheLineAlignmentInBytes = 64);
    
    virtual ~CBuilder() {}

    llvm::Module * getModule() const {
        return mMod;
    }
    
    void setModule(llvm::Module * m) {
        assert (m);
        mMod = m;
    }


    llvm::Value * CreateMalloc(llvm::Value * size);

    llvm::Value * CreateAlignedMalloc(llvm::Value * size, const unsigned alignment);
    
    void CreateFree(llvm::Value * const ptr);
    
    void CreateAlignedFree(llvm::Value * const ptr, const bool testForNullAddress = false);
    
    llvm::Value * CreateRealloc(llvm::Value * ptr, llvm::Value * size);

    llvm::CallInst * CreateMemZero(llvm::Value * ptr, llvm::Value * size, const unsigned alignment = 1) {
        return CreateMemSet(ptr, getInt8(0), size, alignment);
    }

    llvm::AllocaInst * CreateCacheAlignedAlloca(llvm::Type * Ty, llvm::Value * ArraySize = nullptr) {
        llvm::AllocaInst * instr = CreateAlloca(Ty, ArraySize);
        instr->setAlignment(getCacheAlignment());
        return instr;
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

    llvm::Value * CheckMMapSuccess(llvm::Value * const addr);

    llvm::Value * CreateMRemap(llvm::Value * addr, llvm::Value * oldSize, llvm::Value * newSize);

    llvm::Value * CreateMUnmap(llvm::Value * addr, llvm::Value * size);

    //  Posix thread (pthread.h) functions.
    //
    //  Create a call to:  int pthread_create(pthread_t *thread, const pthread_attr_t *attr,
    //                    void *(*start_routine)(void*), void *arg);
    llvm::Value * CreatePThreadCreateCall(llvm::Value * thread, llvm::Value * attr, llvm::Function * start_routine, llvm::Value * arg);
    
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
    
    llvm::PointerType * getVoidPtrTy() const;
    llvm::PointerType * getFILEptrTy();
    
    inline unsigned getCacheAlignment() const {
        return mCacheLineAlignment;
    }
    
    virtual llvm::LoadInst* CreateAtomicLoadAcquire(llvm::Value * ptr);

    virtual llvm::StoreInst *  CreateAtomicStoreRelease(llvm::Value * val, llvm::Value * ptr);

    void CreateAssert(llvm::Value * assertion, llvm::StringRef failureMessage);

    void CreateExit(const int exitCode);

    llvm::BranchInst * CreateLikelyCondBr(llvm::Value * Cond, llvm::BasicBlock * True, llvm::BasicBlock * False, const int probability = 90);

    llvm::BranchInst * CreateUnlikelyCondBr(llvm::Value * Cond, llvm::BasicBlock * True, llvm::BasicBlock * False, const int probability = 90) {
        return CreateLikelyCondBr(Cond, True, False, 100 - probability);
    }

    bool supportsIndirectBr() const {
        return mSupportsIndirectBr;
    }

    llvm::Value * CreatePopcount(llvm::Value * bits);

    llvm::Value * CreateCountForwardZeroes(llvm::Value * value);

    llvm::Value * CreateCountReverseZeroes(llvm::Value * value);

    llvm::Value * CreateCeilLog2(llvm::Value * value);
    
    llvm::Value * CreateReadCycleCounter();

    template <typename ExternalFunctionType>
    llvm::Function * LinkFunction(llvm::StringRef name, ExternalFunctionType * functionPtr) const;

protected:

    llvm::Function * LinkFunction(llvm::StringRef name, llvm::FunctionType * type, void * functionPtr) const;

    void setDriver(ParabixDriver * driver) {
        mDriver = driver;
    }

protected:
    llvm::Module *                  mMod;    
    unsigned                        mCacheLineAlignment;
    llvm::IntegerType *             mSizeType;
    llvm::StructType *              mFILEtype;
    const bool                      mSupportsIndirectBr;
    ParabixDriver *                 mDriver;
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
