/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef CBUILDER_H
#define CBUILDER_H

#include <string>
#include <llvm/IR/IRBuilder.h>
#include <llvm/IR/Constants.h>
namespace llvm { class Function; }
namespace llvm { class IntegerType; }
namespace llvm { class Module; }
namespace llvm { class PointerType; }
namespace llvm { class Type; }
namespace llvm { class Value; }

class CBuilder : public llvm::IRBuilder<> {
    
public:
    
    CBuilder(llvm::Module * m, const unsigned GeneralRegisterWidthInBits, const bool SupportsIndirectBr, const unsigned CacheLineAlignmentInBytes = 64);
    
    virtual ~CBuilder() {}

    llvm::Module * getModule() const {
        return mMod;
    }
    
    void setModule(llvm::Module * m)  {
        mMod = m;
    }
    
    llvm::Value * CreateMalloc(llvm::Type * type, llvm::Value * size);

    llvm::Value * CreateAlignedMalloc(llvm::Type * type, llvm::Value * size, const unsigned alignment);
    
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
    
    llvm::Function * GetPrintf();

    //  Create calls to unistd.h functions.
    //
    //  Create a call to:  int open(const char *filename, int oflag, ...);
    llvm::Value * CreateOpenCall(llvm::Value * filename, llvm::Value * oflag, llvm::Value * mode);
    //  Create a call to:  ssize_t write(int fildes, const void *buf, size_t nbyte);
    llvm::Value * CreateWriteCall(llvm::Value * fildes, llvm::Value * buf, llvm::Value * nbyte);
    //  Create a call to:  ssize_t read(int fildes, void *buf, size_t nbyte);
    llvm::Value * CreateReadCall(llvm::Value * fildes, llvm::Value * buf, llvm::Value * nbyte);
    //  Create a call to:  int close(int filedes);
    llvm::Value * CreateCloseCall(llvm::Value * fildes);

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
    
    inline llvm::IntegerType * getSizeTy() const {
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

protected:
    llvm::Module *      mMod;
    unsigned            mCacheLineAlignment;
    llvm::IntegerType * mSizeType;
    llvm::StructType *  mFILEtype;
    const bool          mSupportsIndirectBr;
};

#endif
