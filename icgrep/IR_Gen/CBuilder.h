/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef CBUILDER_H
#define CBUILDER_H

#include <llvm/IR/Module.h>
#include <llvm/IR/Constant.h>
#include <llvm/IR/Type.h>
#include <llvm/IR/Value.h>
#include <llvm/IR/IRBuilder.h>
#include <llvm/Support/Host.h>
#include <llvm/ADT/Triple.h>
#include <IR_Gen/types/streamtype.h>
#include <boost/container/flat_map.hpp>

using namespace llvm;


class CBuilder : public IRBuilder<> {
    
public:
    
    CBuilder(Module * m, unsigned archBitWidth, unsigned CacheAlignment=64)
    : IRBuilder<>(m->getContext())
    , mMod(m)
    , mCacheLineAlignment(CacheAlignment)
    , mSizeType(getIntNTy(archBitWidth)) {
    }
    
    virtual ~CBuilder() {}

    Module * getModule() const {
        return mMod;
    }
    
    void setModule(Module * m)  {
        mMod = m;
    }
    
    Function * GetPrintf();
    Value * CreateMalloc(Type * type, Value * size);
    Value * CreateAlignedMalloc(Type *type, Value * size, const unsigned alignment);
    void CreateFree(Value * ptr);
    void CreateAlignedFree(Value * ptr);
    Value * CreateRealloc(Value * ptr, Value * size);
    Value * CreateAlignedRealloc(Value * ptr, Value * size, const unsigned alignment);
    void CreateMemZero(Value * ptr, Value * size, const unsigned alignment = 1);
    
    
    // Create calls to unistd.h functions.
    //
    // ssize_t write(int fildes, const void *buf, size_t nbyte);
    Value * CreateWriteCall(Value * fildes, Value * buf, Value * nbyte);
    
    // Create calls to Posix thread (pthread.h) functions.
    //
    //  int pthread_create(pthread_t *thread, const pthread_attr_t *attr,
    //                    void *(*start_routine)(void*), void *arg);
    Value * CreatePThreadCreateCall(Value * thread, Value * attr, Function * start_routine, Value * arg);
    //  void pthread_exit(void *value_ptr);
    Value * CreatePThreadExitCall(Value * value_ptr);
    //  int pthread_join(pthread_t thread, void **value_ptr);
    Value * CreatePThreadJoinCall(Value * thread, Value * value_ptr);
    
    
    void CallPrintRegister(const std::string & regName, Value * const value);
    void CallPrintInt(const std::string & name, Value * const value);
    
    inline llvm::IntegerType * getSizeTy() const {
        return mSizeType;
    }
    
    inline llvm::ConstantInt * getSize(const size_t value) const {
        return ConstantInt::get(getSizeTy(), value);
    }
    
    PointerType * getVoidPtrTy() const;
    
    inline unsigned getCacheAlignment() const {
        return mCacheLineAlignment;
    }
    
    virtual llvm::LoadInst* CreateAtomicLoadAcquire(Value * ptr);
    virtual llvm::StoreInst *  CreateAtomicStoreRelease(Value * val, Value * ptr);
    
protected:
    Module *            mMod;
    unsigned            mCacheLineAlignment;
    IntegerType *       mSizeType;
};

#endif
