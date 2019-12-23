/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#ifndef KERNEL_H
#define KERNEL_H

#include "binding_map.hpp"
#include "relationship.h"
#include "streamset.h"
#include <util/not_null.h>
#include <util/slab_allocator.h>
#include <llvm/ADT/StringRef.h>
#include <llvm/IR/Module.h>
#include <llvm/IR/Function.h>
#include <llvm/Support/Compiler.h>
#include <codegen/FunctionTypeBuilder.h>
#include <memory>
#include <string>
#include <vector>

namespace llvm { class IndirectBrInst; }
namespace llvm { class PHINode; }

class BaseDriver;

namespace kernel {

class KernelBuilder;
class KernelCompiler;
class BlockKernelCompiler;
class StreamSetBuffer;
class StreamSet;

class Kernel : public AttributeSet {
    friend class KernelCompiler;
    friend class PipelineCompiler;
    friend class PipelineKernel;
    friend class OptimizationBranchCompiler;
    friend class OptimizationBranch;
    friend class BaseDriver;
public:

    using BuilderRef = const std::unique_ptr<KernelBuilder> &;

    using KernelCompilerRef = const std::unique_ptr<KernelCompiler> &;

    enum class TypeId {
        SegmentOriented
        , MultiBlock
        , BlockOriented
        , Pipeline
        , OptimizationBranch
    };

    using InitArgs = llvm::SmallVector<llvm::Value *, 32>;

    using InitArgTypes = llvm::SmallVector<llvm::Type *, 32>;

    using ParamMap = llvm::DenseMap<Scalar *, llvm::Value *>;

    struct LinkedFunction {
        const std::string  Name;
        llvm::FunctionType * const Type;
        void * const FunctionPtr;

        LinkedFunction(std::string && Name, llvm::FunctionType * Type, void * FunctionPtr)
        : Name(Name), Type(Type), FunctionPtr(FunctionPtr) { }
    };

    using LinkedFunctions = llvm::SmallVector<LinkedFunction, 0>;

    enum MainMethodGenerationType {
        AddInternal
        , DeclareExternal
        , AddExternal
    };

    using Rational = ProcessingRate::Rational;

    static bool classof(const Kernel *) { return true; }

    static bool classof(const void *) { return false; }

    LLVM_READNONE TypeId getTypeId() const {
        return mTypeId;
    }

    enum class ScalarType { Input, Output, Internal, NonPersistent, ThreadLocal };

    struct InternalScalar {

        ScalarType getScalarType() const {
            return mScalarType;
        }

        llvm::Type * getValueType() const {
            return mValueType;
        }

        const llvm::StringRef getName() const {
            return llvm::StringRef(mName);
        }

        const unsigned getGroup() const {
            return mGroup;
        }

        explicit InternalScalar(llvm::Type * const valueType,
                                const llvm::StringRef name, const unsigned group = 1)
        : InternalScalar(ScalarType::Internal, valueType, name, group) {

        }

        explicit InternalScalar(const ScalarType scalarType, llvm::Type * const valueType,
                                const llvm::StringRef name, const unsigned group = 1)
        : mScalarType(scalarType), mValueType(valueType), mName(name.str()), mGroup(group) {

        }

    private:
        const ScalarType        mScalarType;
        llvm::Type * const      mValueType;
        const std::string       mName;
        const unsigned          mGroup;
    };

    using InternalScalars = std::vector<InternalScalar>;

    using ScalarValueMap = llvm::StringMap<llvm::Value *>;

    enum class PortType { Input, Output };

    struct StreamSetPort {
        PortType Type;
        unsigned Number;

        StreamSetPort() : Type(PortType::Input), Number(0) { }
        explicit StreamSetPort(PortType Type, unsigned Number) : Type(Type), Number(Number) { }

        bool operator < (const StreamSetPort other) const {
            if (Type == other.Type) {
                return Number < other.Number;
            } else {
                return Type == PortType::Input;
            }
        }

        bool operator == (const StreamSetPort other) const {
            return (Type == other.Type) && (Number == other.Number);
        }
    };

    // Kernel Signatures and Module IDs
    //
    // A kernel signature uniquely identifies a kernel and its full functionality.
    // In the event that a particular kernel instance is to be generated and compiled
    // to produce object code, and we have a cached kernel object code instance with
    // the same signature and targetting the same IDISA architecture, then the cached
    // object code may safely be used to avoid recompilation.
    //
    // A kernel signature is a byte string of arbitrary length.
    //
    // Kernel developers should take responsibility for designing appropriate signature
    // mechanisms that are short, inexpensive to compute and guarantee uniqueness
    // based on the semantics of the kernel.
    //
    // If no other mechanism is available, the default makeSignature() method uses the
    // full LLVM IR (before optimization) of the kernel instance.
    //
    // A kernel Module ID is short string that is used as a name for a particular kernel
    // instance.  Kernel Module IDs are used to look up and retrieve cached kernel
    // instances and so should be highly likely to uniquely identify a kernel instance.
    //
    // The ideal case is that a kernel Module ID serves as a full kernel signature thus
    // guaranteeing uniqueness.  In this case, hasSignature() should return false.
    //

    //
    // Kernel builder subtypes define their logic of kernel construction
    // in terms of 3 virtual methods for
    // (a) preparing the Kernel state data structure
    // (c) defining the logic of the finalBlock function.
    //
    // Note: the kernel state data structure must only be finalized after
    // all scalar fields have been added.   If there are no fields to
    // be added, the default method for preparing kernel state may be used.

    LLVM_READNONE virtual const std::string getName() const {
        return mKernelName;
    }

    LLVM_READNONE virtual bool hasFamilyName() const {
        return false;
    }

    LLVM_READNONE virtual bool externallyInitialized() const {
        return hasFamilyName();
    }

    LLVM_READNONE virtual const std::string getFamilyName() const {
        if (hasFamilyName()) {
            return getDefaultFamilyName();
        } else {
            return getName();
        }
    }

    virtual std::string makeSignature(BuilderRef b) const;

    virtual bool hasSignature() const { return true; }

    virtual bool isCachable() const { return false; }

    LLVM_READNONE bool isStateful() const {
        return mSharedStateType != nullptr;
    }

    LLVM_READNONE bool hasThreadLocal() const {
        return mThreadLocalStateType  != nullptr;
    }

    unsigned getStride() const { return mStride; }

    void setStride(const unsigned stride) { mStride = stride; }

    const Bindings & getInputStreamSetBindings() const {
        return mInputStreamSets;
    }

    Bindings & getInputStreamSetBindings() {
        return mInputStreamSets;
    }

    const Binding & getInputStreamSetBinding(const unsigned i) const {
        assert (i < getNumOfStreamInputs());
        return mInputStreamSets[i];
    }

    LLVM_READNONE StreamSet * getInputStreamSet(const unsigned i) const {
        return llvm::cast<StreamSet>(getInputStreamSetBinding(i).getRelationship());
    }

    LLVM_READNONE unsigned getNumOfStreamInputs() const {
        return mInputStreamSets.size();
    }

    virtual void setInputStreamSetAt(const unsigned i, StreamSet * value);

    LLVM_READNONE const Binding & getOutputStreamSetBinding(const unsigned i) const {
        assert (i < getNumOfStreamOutputs());
        return mOutputStreamSets[i];
    }

    LLVM_READNONE StreamSet * getOutputStreamSet(const unsigned i) const {
        return llvm::cast<StreamSet>(getOutputStreamSetBinding(i).getRelationship());
    }

    const Bindings & getOutputStreamSetBindings() const {
        return mOutputStreamSets;
    }

    Bindings & getOutputStreamSetBindings() {
        return mOutputStreamSets;
    }

    unsigned getNumOfStreamOutputs() const {
        return mOutputStreamSets.size();
    }

    Scalar * getInputScalarAt(const unsigned i) const {
        return llvm::cast<Scalar>(getInputScalarBinding(i).getRelationship());
    }

    virtual void setOutputStreamSetAt(const unsigned i, StreamSet * value);

    const Bindings & getInputScalarBindings() const {
        return mInputScalars;
    }

    Bindings & getInputScalarBindings() {
        return mInputScalars;
    }

    const Binding & getInputScalarBinding(const unsigned i) const {
        assert (i < mInputScalars.size());
        return mInputScalars[i];
    }

    LLVM_READNONE unsigned getNumOfScalarInputs() const {
        return mInputScalars.size();
    }

    virtual void setInputScalarAt(const unsigned i, Scalar * value);

    const Bindings & getOutputScalarBindings() const {
        return mOutputScalars;
    }

    Bindings & getOutputScalarBindings() {
        return mOutputScalars;
    }

    const Binding & getOutputScalarBinding(const unsigned i) const {
        assert (i < mOutputScalars.size());
        return mOutputScalars[i];
    }

    LLVM_READNONE unsigned getNumOfScalarOutputs() const {
        return mOutputScalars.size();
    }

    Scalar * getOutputScalarAt(const unsigned i) const {
        return llvm::cast<Scalar>(getOutputScalarBinding(i).getRelationship());
    }

    virtual void setOutputScalarAt(const unsigned i, Scalar * value);

    void addInternalScalar(llvm::Type * type, const llvm::StringRef name, const unsigned group = 1) {
        assert ("cannot modify state types after initialization" && !mSharedStateType && !mThreadLocalStateType);
        mInternalScalars.emplace_back(ScalarType::Internal, type, name, group);
    }

    void addNonPersistentScalar(llvm::Type * type, const llvm::StringRef name) {
        assert ("cannot modify state types after initialization" && !mSharedStateType && !mThreadLocalStateType);
        mInternalScalars.emplace_back(ScalarType::NonPersistent, type, name, 0);
    }

    void addThreadLocalScalar(llvm::Type * type, const llvm::StringRef name, const unsigned group = 1) {
        assert ("cannot modify state types after initialization" && !mSharedStateType && !mThreadLocalStateType);
        mInternalScalars.emplace_back(ScalarType::ThreadLocal, type, name, group);
    }

    llvm::Module * setModule(llvm::Module * const module) {
        mModule = module;
        return module;
    }

    llvm::Module * getModule() const {
        return mModule;
    }

    llvm::StructType * getSharedStateType() const {
        return mSharedStateType;
    }

    llvm::StructType * getThreadLocalStateType() const {
        return mThreadLocalStateType;
    }

    void makeModule(BuilderRef b);

    void generateKernel(BuilderRef b);

    void loadCachedKernel(BuilderRef b);

    template <typename ExternalFunctionType>
    void link(llvm::StringRef name, ExternalFunctionType & functionPtr);

    LLVM_READNONE std::string getCacheName(BuilderRef b) const;

    virtual void addKernelDeclarations(BuilderRef b);

    virtual std::unique_ptr<KernelCompiler> instantiateKernelCompiler(BuilderRef b) const noexcept;

    virtual ~Kernel() = 0;

protected:

    llvm::Function * getInitializeFunction(BuilderRef b, const bool alwayReturnDeclaration = true) const;

    llvm::Function * addInitializeDeclaration(BuilderRef b) const;

    llvm::Function * getInitializeThreadLocalFunction(BuilderRef b, const bool alwayReturnDeclaration = true) const;

    llvm::Function * addInitializeThreadLocalDeclaration(BuilderRef b) const;

    llvm::Function * addDoSegmentDeclaration(BuilderRef b) const;

    std::vector<llvm::Type *> getDoSegmentFields(BuilderRef b) const;

    llvm::Function * getDoSegmentFunction(BuilderRef b, const bool alwayReturnDeclaration = true) const;

    llvm::Function * getFinalizeThreadLocalFunction(BuilderRef b, const bool alwayReturnDeclaration = true) const;

    llvm::Function * addFinalizeThreadLocalDeclaration(BuilderRef b) const;

    llvm::Function * getFinalizeFunction(BuilderRef b, const bool alwayReturnDeclaration = true) const;

    llvm::Function * addFinalizeDeclaration(BuilderRef b) const;

public:

    llvm::Function * addOrDeclareMainFunction(BuilderRef b, const MainMethodGenerationType method);

protected:

    llvm::Value * constructFamilyKernels(BuilderRef b, InitArgs & hostArgs, const ParamMap & params) const;

    virtual void addFamilyInitializationArgTypes(BuilderRef b, InitArgTypes & argTypes) const;

    virtual void recursivelyConstructFamilyKernels(BuilderRef b, InitArgs & args, const ParamMap & params) const;

protected:

    llvm::Value * createInstance(BuilderRef b) const;

    void initializeInstance(BuilderRef b, llvm::ArrayRef<llvm::Value *> args);

    llvm::Value * finalizeInstance(BuilderRef b, llvm::Value * const handle) const;

    llvm::Value * initializeThreadLocalInstance(BuilderRef b, llvm::Value * handle);

    void finalizeThreadLocalInstance(BuilderRef b, llvm::ArrayRef<llvm::Value *> args) const;

protected:

    static std::string getStringHash(const llvm::StringRef str);

    LLVM_READNONE std::string getDefaultFamilyName() const;

    LLVM_READNONE bool canSetTerminateSignal() const;

    static bool isLocalBuffer(const Binding & output);

    static bool requiresExplicitPartialFinalStride(const Kernel * const kernel);

    LLVM_READNONE bool hasFixedRate() const;

    LLVM_READNONE Rational getFixedRateLCM() const;

    virtual void addInternalProperties(BuilderRef) { }

    virtual void addAdditionalFunctions(BuilderRef) { }

    virtual void linkExternalMethods(BuilderRef b);

    void constructStateTypes(BuilderRef b);

    virtual void generateInitializeMethod(BuilderRef) { }

    virtual void generateInitializeThreadLocalMethod(BuilderRef) { }

    virtual void generateKernelMethod(BuilderRef) = 0;

    virtual void generateFinalizeThreadLocalMethod(BuilderRef) { }

    virtual void generateFinalizeMethod(BuilderRef) { }

private:

    LLVM_READNONE bool supportsInternalSynchronization() const;

protected:

    // Constructor
    Kernel(BuilderRef b,
           const TypeId typeId, std::string && kernelName,
           Bindings &&stream_inputs, Bindings &&stream_outputs,
           Bindings &&scalar_inputs, Bindings &&scalar_outputs,
           InternalScalars && internal_scalars);

protected:

    const TypeId        mTypeId;
    unsigned            mStride;
    llvm::Module *      mModule = nullptr;
    llvm::StructType *  mSharedStateType = nullptr;
    llvm::StructType *  mThreadLocalStateType = nullptr;

    Bindings            mInputStreamSets;
    Bindings            mOutputStreamSets;
    Bindings            mInputScalars;
    Bindings            mOutputScalars;
    InternalScalars     mInternalScalars;
    const std::string   mKernelName;
    LinkedFunctions     mLinkedFunctions;
};

template <typename ExternalFunctionType>
inline void Kernel::link(llvm::StringRef name, ExternalFunctionType & functionPtr) {
    assert ("Kernel does not have a module?" && mModule);
    auto & C = mModule->getContext();
    auto * const type = FunctionTypeBuilder<ExternalFunctionType>::get(C);
    assert ("FunctionTypeBuilder did not resolve a function type." && type);
    mLinkedFunctions.emplace_back(name, type, reinterpret_cast<void *>(functionPtr));
}

class SegmentOrientedKernel : public Kernel {
public:

    static bool classof(const Kernel * const k) {
        return k->getTypeId() == TypeId::SegmentOriented;
    }

    static bool classof(const void *) { return false; }

protected:

    SegmentOrientedKernel(BuilderRef b,
                          std::string && kernelName,
                          Bindings &&stream_inputs,
                          Bindings &&stream_outputs,
                          Bindings &&scalar_parameters,
                          Bindings &&scalar_outputs,
                          InternalScalars && internal_scalars);
public:

    virtual void generateDoSegmentMethod(BuilderRef b) = 0;

protected:

    void generateKernelMethod(BuilderRef b) final;

};

class MultiBlockKernel : public Kernel {
    friend class BlockOrientedKernel;
    friend class OptimizationBranch;
public:

    static bool classof(const Kernel * const k) {
        return k->getTypeId() == TypeId::MultiBlock;
    }

    static bool classof(const void *) { return false; }

protected:

    MultiBlockKernel(BuilderRef b,
                     std::string && kernelName,
                     Bindings && stream_inputs,
                     Bindings && stream_outputs,
                     Bindings && scalar_parameters,
                     Bindings && scalar_outputs,
                     InternalScalars && internal_scalars);

    virtual void generateMultiBlockLogic(BuilderRef b, llvm::Value * const numOfStrides) = 0;

private:

    MultiBlockKernel(BuilderRef b,
                     const TypeId kernelTypId,
                     std::string && kernelName,
                     Bindings && stream_inputs,
                     Bindings && stream_outputs,
                     Bindings && scalar_parameters,
                     Bindings && scalar_outputs,
                     InternalScalars && internal_scalars);

private:

    void generateKernelMethod(BuilderRef b) final;

};


class BlockOrientedKernel : public MultiBlockKernel {
    friend class BlockKernelCompiler;
public:

    static bool classof(const Kernel * const k) {
        return k->getTypeId() == TypeId::BlockOriented;
    }

    static bool classof(const void *) { return false; }

    std::unique_ptr<KernelCompiler> instantiateKernelCompiler(BuilderRef b) const noexcept;

protected:

    // Each BlockOrientedKernel must provide its own logic for generating
    // doBlock calls.
    virtual void generateDoBlockMethod(BuilderRef b) = 0;

    // Each BlockOrientedKernel must also specify the logic for processing the
    // final block of stream data, if there is any special processing required
    // beyond simply calling the doBlock function. In the case that the final block
    // processing may be trivially implemented by dispatching to the doBlock method
    // without additional preparation, the default generateFinalBlockMethod need
    // not be overridden.

    void RepeatDoBlockLogic(BuilderRef b);

    virtual void generateFinalBlockMethod(BuilderRef b, llvm::Value * remainingItems);



    BlockOrientedKernel(BuilderRef b,
                        std::string && kernelName,
                        Bindings && stream_inputs,
                        Bindings && stream_outputs,
                        Bindings && scalar_parameters,
                        Bindings && scalar_outputs,
                        InternalScalars && internal_scalars);

private:

    void generateMultiBlockLogic(BuilderRef b, llvm::Value * const numOfStrides) final;

};

}

#endif
