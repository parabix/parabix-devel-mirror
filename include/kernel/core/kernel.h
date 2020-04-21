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
        , PopCountKernel
    };

    using InitArgs = llvm::SmallVector<llvm::Value *, 32>;

    using InitArgTypes = llvm::SmallVector<llvm::Type *, 32>;

    using ParamMap = llvm::DenseMap<const Scalar *, llvm::Value *>;

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

    enum class PortType { Input, Output };

    struct StreamSetPort {
        PortType Type;
        unsigned Number;

        StreamSetPort() : Type(PortType::Input), Number(0) { }
        StreamSetPort(const PortType Type, const unsigned Number) : Type(Type), Number(Number) { }
        StreamSetPort(const StreamSetPort & other) = default;
        StreamSetPort & operator = (const StreamSetPort & other) {
            Type = other.Type;
            Number = other.Number;
            return *this;
        }
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
    // A kernel Module ID is short string that is used as a name for a particular kernel
    // instance.  Kernel Module IDs are used to look up and retrieve cached kernel
    // instances and so should be highly likely to uniquely identify a kernel instance.
    //
    // The ideal case is that a kernel Module ID serves as a full kernel signature thus
    // guaranteeing uniqueness.  In this case, hasSignature() should return false.
    //

    LLVM_READNONE const llvm::StringRef getName() const {
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
            return getName().str();
        }
    }

    virtual bool isCachable() const { return true; }

    virtual bool hasSignature() const { return false; }

    virtual llvm::StringRef getSignature() const {
        return getName();
    }

    LLVM_READNONE bool isStateful() const {
        return mSharedStateType != nullptr;
    }

    LLVM_READNONE bool hasThreadLocal() const {
        return mThreadLocalStateType  != nullptr;
    }

    virtual bool requiresExplicitPartialFinalStride() const;

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

    void setModule(llvm::Module * const module) {
        mModule = module;
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

    bool isGenerated() const {
        return mModule != nullptr;
    }

    std::string makeCacheName(BuilderRef b);

    void makeModule(BuilderRef b);

    void ensureLoaded();

    void generateKernel(BuilderRef b);

    void loadCachedKernel(BuilderRef b);

    template <typename ExternalFunctionType>
    void link(llvm::StringRef name, ExternalFunctionType & functionPtr);

    static bool isLocalBuffer(const Binding & output);

    LLVM_READNONE bool canSetTerminateSignal() const;

    virtual void addKernelDeclarations(BuilderRef b);

    virtual std::unique_ptr<KernelCompiler> instantiateKernelCompiler(BuilderRef b) const;

    virtual ~Kernel() = 0;

protected:

    llvm::Function * getInitializeFunction(BuilderRef b, const bool alwayReturnDeclaration = true) const;

    llvm::Function * addInitializeDeclaration(BuilderRef b) const;

    LLVM_READNONE virtual bool allocatesInternalStreamSets() const;

    llvm::Function * getAllocateSharedInternalStreamSetsFunction(BuilderRef b, const bool alwayReturnDeclaration = true) const;

    llvm::Function * addAllocateSharedInternalStreamSetsDeclaration(BuilderRef b) const;

    llvm::Function * getInitializeThreadLocalFunction(BuilderRef b, const bool alwayReturnDeclaration = true) const;

    llvm::Function * addInitializeThreadLocalDeclaration(BuilderRef b) const;

    llvm::Function * getAllocateThreadLocalInternalStreamSetsFunction(BuilderRef b, const bool alwayReturnDeclaration = true) const;

    llvm::Function * addAllocateThreadLocalInternalStreamSetsDeclaration(BuilderRef b) const;

    llvm::Function * addDoSegmentDeclaration(BuilderRef b) const;

    std::vector<llvm::Type *> getDoSegmentFields(BuilderRef b) const;

    llvm::Function * getDoSegmentFunction(BuilderRef b, const bool alwayReturnDeclaration = true) const;

    llvm::Function * getFinalizeThreadLocalFunction(BuilderRef b, const bool alwayReturnDeclaration = true) const;

    llvm::Function * addFinalizeThreadLocalDeclaration(BuilderRef b) const;

    llvm::Function * getFinalizeFunction(BuilderRef b, const bool alwayReturnDeclaration = true) const;

    llvm::Function * addFinalizeDeclaration(BuilderRef b) const;

    virtual void runOptimizationPasses(BuilderRef b) const;

public:

    llvm::Function * addOrDeclareMainFunction(BuilderRef b, const MainMethodGenerationType method) const;

protected:

    llvm::Value * constructFamilyKernels(BuilderRef b, InitArgs & hostArgs, const ParamMap & params) const;

    virtual void addFamilyInitializationArgTypes(BuilderRef b, InitArgTypes & argTypes) const;

    virtual void recursivelyConstructFamilyKernels(BuilderRef b, InitArgs & args, const ParamMap & params) const;

protected:

    llvm::Value * createInstance(BuilderRef b) const;

    void initializeInstance(BuilderRef b, llvm::ArrayRef<llvm::Value *> args) const;

    llvm::Value * finalizeInstance(BuilderRef b, llvm::Value * const handle) const;

    llvm::Value * initializeThreadLocalInstance(BuilderRef b, llvm::Value * handle) const;

    void finalizeThreadLocalInstance(BuilderRef b, llvm::ArrayRef<llvm::Value *> args) const;

protected:

    static std::string getStringHash(const llvm::StringRef str);

    LLVM_READNONE std::string getDefaultFamilyName() const;

    LLVM_READNONE bool hasFixedRateInput() const;

    LLVM_READNONE bool isGreedy() const;

    virtual void addInternalProperties(BuilderRef) { }

    virtual void addAdditionalFunctions(BuilderRef) { }

    virtual void linkExternalMethods(BuilderRef b);

    void constructStateTypes(BuilderRef b);

    virtual void generateInitializeMethod(BuilderRef) { }

    virtual void generateInitializeThreadLocalMethod(BuilderRef) { }

    virtual void generateAllocateSharedInternalStreamSetsMethod(BuilderRef b, llvm::Value * expectedNumOfStrides);

    virtual void generateAllocateThreadLocalInternalStreamSetsMethod(BuilderRef b, llvm::Value * expectedNumOfStrides);

    virtual void generateKernelMethod(BuilderRef) = 0;

    virtual void generateFinalizeThreadLocalMethod(BuilderRef) { }

    virtual void generateFinalizeMethod(BuilderRef) { }

protected:

    // Constructor
    Kernel(BuilderRef b,
           const TypeId typeId, std::string && kernelName,
           Bindings &&stream_inputs, Bindings &&stream_outputs,
           Bindings &&scalar_inputs, Bindings &&scalar_outputs,
           InternalScalars && internal_scalars);

protected:

    const TypeId                mTypeId;
    unsigned                    mStride;
    llvm::Module *              mModule = nullptr;
    llvm::StructType *          mSharedStateType = nullptr;
    llvm::StructType *          mThreadLocalStateType = nullptr;
    bool                        mGenerated = false;

    Bindings                    mInputStreamSets;
    Bindings                    mOutputStreamSets;
    Bindings                    mInputScalars;
    Bindings                    mOutputScalars;
    InternalScalars             mInternalScalars;
    const std::string           mKernelName;
    LinkedFunctions             mLinkedFunctions;
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

    MultiBlockKernel(BuilderRef b,
                     const TypeId kernelTypId,
                     std::string && kernelName,
                     Bindings && stream_inputs,
                     Bindings && stream_outputs,
                     Bindings && scalar_parameters,
                     Bindings && scalar_outputs,
                     InternalScalars && internal_scalars);

    virtual void generateMultiBlockLogic(BuilderRef b, llvm::Value * const numOfStrides) = 0;

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

    std::unique_ptr<KernelCompiler> instantiateKernelCompiler(BuilderRef b) const;

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
