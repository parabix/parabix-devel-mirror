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
#include <llvm/ADT/StringRef.h>
#include <llvm/Support/Compiler.h>
#include <memory>
#include <string>
#include <vector>

namespace llvm { class AllocaInst; }
namespace llvm { class BasicBlock; }
namespace llvm { class CallInst; }
namespace llvm { class Constant; }
namespace llvm { class Function; }
namespace llvm { class IntegerType; }
namespace llvm { class IndirectBrInst; }
namespace llvm { class Module; }
namespace llvm { class PHINode; }
namespace llvm { class StructType; }
namespace llvm { class LoadInst; }
namespace llvm { class Type; }
namespace llvm { class Value; }

class BaseDriver;

const static std::string BUFFER_HANDLE_SUFFIX = "_buffer";

namespace kernel {

class KernelBuilder;
class StreamSetBuffer;
class StreamSet;

class Kernel : public AttributeSet {
    friend class KernelBuilder;
    friend class PipelineBuilder;
    friend class PipelineCompiler;
    friend class PipelineKernel;
    friend class OptimizationBranch;
    friend class OptimizationBranchCompiler;
    friend class BaseDriver;
public:

    enum class TypeId {
        SegmentOriented
        , MultiBlock
        , BlockOriented
        , Pipeline
        , OptimizationBranch
    };

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

        const std::string & getName() const {
            return mName;
        }

        explicit InternalScalar(llvm::Type * const valueType, const llvm::StringRef name)
        : InternalScalar(ScalarType::Internal, valueType, name) {

        }

        explicit InternalScalar(const ScalarType scalarType, llvm::Type * const valueType, const llvm::StringRef name)
        : mScalarType(scalarType), mValueType(valueType), mName(name.str()) {

        }

    private:
        const ScalarType        mScalarType;
        llvm::Type * const      mValueType;
        const std::string       mName;
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

    LLVM_READNONE virtual const std::string getFamilyName() const {
        if (hasFamilyName()) {
            return getDefaultFamilyName();
        } else {
            return getName();
        }
    }

    virtual std::string makeSignature(const std::unique_ptr<KernelBuilder> & b);

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

    LLVM_READNONE const Binding & getInputStreamSetBinding(const llvm::StringRef name) const {
        return getInputStreamSetBinding(getBinding(BindingType::StreamInput, name).Index);
    }

    LLVM_READNONE StreamSet * getInputStreamSet(const unsigned i) const {
        return llvm::cast<StreamSet>(getInputStreamSetBinding(i).getRelationship());
    }

    LLVM_READNONE StreamSet * getInputStreamSet(const llvm::StringRef name) const {
        return llvm::cast<StreamSet>(getInputStreamSetBinding(name).getRelationship());
    }

    void setInputStreamSet(const llvm::StringRef name, StreamSet * value) {
        setInputStreamSetAt(getBinding(BindingType::StreamInput, name).Index, value);
    }

    LLVM_READNONE unsigned getNumOfStreamInputs() const {
        return mInputStreamSets.size();
    }

    LLVM_READNONE const OwnedStreamSetBuffers & getInputStreamSetBuffers() const {
        return mStreamSetInputBuffers;
    }

    LLVM_READNONE StreamSetBuffer * getInputStreamSetBuffer(const unsigned i) const {
        assert (i < mStreamSetInputBuffers.size());
        assert (mStreamSetInputBuffers[i]);
        return mStreamSetInputBuffers[i].get();
    }

    LLVM_READNONE StreamSetBuffer * getInputStreamSetBuffer(const llvm::StringRef name) const {
        return getInputStreamSetBuffer(getBinding(BindingType::StreamInput, name).Index);
    }

    LLVM_READNONE const Binding & getOutputStreamSetBinding(const unsigned i) const {
        assert (i < getNumOfStreamOutputs());
        return mOutputStreamSets[i];
    }

    LLVM_READNONE const Binding & getOutputStreamSetBinding(const llvm::StringRef name) const {
        return getOutputStreamSetBinding(getBinding(BindingType::StreamOutput, name).Index);
    }

    LLVM_READNONE StreamSet * getOutputStreamSet(const unsigned i) const {
        return llvm::cast<StreamSet>(getOutputStreamSetBinding(i).getRelationship());
    }

    LLVM_READNONE StreamSet * getOutputStreamSet(const llvm::StringRef name) const {
        return llvm::cast<StreamSet>(getOutputStreamSetBinding(name).getRelationship());
    }

    void setOutputStreamSet(const llvm::StringRef name, StreamSet * value) {
        setOutputStreamSetAt(getBinding(BindingType::StreamOutput, name).Index, value);
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

    LLVM_READNONE const OwnedStreamSetBuffers & getOutputStreamSetBuffers() const {
        return mStreamSetOutputBuffers;
    }

    LLVM_READNONE StreamSetBuffer * getOutputStreamSetBuffer(const unsigned i) const {
        assert (i < mStreamSetOutputBuffers.size());
        assert (mStreamSetOutputBuffers[i]);
        return mStreamSetOutputBuffers[i].get();
    }

    LLVM_READNONE StreamSetBuffer * getOutputStreamSetBuffer(const llvm::StringRef name) const {
        return getOutputStreamSetBuffer(getBinding(BindingType::StreamOutput, name).Index);
    }

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

    LLVM_READNONE const Binding & getInputScalarBinding(const llvm::StringRef name) const {
        return getInputScalarBinding(getBinding(BindingType::ScalarInput, name).Index);
    }

    LLVM_READNONE Scalar * getInputScalar(const unsigned i) {
        return llvm::cast<Scalar>(getInputScalarBinding(i).getRelationship());
    }

    LLVM_READNONE Scalar * getInputScalar(const llvm::StringRef name) {
        return llvm::cast<Scalar>(getInputScalarBinding(name).getRelationship());
    }

    LLVM_READNONE unsigned getNumOfScalarInputs() const {
        return mInputScalars.size();
    }

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

    LLVM_READNONE const Binding & getOutputScalarBinding(const llvm::StringRef name) const {
        return getOutputScalarBinding(getBinding(BindingType::ScalarOutput, name).Index);
    }

    LLVM_READNONE Scalar * getOutputScalar(const llvm::StringRef name) {
        return llvm::cast<Scalar>(getOutputScalarBinding(name).getRelationship());
    }

    LLVM_READNONE Scalar * getOutputScalar(const unsigned i) {
        return llvm::cast<Scalar>(getOutputScalarBinding(i).getRelationship());
    }


    LLVM_READNONE unsigned getNumOfScalarOutputs() const {
        return mOutputScalars.size();
    }

    void addInternalScalar(llvm::Type * type, const llvm::StringRef name) {
        mInternalScalars.emplace_back(ScalarType::Internal, type, name);
    }

    void addNonPersistentScalar(llvm::Type * type, const llvm::StringRef name) {
        mInternalScalars.emplace_back(ScalarType::NonPersistent, type, name);
    }

    void addThreadLocalScalar(llvm::Type * type, const llvm::StringRef name) {
        mInternalScalars.emplace_back(ScalarType::ThreadLocal, type, name);
    }

    llvm::Value * getHandle() const {
        return mSharedHandle;
    }

    void setHandle(const std::unique_ptr<KernelBuilder> & b, llvm::Value * const handle) const;

    llvm::Value * getThreadLocalHandle() const {
        return mThreadLocalHandle;
    }

    void setThreadLocalHandle(llvm::Value * const handle) const {
        mThreadLocalHandle = handle;
    }

    llvm::Module * setModule(llvm::Module * const module);

    llvm::Module * getModule() const {
        return mModule;
    }

    llvm::StructType * getSharedStateType() const {
        return mSharedStateType;
    }

    llvm::StructType * getThreadLocalStateType() const {
        return mThreadLocalStateType;
    }

    LLVM_READNONE const StreamSetBuffer * getStreamSetBuffer(const llvm::StringRef name) const {
        const auto port = getStreamPort(name);
        if (port.Type == PortType::Input) {
            return getInputStreamSetBuffer(port.Number);
        } else {
            return getOutputStreamSetBuffer(port.Number);
        }
    }

    llvm::Module * makeModule(const std::unique_ptr<KernelBuilder> & b);

    // Add ExternalLinkage method declarations for the kernel to a given client module.
    virtual void addKernelDeclarations(const std::unique_ptr<KernelBuilder> & b);

    llvm::Value * createInstance(const std::unique_ptr<KernelBuilder> & b) const;

    virtual void initializeInstance(const std::unique_ptr<KernelBuilder> & b, llvm::ArrayRef<llvm::Value *> args);

    llvm::Value * finalizeInstance(const std::unique_ptr<KernelBuilder> & b, llvm::Value * const handle) const;


    llvm::Value * createThreadLocalInstance(const std::unique_ptr<KernelBuilder> & b) const;

    void initializeThreadLocalInstance(const std::unique_ptr<KernelBuilder> & b, llvm::ArrayRef<llvm::Value *> args);

    void finalizeThreadLocalInstance(const std::unique_ptr<KernelBuilder> & b, llvm::ArrayRef<llvm::Value *> args) const;

    void generateKernel(const std::unique_ptr<KernelBuilder> & b);

    void prepareKernel(const std::unique_ptr<KernelBuilder> & b);

    void prepareCachedKernel(const std::unique_ptr<KernelBuilder> & b);

    LLVM_READNONE std::string getCacheName(const std::unique_ptr<KernelBuilder> & b) const;

    LLVM_READNONE StreamSetPort getStreamPort(const llvm::StringRef name) const;

    LLVM_READNONE const Binding & getStreamBinding(const llvm::StringRef name) const;

    LLVM_READNONE ProcessingRate::RateValue getLowerBound(const Binding & binding) const;

    LLVM_READNONE ProcessingRate::RateValue getUpperBound(const Binding & binding) const;

    LLVM_READNONE bool requiresOverflow(const Binding & binding) const;

    /* Fill in any generated names / attributes for the kernel if their initialization is dependent on
     * settings / bindings added after construction. */
    virtual void finalizeKernel() { }

    enum MainMethodGenerationType {
        AddInternal
        , DeclareExternal
        , AddExternal
    };

    llvm::Function * addOrDeclareMainFunction(const std::unique_ptr<kernel::KernelBuilder> & b, const MainMethodGenerationType method);

    virtual bool hasStaticMain() const { return true; }

    virtual ~Kernel() = 0;

protected:

    static std::string getStringHash(const llvm::StringRef str);

    LLVM_READNONE std::string getDefaultFamilyName() const;

    virtual void addInternalProperties(const std::unique_ptr<KernelBuilder> &) { }

    virtual void linkExternalMethods(const std::unique_ptr<KernelBuilder> &) { }

    virtual void generateInitializeMethod(const std::unique_ptr<KernelBuilder> &) { }

    virtual void generateInitializeThreadLocalMethod(const std::unique_ptr<KernelBuilder> &) { }

    virtual void generateKernelMethod(const std::unique_ptr<KernelBuilder> &) = 0;

    virtual void generateFinalizeThreadLocalMethod(const std::unique_ptr<KernelBuilder> &) { }

    virtual void generateFinalizeMethod(const std::unique_ptr<KernelBuilder> &) { }

    virtual void addAdditionalFunctions(const std::unique_ptr<KernelBuilder> &) { }

    virtual void setInputStreamSetAt(const unsigned i, StreamSet * value);

    virtual void setOutputStreamSetAt(const unsigned i, StreamSet * value);

    virtual void setInputScalarAt(const unsigned i, Scalar * value);

    virtual void setOutputScalarAt(const unsigned i, Scalar * value);

    virtual std::vector<llvm::Value *> getFinalOutputScalars(const std::unique_ptr<KernelBuilder> & b);

    LLVM_READNONE const BindingMapEntry & getBinding(const BindingType type, const llvm::StringRef name) const;

    LLVM_READNONE llvm::Value * getAccessibleInputItems(const llvm::StringRef name) const {
        return getAccessibleInputItems(getBinding(BindingType::StreamInput, name).Index);
    }

    LLVM_READNONE llvm::Value * getAccessibleInputItems(const unsigned index) const {
        assert (index < mAccessibleInputItems.size());
        return mAccessibleInputItems[index];
    }

    LLVM_READNONE llvm::Value * getAvailableInputItems(const llvm::StringRef name) const {
        return getAvailableInputItems(getBinding(BindingType::StreamInput, name).Index);
    }

    LLVM_READNONE llvm::Value * getAvailableInputItems(const unsigned index) const {
        assert (index < mAvailableInputItems.size());
        return mAvailableInputItems[index];
    }

    LLVM_READNONE bool canSetTerminateSignal() const {
        return hasAttribute(Attribute::KindId::CanTerminateEarly) || hasAttribute(Attribute::KindId::MustExplicitlyTerminate);
    }

    LLVM_READNONE llvm::Value * getTerminationSignalPtr() const {
        return mTerminationSignalPtr;
    }

    LLVM_READNONE llvm::Value * getProcessedInputItemsPtr(const llvm::StringRef name) const {
        return getProcessedInputItemsPtr(getBinding(BindingType::StreamInput, name).Index);
    }

    LLVM_READNONE llvm::Value * getProcessedInputItemsPtr(const unsigned index) const {
        return mProcessedInputItemPtr[index];
    }

    LLVM_READNONE llvm::Value * getProducedOutputItemsPtr(const llvm::StringRef name) const {
        return getProducedOutputItemsPtr(getBinding(BindingType::StreamOutput, name).Index);
    }

    LLVM_READNONE llvm::Value * getProducedOutputItemsPtr(const unsigned index) const {
        return mProducedOutputItemPtr[index];
    }

    LLVM_READNONE llvm::Value * getWritableOutputItems(const llvm::StringRef name) const {
        return getWritableOutputItems(getBinding(BindingType::StreamOutput, name).Index);
    }

    LLVM_READNONE llvm::Value * getWritableOutputItems(const unsigned index) const {
        return mWritableOutputItems[index];
    }

    LLVM_READNONE llvm::Value * getConsumedOutputItems(const llvm::StringRef name) const {
        return getConsumedOutputItems(getBinding(BindingType::StreamOutput, name).Index);
    }

    LLVM_READNONE llvm::Value * getConsumedOutputItems(const unsigned index) const {
        return mConsumedOutputItems[index];
    }

    llvm::Value * getNumOfStrides() const {
        return mNumOfStrides;
    }

    LLVM_READNONE llvm::Value * isFinal() const {
        return mIsFinal;
    }

    // Constructor
    Kernel(const std::unique_ptr<KernelBuilder> & b,
           const TypeId typeId, std::string && kernelName,
           Bindings &&stream_inputs, Bindings &&stream_outputs,
           Bindings &&scalar_inputs, Bindings &&scalar_outputs,
           InternalScalars && internal_scalars);

private:

    void addInitializeDeclaration(const std::unique_ptr<KernelBuilder> & b) const;

    void callGenerateInitializeMethod(const std::unique_ptr<KernelBuilder> & b);

    void addInitializeThreadLocalDeclaration(const std::unique_ptr<KernelBuilder> & b) const;

    void callGenerateInitializeThreadLocalMethod(const std::unique_ptr<KernelBuilder> & b);

    void addDoSegmentDeclaration(const std::unique_ptr<KernelBuilder> & b) const;

    std::vector<llvm::Type *> getDoSegmentFields(const std::unique_ptr<KernelBuilder> & b) const;

    void callGenerateDoSegmentMethod(const std::unique_ptr<KernelBuilder> & b);

    void setDoSegmentProperties(const std::unique_ptr<KernelBuilder> & b, const llvm::ArrayRef<llvm::Value *> args);

    std::vector<llvm::Value *> getDoSegmentProperties(const std::unique_ptr<KernelBuilder> & b) const;

    void addFinalizeThreadLocalDeclaration(const std::unique_ptr<KernelBuilder> & b) const;

    void callGenerateFinalizeThreadLocalMethod(const std::unique_ptr<KernelBuilder> & b);

    void addFinalizeDeclaration(const std::unique_ptr<KernelBuilder> & b) const;

    void callGenerateFinalizeMethod(const std::unique_ptr<KernelBuilder> & b);

    void addBaseKernelProperties(const std::unique_ptr<KernelBuilder> & b);

    llvm::Function * getInitializeFunction(llvm::Module * const module) const;

    llvm::Function * getInitializeThreadLocalFunction(llvm::Module * const module) const;

    llvm::Function * getDoSegmentFunction(llvm::Module * const module) const;

    llvm::Function * getFinalizeThreadLocalFunction(llvm::Module * const module) const;

    llvm::Function * getFinalizeFunction(llvm::Module * const module) const;

    void constructStateTypes(const std::unique_ptr<KernelBuilder> & b);

    void initializeScalarMap(const std::unique_ptr<KernelBuilder> & b, const bool skipThreadLocal = false) const;

    unsigned getSharedScalarIndex(const llvm::StringRef name) const;

    llvm::Value * getScalarValuePtr(const llvm::StringRef name) const;

    void setTerminationSignalPtr(llvm::Value * ptr) {
        mTerminationSignalPtr = ptr;
    }

protected:

    mutable bool                    mIsGenerated;

    mutable llvm::Value *           mSharedHandle;
    mutable llvm::Value *           mThreadLocalHandle;

    llvm::Module *                  mModule;
    llvm::StructType *              mSharedStateType;
    llvm::StructType *              mThreadLocalStateType;

    Bindings                        mInputStreamSets;
    Bindings                        mOutputStreamSets;

    Bindings                        mInputScalars;
    Bindings                        mOutputScalars;
    InternalScalars                 mInternalScalars;

    llvm::Function *                mCurrentMethod;
    unsigned                        mStride;

    llvm::Value *                   mTerminationSignalPtr;
    llvm::Value *                   mIsFinal;
    llvm::Value *                   mNumOfStrides;
    llvm::Value *                   mThreadLocalPtr;

    std::vector<llvm::Value *>      mUpdatableProcessedInputItemPtr;
    std::vector<llvm::Value *>      mProcessedInputItemPtr;

    std::vector<llvm::Value *>      mAccessibleInputItems;
    std::vector<llvm::Value *>      mAvailableInputItems;
    std::vector<llvm::Value *>      mPopCountRateArray;
    std::vector<llvm::Value *>      mNegatedPopCountRateArray;

    std::vector<llvm::Value *>      mUpdatableProducedOutputItemPtr;
    std::vector<llvm::Value *>      mProducedOutputItemPtr;

    std::vector<llvm::Value *>      mWritableOutputItems;
    std::vector<llvm::Value *>      mConsumedOutputItems;

    mutable ScalarValueMap          mScalarValueMap;
    mutable BindingMap              mBindingMap;

    const std::string               mKernelName;
    const TypeId                    mTypeId;

    OwnedStreamSetBuffers           mStreamSetInputBuffers;
    OwnedStreamSetBuffers           mStreamSetOutputBuffers;

};

class SegmentOrientedKernel : public Kernel {
public:

    static bool classof(const Kernel * const k) {
        return k->getTypeId() == TypeId::SegmentOriented;
    }

    static bool classof(const void *) { return false; }

protected:

    SegmentOrientedKernel(const std::unique_ptr<KernelBuilder> & b,
                          std::string && kernelName,
                          Bindings &&stream_inputs,
                          Bindings &&stream_outputs,
                          Bindings &&scalar_parameters,
                          Bindings &&scalar_outputs,
                          InternalScalars && internal_scalars);
public:

    virtual void generateDoSegmentMethod(const std::unique_ptr<KernelBuilder> & b) = 0;

protected:

    void generateKernelMethod(const std::unique_ptr<KernelBuilder> & b) final;

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

    MultiBlockKernel(const std::unique_ptr<KernelBuilder> & b,
                     std::string && kernelName,
                     Bindings && stream_inputs,
                     Bindings && stream_outputs,
                     Bindings && scalar_parameters,
                     Bindings && scalar_outputs,
                     InternalScalars && internal_scalars);

    virtual void generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & b, llvm::Value * const numOfStrides) = 0;

private:

    MultiBlockKernel(const std::unique_ptr<KernelBuilder> & b,
                     const TypeId kernelTypId,
                     std::string && kernelName,
                     Bindings && stream_inputs,
                     Bindings && stream_outputs,
                     Bindings && scalar_parameters,
                     Bindings && scalar_outputs,
                     InternalScalars && internal_scalars);

private:

    void generateKernelMethod(const std::unique_ptr<KernelBuilder> & b) final;

};


class BlockOrientedKernel : public MultiBlockKernel {
public:

    static bool classof(const Kernel * const k) {
        return k->getTypeId() == TypeId::BlockOriented;
    }

    static bool classof(const void *) { return false; }

protected:

    void CreateDoBlockMethodCall(const std::unique_ptr<KernelBuilder> & b);

    // Each kernel builder subtype must provide its own logic for generating
    // doBlock calls.
    virtual void generateDoBlockMethod(const std::unique_ptr<KernelBuilder> & b) = 0;

    // Each kernel builder subtypre must also specify the logic for processing the
    // final block of stream data, if there is any special processing required
    // beyond simply calling the doBlock function.   In the case that the final block
    // processing may be trivially implemented by dispatching to the doBlock method
    // without additional preparation, the default generateFinalBlockMethod need
    // not be overridden.

    virtual void generateFinalBlockMethod(const std::unique_ptr<KernelBuilder> & b, llvm::Value * remainingItems);

    BlockOrientedKernel(const std::unique_ptr<KernelBuilder> & b,
                        std::string && kernelName,
                        Bindings && stream_inputs,
                        Bindings && stream_outputs,
                        Bindings && scalar_parameters,
                        Bindings && scalar_outputs,
                        InternalScalars && internal_scalars);

    llvm::Value * getRemainingItems(const std::unique_ptr<KernelBuilder> & b);

private:

    void annotateInputBindingsWithPopCountArrayAttributes();

    void incrementCountableItemCounts(const std::unique_ptr<KernelBuilder> & b);

    llvm::Value * getPopCountRateItemCount(const std::unique_ptr<KernelBuilder> & b, const ProcessingRate & rate);

    void generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & b, llvm::Value * const numOfStrides) final;

    void writeDoBlockMethod(const std::unique_ptr<KernelBuilder> & b);

    void writeFinalBlockMethod(const std::unique_ptr<KernelBuilder> & b, llvm::Value * remainingItems);

private:

    llvm::Function *            mDoBlockMethod;
    llvm::BasicBlock *          mStrideLoopBody;
    llvm::IndirectBrInst *      mStrideLoopBranch;
    llvm::PHINode *             mStrideLoopTarget;
    llvm::PHINode *             mStrideBlockIndex;
};

}

#endif
