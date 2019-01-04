#ifndef OPTIMIZATIONBRANCH_H
#define OPTIMIZATIONBRANCH_H

#include <kernels/kernel.h>

namespace llvm { class Value; }

namespace kernel {

class OptimizationBranch final : public Kernel {
    friend class OptimizationBranchBuilder;
public:

    const static std::string CONDITION_TAG;

    ~OptimizationBranch();

protected:

    OptimizationBranch(const std::unique_ptr<KernelBuilder> & b,
                       std::string && signature,
                       not_null<Relationship *> condition,
                       not_null<Kernel *> nonZeroKernel,
                       not_null<Kernel *> allZeroKernel,
                       Bindings && stream_inputs,
                       Bindings && stream_outputs,
                       Bindings && scalar_inputs,
                       Bindings && scalar_outputs);

    void linkExternalMethods(const std::unique_ptr<KernelBuilder> & b) final;

    void generateInitializeMethod(const std::unique_ptr<KernelBuilder> & b) final;

    void initializeInstance(const std::unique_ptr<KernelBuilder> & b, std::vector<llvm::Value *> & args) final;

    void generateKernelMethod(const std::unique_ptr<KernelBuilder> & b) final;

    void generateFinalizeMethod(const std::unique_ptr<KernelBuilder> & b) final;

    void addAdditionalFunctions(const std::unique_ptr<KernelBuilder> & b) final;

    llvm::Value * finalizeInstance(const std::unique_ptr<KernelBuilder> & b) final;

    void addInternalKernelProperties(const std::unique_ptr<kernel::KernelBuilder> & b) final;

    std::vector<llvm::Value *> getFinalOutputScalars(const std::unique_ptr<KernelBuilder> & b) final;

private:

    void callKernel(const std::unique_ptr<KernelBuilder> & b,
                    const Kernel * const kernel, llvm::Value * const first, llvm::Value * const last,
                    llvm::PHINode * const terminatedPhi);

private:

    Relationship * const        mCondition;
    Kernel * const              mTrueKernel;
    Kernel * const              mFalseKernel;
    std::vector<llvm::Value *>  mProcessedInputItems;
    std::vector<llvm::Value *>  mPartialAccessibleInputItems;

    std::vector<llvm::Value *>  mProducedOutputItems;
};

}

#endif // OPTIMIZATIONBRANCH_H
