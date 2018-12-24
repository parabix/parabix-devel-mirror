#ifndef OPTIMIZATIONBRANCH_H
#define OPTIMIZATIONBRANCH_H

#include <kernels/kernel.h>

namespace llvm { class Value; }

namespace kernel {

class OptimizationBranch final : public Kernel {
    friend class OptimizationBranchBuilder;
public:

    ~OptimizationBranch();

protected:

    OptimizationBranch(std::string && signature,
                       not_null<StreamSet *> condition,
                       not_null<Kernel *> trueKernel,
                       not_null<Kernel *> falseKernel,
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

    StreamSet * const       mCondition;
    Kernel * const          mTrueKernel;
    Kernel * const          mFalseKernel;

};

}

#endif // OPTIMIZATIONBRANCH_H
